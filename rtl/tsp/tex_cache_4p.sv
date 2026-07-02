//
// tex_cache_4p - 4-READ-PORT texture cache (64-entry x 64-bit, direct-mapped)
// over the DDR3 raw 64-bit read port. Backs up to 4 clients (e.g. the 4 bilinear
// corner fetchers) that can each present a lookup the SAME cycle.
//
// 4 read ports are built from 2x dual-port M10K (each M10K has 2 read ports):
// the cache DATA is stored as a FULL COPY in both blocks (data_a, data_b) so any
// port can read any address. Ports 0,1 read data_a; ports 2,3 read data_b. Fills
// write BOTH copies (and the shared tag/valid) so the copies stay identical.
//
// Client protocol per port i (same as tex_cache): pulse creq[i].req with
// creq[i].waddr (64-bit-WORD addr) -> cresp[i].ack pulse + cresp[i].rdata (hit:
// soon; miss: after the DDR read completes).
//
// MISS handling with a single DDR port: hits on all 4 ports are served in
// parallel. Misses are processed one distinct address at a time (a single 64-bit
// DDR read each). Simultaneous misses to the SAME line word are DEDUPED - one DDR
// read fills the line and acks every port whose waddr matches it. The parent
// (tsp_shade_pp) stalls its whole pipe while any corner is busy, so serializing
// the distinct-address misses is fine.
//
// index = waddr[5:0], tag = waddr[28:6]; DDR word at 0x30000000
// (dreq.addr = {4'b0011, waddr[24:0]}).
//
module tex_cache_4p import tsp_pkg::*; (
    input                clk,
    input                reset,
    // 4 client ports
    input  cache_req_t   creq  [0:3],
    output cache_resp_t  cresp [0:3],
    // injected DDR3 read port (single, shared)
    output ddr_rd_req_t  dreq,
    input  ddr_rd_resp_t dresp
);
    localparam integer NLINE = 64;
    localparam integer IXW   = 6;
    localparam integer TAGW  = 29 - IXW;

    // full-copy storage: data_a (ports 0,1), data_b (ports 2,3); shared tag/valid.
    (* ramstyle = "M10K" *) reg [63:0] data_a [0:NLINE-1];
    (* ramstyle = "M10K" *) reg [63:0] data_b [0:NLINE-1];
    reg [TAGW-1:0] tags [0:NLINE-1];
    reg            vld  [0:NLINE-1];

    integer i, k;

    // ---- registered requests (latched when idle & any req) ----
    reg            q_v   [0:3];         // this port has a pending (unacked) request
    reg [28:0]     q_addr[0:3];
    wire [IXW-1:0]  q_ix  [0:3];
    wire [TAGW-1:0] q_tag [0:3];
    genvar gi;
    generate
      for (gi=0; gi<4; gi=gi+1) begin : qd
        assign q_ix[gi]  = q_addr[gi][IXW-1:0];
        assign q_tag[gi] = q_addr[gi][28:IXW];
      end
    endgenerate

    // combinational per-port hit test (each port reads its own copy)
    // ports 0,1 -> data_a ; ports 2,3 -> data_b
    wire        hit [0:3];
    wire [63:0] hdat[0:3];
    assign hit[0]  = q_v[0] && vld[q_ix[0]] && tags[q_ix[0]]==q_tag[0];
    assign hit[1]  = q_v[1] && vld[q_ix[1]] && tags[q_ix[1]]==q_tag[1];
    assign hit[2]  = q_v[2] && vld[q_ix[2]] && tags[q_ix[2]]==q_tag[2];
    assign hit[3]  = q_v[3] && vld[q_ix[3]] && tags[q_ix[3]]==q_tag[3];
    assign hdat[0] = data_a[q_ix[0]];
    assign hdat[1] = data_a[q_ix[1]];
    assign hdat[2] = data_b[q_ix[2]];
    assign hdat[3] = data_b[q_ix[3]];

    // ---- registered outputs ----
    reg        ack_r  [0:3]; reg [63:0] rdata_r [0:3];
    reg        rd_r;   reg [28:0] addr_r; reg [7:0] burst_r;
    generate
      for (gi=0; gi<4; gi=gi+1) begin : od
        assign cresp[gi].ack   = ack_r[gi];
        assign cresp[gi].rdata = rdata_r[gi];
      end
    endgenerate
    assign dreq.rd    = rd_r;
    assign dreq.addr  = addr_r;
    assign dreq.burst = burst_r;

    // miss servicing
    localparam S_IDLE=0, S_LOOK=1, S_MISS=2, S_FILL=3;
    reg [1:0] st;
    reg [28:0] m_addr;                  // the distinct miss address being filled
    reg [IXW-1:0]  m_ix;
    reg [TAGW-1:0] m_tag;

    // lowest-index port that still has a pending MISS (not hit this cycle):
    // fm[2]=1 => none; fm[1:0]=port index. Its address is the next line to fill.
    wire [2:0] fm = (q_v[0] && !hit[0]) ? 3'd0 :
                    (q_v[1] && !hit[1]) ? 3'd1 :
                    (q_v[2] && !hit[2]) ? 3'd2 :
                    (q_v[3] && !hit[3]) ? 3'd3 : 3'b100;

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; rd_r <= 0;
            for (i=0;i<4;i=i+1) begin ack_r[i]<=0; q_v[i]<=0; end
            for (i=0;i<NLINE;i=i+1) vld[i] <= 1'b0;
        end else begin
            for (i=0;i<4;i=i+1) ack_r[i] <= 1'b0;
            rd_r <= 1'b0;

            // accept new requests into any FREE (non-pending) port slot.
            for (i=0;i<4;i=i+1) begin
                if (!q_v[i] && creq[i].req) begin
                    q_v[i]    <= 1'b1;
                    q_addr[i] <= creq[i].waddr;
                end
            end

            case (st)
            // wait for at least one pending request, then evaluate hits/misses.
            S_IDLE: begin
                if (q_v[0]||q_v[1]||q_v[2]||q_v[3]) st <= S_LOOK;
            end
            // serve ALL current hits this cycle; if misses remain, go fill one.
            S_LOOK: begin
                for (i=0;i<4;i=i+1) begin
                    if (hit[i]) begin
                        rdata_r[i] <= hdat[i]; ack_r[i] <= 1'b1; q_v[i] <= 1'b0;
                    end
                end
                if (fm[2]) begin
                    // no misses left; if new reqs arrived, loop, else idle.
                    st <= S_IDLE;
                end else begin
                    m_addr <= q_addr[fm[1:0]];
                    m_ix   <= q_addr[fm[1:0]][IXW-1:0];
                    m_tag  <= q_addr[fm[1:0]][28:IXW];
                    st     <= S_MISS;
                end
            end
            // single 64-bit DDR read for the chosen miss line.
            S_MISS: if (!dresp.busy) begin
                rd_r    <= 1'b1;
                addr_r  <= {4'b0011, m_addr[24:0]};
                burst_r <= 8'd1;
                st      <= S_FILL;
            end
            // fill both copies + tag/valid; DEDUP-ack every pending port whose
            // waddr matches this filled line word, then re-evaluate.
            S_FILL: if (dresp.dready) begin
                data_a[m_ix] <= dresp.dout;
                data_b[m_ix] <= dresp.dout;
                tags[m_ix]   <= m_tag;
                vld [m_ix]   <= 1'b1;
                for (i=0;i<4;i=i+1) begin
                    if (q_v[i] && !ack_r[i] && q_addr[i]==m_addr) begin
                        rdata_r[i] <= dresp.dout; ack_r[i] <= 1'b1; q_v[i] <= 1'b0;
                    end
                end
                st <= S_LOOK;   // re-look: remaining pending ports (hits now valid
                                // for this line, or other miss addresses)
            end
            endcase
        end
    end
endmodule
