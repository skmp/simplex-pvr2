//
// tex_cache_4p - 4-READ-PORT texture cache, 1024 lines x 32 BYTES (256-bit, =4x
// 64-bit words), direct-mapped, over the DDR3 raw 64-bit read port. Backs up to 4
// clients (the 4 bilinear corner fetchers) that can each present a lookup the
// SAME cycle.
//
// 4 read ports = 2x dual-port M10K (each has 2 read ports): the line DATA is a
// FULL COPY in both blocks (data_a, data_b) so any port reads any address. Ports
// 0,1 read data_a; ports 2,3 read data_b. Fills write BOTH copies + shared
// tag/valid so the copies stay identical.
//
// A 32-byte line holds 4 consecutive 64-bit words. On a miss the whole line is
// fetched as ONE 4-beat DDR burst (paying DDR latency once and pulling in the
// 2x2 bilinear neighbourhood), then the requested word is served.
//
// Client protocol per port i (unchanged): pulse creq[i].req with creq[i].waddr
// (64-bit-WORD addr) -> cresp[i].ack pulse + cresp[i].rdata (the 64-bit word;
// hit soon, miss after the burst). Line addr = waddr[28:2]; word-in-line =
// waddr[1:0]. index = line[9:0], tag = line[26:10].
//
// MISS handling (single DDR): hits on all 4 ports serve in parallel. Misses are
// processed one distinct LINE at a time (one 4-beat burst each); simultaneous
// misses to the SAME line are DEDUPED - one burst acks every matching port. The
// parent (tsp_shade_pp) stalls its pipe while any corner is busy.
//
module tex_cache_4p import tsp_pkg::*; (
    input                clk,
    input                reset,
    input  cache_req_t   creq  [0:3],
    output cache_resp_t  cresp [0:3],
    output ddr_rd_req_t  dreq,
    input  ddr_rd_resp_t dresp
);
    localparam integer NLINE = 1024;
    localparam integer IXW   = 10;                 // line index bits
    localparam integer LAW   = 27;                 // line-address width (waddr[28:2])
    localparam integer TAGW  = LAW - IXW;           // 17

    // full-copy storage: 256-bit (4-word) lines. data_a (ports 0,1), data_b (2,3).
    (* ramstyle = "M10K" *) reg [255:0] data_a [0:NLINE-1];
    (* ramstyle = "M10K" *) reg [255:0] data_b [0:NLINE-1];
    reg [TAGW-1:0] tags [0:NLINE-1];
    reg            vld  [0:NLINE-1];

    integer i;

    // ---- registered requests (one pending slot per port) ----
    reg            q_v   [0:3];
    reg [28:0]     q_addr[0:3];              // full 64-bit-word address
    wire [LAW-1:0]  q_line[0:3];             // 32-byte line address = waddr[28:2]
    wire [IXW-1:0]  q_ix  [0:3];
    wire [TAGW-1:0] q_tag [0:3];
    wire [1:0]      q_wsel[0:3];             // which 64-bit word within the line
    genvar gi;
    generate
      for (gi=0; gi<4; gi=gi+1) begin : qd
        assign q_line[gi] = q_addr[gi][28:2];
        assign q_ix[gi]   = q_line[gi][IXW-1:0];
        assign q_tag[gi]  = q_line[gi][LAW-1:IXW];
        assign q_wsel[gi] = q_addr[gi][1:0];
      end
    endgenerate

    // combinational per-port hit test + word extract (own copy)
    wire        hit [0:3];
    wire [63:0] hdat[0:3];
    assign hit[0] = q_v[0] && vld[q_ix[0]] && tags[q_ix[0]]==q_tag[0];
    assign hit[1] = q_v[1] && vld[q_ix[1]] && tags[q_ix[1]]==q_tag[1];
    assign hit[2] = q_v[2] && vld[q_ix[2]] && tags[q_ix[2]]==q_tag[2];
    assign hit[3] = q_v[3] && vld[q_ix[3]] && tags[q_ix[3]]==q_tag[3];
    assign hdat[0] = data_a[q_ix[0]][64*q_wsel[0] +: 64];
    assign hdat[1] = data_a[q_ix[1]][64*q_wsel[1] +: 64];
    assign hdat[2] = data_b[q_ix[2]][64*q_wsel[2] +: 64];
    assign hdat[3] = data_b[q_ix[3]][64*q_wsel[3] +: 64];

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

    localparam S_RST=0, S_IDLE=1, S_LOOK=2, S_MISS=3, S_FILL=4;
    reg [2:0] st;
    reg [IXW:0] rst_i;                        // reset sweep counter (clears vld[])
    reg [LAW-1:0]  m_line;                   // line being filled
    reg [IXW-1:0]  m_ix;
    reg [TAGW-1:0] m_tag;
    reg [1:0]      m_beat;                    // 0..3 burst beat
    reg [255:0]    m_acc;                     // assembled 256-bit line
    // base 64-bit-word address of the line = {m_line, 2'b00}
    wire [28:0] m_base = {m_line, 2'b00};

    // lowest-index pending MISS port (fm[2]=1 => none)
    wire [2:0] fm = (q_v[0] && !hit[0]) ? 3'd0 :
                    (q_v[1] && !hit[1]) ? 3'd1 :
                    (q_v[2] && !hit[2]) ? 3'd2 :
                    (q_v[3] && !hit[3]) ? 3'd3 : 3'b100;

    always @(posedge clk) begin
        if (reset) begin
            st <= S_RST; rd_r <= 0; rst_i <= 0;
            for (i=0;i<4;i=i+1) begin ack_r[i]<=0; q_v[i]<=0; end
        end else begin
            for (i=0;i<4;i=i+1) ack_r[i] <= 1'b0;
            rd_r <= 1'b0;

            // accept new requests into any free port slot (safe during S_RST too)
            for (i=0;i<4;i=i+1)
                if (!q_v[i] && creq[i].req) begin q_v[i]<=1'b1; q_addr[i]<=creq[i].waddr; end

            case (st)
            // clear vld[] one entry/cycle after reset (M10K-friendly, no big loop)
            S_RST: begin
                vld[rst_i[IXW-1:0]] <= 1'b0;
                if (rst_i == NLINE-1) st <= S_IDLE;
                else rst_i <= rst_i + 1'b1;
            end
            S_IDLE: if (q_v[0]||q_v[1]||q_v[2]||q_v[3]) st <= S_LOOK;
            S_LOOK: begin
                // serve all current hits
                for (i=0;i<4;i=i+1) if (hit[i]) begin
                    rdata_r[i] <= hdat[i]; ack_r[i] <= 1'b1; q_v[i] <= 1'b0;
                end
                if (fm[2]) st <= S_IDLE;      // no misses left
                else begin
                    m_line <= q_line[fm[1:0]];
                    m_ix   <= q_ix[fm[1:0]];
                    m_tag  <= q_tag[fm[1:0]];
                    m_beat <= 2'd0;
                    st     <= S_MISS;
                end
            end
            // burst-read the 4 words of the line, one 64-bit beat per beat index.
            S_MISS: if (!dresp.busy) begin
                rd_r    <= 1'b1;
                addr_r  <= {4'b0011, m_base[24:0]};   // burst base = line's word 0
                burst_r <= 8'd4;
                st      <= S_FILL;
            end
            S_FILL: if (dresp.dready) begin
                m_acc[64*m_beat +: 64] <= dresp.dout;
                if (m_beat == 2'd3) begin
                    // commit the line (fold in the just-arrived last beat)
                    data_a[m_ix] <= { dresp.dout, m_acc[191:0] };
                    data_b[m_ix] <= { dresp.dout, m_acc[191:0] };
                    tags[m_ix]   <= m_tag;
                    vld [m_ix]   <= 1'b1;
                    // dedup-ack every pending port whose line matches; extract its
                    // requested word from the assembled line.
                    for (i=0;i<4;i=i+1)
                        if (q_v[i] && !ack_r[i] && q_line[i]==m_line) begin
                            rdata_r[i] <= (q_wsel[i]==2'd3) ? dresp.dout
                                        : m_acc[64*q_wsel[i] +: 64];
                            ack_r[i]   <= 1'b1; q_v[i] <= 1'b0;
                        end
                    st <= S_LOOK;
                end else m_beat <= m_beat + 2'd1;
            end
            endcase
        end
    end
endmodule
