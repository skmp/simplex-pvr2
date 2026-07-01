//
// data_cache256 - direct-mapped read cache with a 32-BYTE (256-bit) line, over
// the raw 64-bit DDR3 read port. A line is 4 x 64-bit DDR beats. Single client,
// one outstanding miss at a time (the ISP/TSP param paths are serialized). The
// DDR3 read port is INJECTED (dreq/dresp) exactly like tex_cache, and the client
// port uses the 256-bit bundle (cache_req256_t/cache_resp256_t).
//
//   client: pulse creq.req with creq.laddr (32-BYTE-line addr = byte_addr>>5)
//           -> cresp.ack pulse + cresp.rdata (256-bit line: 8 x 32-bit words,
//              word w = rdata[32*w +: 32]).
//
// line addr laddr[26:0]; index = laddr[NIXW-1:0], tag = laddr[26:NIXW].
// The DDR 64-bit-word address of beat b of line laddr is  (laddr<<2)+b, mapped
// to dreq.addr = {4'b0011, wordaddr[24:0]} (DDR base 0x30000000), matching tex_cache.
//
module data_cache256 import tsp_pkg::*; #(
    parameter integer NLINE = 256           // lines; 256 x 256b = 64 Kb ~= 7 M10K
) (
    input                 clk,
    input                 reset,
    // client port (256-bit line)
    input  cache_req256_t  creq,
    output cache_resp256_t cresp,
    // injected DDR3 read port (64-bit beats)
    output ddr_rd_req_t   dreq,
    input  ddr_rd_resp_t  dresp
);
    localparam integer IXW  = $clog2(NLINE);
    localparam integer TAGW = 27 - IXW;

    (* ramstyle = "M10K" *) reg [255:0] data [0:NLINE-1];
    reg [TAGW-1:0] tags [0:NLINE-1];
    reg            vld  [0:NLINE-1];

    wire [IXW-1:0]  r_ix  = creq.laddr[IXW-1:0];
    wire [TAGW-1:0] r_tag = creq.laddr[26:IXW];

    localparam S_RST=0, S_IDLE=1, S_LOOK=2, S_MISS=3, S_FILL=4;
    reg [2:0] st;
    reg [IXW:0] rst_i;         // reset sweep counter (clears vld[])
    reg [IXW-1:0]  m_ix;
    reg [TAGW-1:0] m_tag;
    reg [26:0]     m_laddr;
    reg [1:0]      beat;        // which 64-bit beat of the 256-bit line
    reg [255:0]    fill;        // assembled line

    // registered outputs
    reg         ack_r;  reg [255:0] rdata_r;
    reg         rd_r;   reg [28:0]  addr_r; reg [7:0] burst_r;
    assign cresp.ack   = ack_r;
    assign cresp.rdata = rdata_r;
    assign dreq.rd     = rd_r;
    assign dreq.addr   = addr_r;
    assign dreq.burst  = burst_r;

    // 64-bit-word address of beat `b` of line m_laddr = (m_laddr<<2) + b
    wire [28:0] beat_word = {m_laddr, 2'b00} + beat;

    always @(posedge clk) begin
        if (reset) begin
            st <= S_RST; ack_r <= 0; rd_r <= 0; beat <= 0; rst_i <= 0;
        end else begin
            ack_r <= 1'b0;
            rd_r  <= 1'b0;
            case (st)
            // sweep vld[] clear after reset (one entry/cycle; M10K-friendly)
            S_RST: begin
                vld[rst_i[IXW-1:0]] <= 1'b0;
                if (rst_i == NLINE-1) st <= S_IDLE;
                else rst_i <= rst_i + 1'b1;
            end
            S_IDLE: if (creq.req) begin
                m_ix    <= r_ix;
                m_tag   <= r_tag;
                m_laddr <= creq.laddr;
                st      <= S_LOOK;
            end
            S_LOOK: begin
                if (vld[m_ix] && tags[m_ix] == m_tag) begin
                    rdata_r <= data[m_ix];
                    ack_r   <= 1'b1;
                    st      <= S_IDLE;
                end else begin
                    beat <= 2'd0;
                    st   <= S_MISS;
                end
            end
            // issue a single-beat read for the current beat, wait for it in FILL
            S_MISS: if (!dresp.busy) begin
                rd_r    <= 1'b1;
                addr_r  <= {4'b0011, beat_word[24:0]};
                burst_r <= 8'd1;
                st      <= S_FILL;
            end
            S_FILL: if (dresp.dready) begin
                fill[64*beat +: 64] <= dresp.dout;
                if (beat == 2'd3) begin
                    // last beat: commit the line (use combinational assembly so
                    // the just-arrived beat is included)
                    data[m_ix] <= { dresp.dout, fill[191:0] };
                    tags[m_ix] <= m_tag;
                    vld [m_ix] <= 1'b1;
                    rdata_r    <= { dresp.dout, fill[191:0] };
                    ack_r      <= 1'b1;
                    st         <= S_IDLE;
                end else begin
                    beat <= beat + 2'd1;
                    st   <= S_MISS;
                end
            end
            endcase
        end
    end
endmodule
