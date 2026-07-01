//
// tex_mem - memory provider for the caches. Presents FOUR injected DDR read
// ports and arbitrates them onto ONE physical 64-bit read channel with fixed
// priority (only one read is in flight at a time in every phase, so this is
// sufficient):
//   d  (texture data)  > q  (texture VQ)  > isp (ISP param data$) > tsp (TSP param data$)
// Texture wins because it is on the pixel-critical path during shading; the
// param data$ fills are bursty and off that path.
//
// Two builds:
//   `ifdef SYNTHESIS  : drive the HPS f2sdram ram2 Avalon read port (a spare
//                       64-bit DDR3 channel; ram1 is used by the ddram wrapper).
//   `else (Verilator) : serve reads from a behavioral VRAM array so the whole
//                       shading pipeline is testable without the HPS bridge.
//                       Preload VRAM via the public `vram` array.
//
module tex_mem import tsp_pkg::*; #(
    parameter integer VRAM_WORDS = 1<<20   // 64-bit words (sim only)
) (
    input                clk,
    input                reset,

    // injected into the caches (fixed priority d > q > isp > tsp)
    input  ddr_rd_req_t  d_dreq,   output ddr_rd_resp_t d_dresp,   // texture data cache
    input  ddr_rd_req_t  q_dreq,   output ddr_rd_resp_t q_dresp,   // texture VQ cache
    input  ddr_rd_req_t  i_dreq,   output ddr_rd_resp_t i_dresp,   // ISP param data$
    input  ddr_rd_req_t  t_dreq,   output ddr_rd_resp_t t_dresp    // TSP param data$

`ifdef SYNTHESIS
    ,
    // HPS ram2 Avalon read channel
    output reg  [28:0] ram2_address,
    output reg  [7:0]  ram2_burstcount,
    input              ram2_waitrequest,
    input       [63:0] ram2_readdata,
    input              ram2_readdatavalid,
    output reg         ram2_read
`endif
);
    // ---- fixed-priority arbiter: d > q > isp > tsp ----
    // Only one read is in flight at a time, so we latch which port issued and
    // route the response back to it.
    localparam A_NONE=0, A_D=1, A_Q=2, A_I=3, A_T=4;
    reg [2:0] owner;

    wire d_wants = d_dreq.rd;
    wire q_wants = q_dreq.rd;
    wire i_wants = i_dreq.rd;
    wire t_wants = t_dreq.rd;

    // winner of this cycle's priority arbitration
    wire [2:0] win = d_wants ? A_D : q_wants ? A_Q : i_wants ? A_I : t_wants ? A_T : A_NONE;
    wire       want  = d_wants | q_wants | i_wants | t_wants;
    wire [28:0] waddr = d_wants ? d_dreq.addr : q_wants ? q_dreq.addr
                      : i_wants ? i_dreq.addr : t_dreq.addr;

`ifdef SYNTHESIS
    // ---- real DDR via HPS ram2 ----
    reg busy;   // a read is outstanding (addr accepted, awaiting data)
    always @(posedge clk) begin
        if (reset) begin busy<=0; ram2_read<=0; owner<=A_NONE; end
        else begin
            ram2_read <= 0;
            if (!busy && want && !ram2_waitrequest) begin
                ram2_read       <= 1'b1;
                ram2_address    <= waddr;
                ram2_burstcount <= 8'd1;
                owner           <= win;
                busy            <= 1'b1;
            end
            if (busy && ram2_readdatavalid) busy <= 1'b0;
        end
    end
    // a port is busy (cannot issue) while a read is outstanding OR a
    // higher-priority port is requesting this cycle.
    assign d_dresp.busy = busy;
    assign q_dresp.busy = busy | d_wants;
    assign i_dresp.busy = busy | d_wants | q_wants;
    assign t_dresp.busy = busy | d_wants | q_wants | i_wants;
    assign d_dresp.dout = ram2_readdata; assign q_dresp.dout = ram2_readdata;
    assign i_dresp.dout = ram2_readdata; assign t_dresp.dout = ram2_readdata;
    assign d_dresp.dready = ram2_readdatavalid & (owner==A_D);
    assign q_dresp.dready = ram2_readdatavalid & (owner==A_Q);
    assign i_dresp.dready = ram2_readdatavalid & (owner==A_I);
    assign t_dresp.dready = ram2_readdatavalid & (owner==A_T);

`else
    // ---- behavioral VRAM (Verilator) ----
    // 64-bit words; addr is a 64-bit-word address {4'b0011, wordidx[24:0]} - we
    // index by the low word bits. Preload via the public `vram` array.
    reg [63:0] vram [0:VRAM_WORDS-1] /*verilator public_flat_rw*/;
    reg [63:0] dout_r; reg dready_r; reg [2:0] owner_r;
    always @(posedge clk) begin
        if (reset) begin dready_r<=0; owner_r<=A_NONE; end
        else begin
            dready_r <= 0;
            if (want) begin
                dout_r  <= vram[waddr[$clog2(VRAM_WORDS)-1:0]];
                owner_r <= win;
                dready_r<= 1;
            end
        end
    end
    assign d_dresp.busy = 1'b0;
    assign q_dresp.busy = d_wants;
    assign i_dresp.busy = d_wants | q_wants;
    assign t_dresp.busy = d_wants | q_wants | i_wants;
    assign d_dresp.dout = dout_r; assign q_dresp.dout = dout_r;
    assign i_dresp.dout = dout_r; assign t_dresp.dout = dout_r;
    assign d_dresp.dready = dready_r & (owner_r==A_D);
    assign q_dresp.dready = dready_r & (owner_r==A_Q);
    assign i_dresp.dready = dready_r & (owner_r==A_I);
    assign t_dresp.dready = dready_r & (owner_r==A_T);
`endif
endmodule
