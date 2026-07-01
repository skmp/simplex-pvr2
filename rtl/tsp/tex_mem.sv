//
// tex_mem - memory provider for the two texture caches. Presents two injected
// DDR read ports (data + VQ), arbitrates them onto ONE physical 64-bit read
// channel (the shade path issues at most one outstanding read, so a simple
// fixed-priority arbiter suffices).
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

    // injected into the two caches
    input  ddr_rd_req_t  d_dreq,   output ddr_rd_resp_t d_dresp,   // data cache
    input  ddr_rd_req_t  q_dreq,   output ddr_rd_resp_t q_dresp    // VQ cache

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
    // ---- fixed-priority arbiter: data cache wins over VQ ----
    // Only one read is in flight (1 pixel/iter), so we latch which port issued
    // and route the response back to it.
    localparam A_NONE=0, A_D=1, A_Q=2;
    reg [1:0] owner;

    wire d_wants = d_dreq.rd;
    wire q_wants = q_dreq.rd;

`ifdef SYNTHESIS
    // ---- real DDR via HPS ram2 ----
    reg busy;   // a read is outstanding (addr accepted, awaiting data)
    // present a read when a port wants one and none is outstanding
    wire        want   = (d_wants | q_wants) & ~busy;
    wire [28:0] waddr  = d_wants ? d_dreq.addr : q_dreq.addr;

    always @(posedge clk) begin
        if (reset) begin busy<=0; ram2_read<=0; owner<=A_NONE; end
        else begin
            ram2_read <= 0;
            if (!busy && want && !ram2_waitrequest) begin
                ram2_read       <= 1'b1;
                ram2_address    <= waddr;
                ram2_burstcount <= 8'd1;
                owner           <= d_wants ? A_D : A_Q;
                busy            <= 1'b1;
            end
            if (busy && ram2_readdatavalid) busy <= 1'b0;
        end
    end
    // busy=1 while a read is outstanding -> caches see !dresp.busy only when idle
    assign d_dresp.busy   = busy | (q_wants & ~d_wants); // data has priority
    assign q_dresp.busy   = busy | d_wants;
    assign d_dresp.dout   = ram2_readdata;
    assign q_dresp.dout   = ram2_readdata;
    assign d_dresp.dready = ram2_readdatavalid & (owner==A_D);
    assign q_dresp.dready = ram2_readdatavalid & (owner==A_Q);

`else
    // ---- behavioral VRAM (Verilator) ----
    // 64-bit words; addr is a 64-bit-word address {4'b0011, wordidx[24:0]} - we
    // index by the low word bits. Preload via the public `vram` array.
    reg [63:0] vram [0:VRAM_WORDS-1] /*verilator public_flat_rw*/;
    reg [63:0] dout_r; reg dready_r; reg [1:0] owner_r;
    wire        want  = (d_wants | q_wants);
    wire [28:0] waddr = d_wants ? d_dreq.addr : q_dreq.addr;
    always @(posedge clk) begin
        if (reset) begin dready_r<=0; owner_r<=A_NONE; end
        else begin
            dready_r <= 0;
            if (want) begin
                dout_r  <= vram[waddr[$clog2(VRAM_WORDS)-1:0]];
                owner_r <= d_wants ? A_D : A_Q;
                dready_r<= 1;
            end
        end
    end
    assign d_dresp.busy   = 1'b0;
    assign q_dresp.busy   = d_wants;            // data priority
    assign d_dresp.dout   = dout_r;
    assign q_dresp.dout   = dout_r;
    assign d_dresp.dready = dready_r & (owner_r==A_D);
    assign q_dresp.dready = dready_r & (owner_r==A_Q);
`endif
endmodule
