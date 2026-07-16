// Minimal ddram stub matching DC_MiSTer/rtl/ddram.sv port list, for lint/sim.
module ddram (
    output        DDRAM_CLK,
    input         DDRAM_BUSY,
    output  [7:0] DDRAM_BURSTCNT,
    output [28:0] DDRAM_ADDR,
    input  [63:0] DDRAM_DOUT,
    input         DDRAM_DOUT_READY,
    output        DDRAM_RD,
    output [63:0] DDRAM_DIN,
    output  [7:0] DDRAM_BE,
    output        DDRAM_WE,
    input         clk,
    input  [27:1] mem_addr,
    output reg [31:0] mem_dout,
    input  [31:0] mem_din,
    input         mem_rd,
    input   [3:0] mem_wr,
    input   [1:0] mem_chan,
    input         mem_16b,
    output        mem_busy
);
    // trivial single-cycle-accept behavioural model
    reg busy = 0;
    always @(posedge clk) busy <= (mem_rd | (|mem_wr)) & ~busy;
    assign mem_busy = busy;
    assign DDRAM_CLK = clk;
    assign DDRAM_BURSTCNT = 8'd1;
    assign DDRAM_ADDR = {4'b0011, mem_addr[27:3]};
    assign DDRAM_RD = mem_rd;
    assign DDRAM_DIN = {2{mem_din}};
    assign DDRAM_BE = {mem_wr, 4'b0};
    assign DDRAM_WE = |mem_wr;
    always @(*) mem_dout = DDRAM_DOUT[31:0];
endmodule
