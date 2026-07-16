// Stub of sysmem_lite for lint/sim: exposes clock+reset and an internal
// behavioural DDR on ram1 so tile_engine_top can be exercised without the HPS.
module sysmem_lite (
    output         clock,
    output         reset_out,
    input          reset_hps_cold_req,
    input          reset_hps_warm_req,
    input          reset_core_req,

    input          ram1_clk,
    input   [28:0] ram1_address,
    input    [7:0] ram1_burstcount,
    output         ram1_waitrequest,
    output  [63:0] ram1_readdata,
    output         ram1_readdatavalid,
    input          ram1_read,
    input   [63:0] ram1_writedata,
    input    [7:0] ram1_byteenable,
    input          ram1_write,

    input          ram2_clk,
    input   [28:0] ram2_address,
    input    [7:0] ram2_burstcount,
    output         ram2_waitrequest,
    output  [63:0] ram2_readdata,
    output         ram2_readdatavalid,
    input          ram2_read,
    input   [63:0] ram2_writedata,
    input    [7:0] ram2_byteenable,
    input          ram2_write,

    input          vbuf_clk,
    input   [27:0] vbuf_address,
    input    [7:0] vbuf_burstcount,
    output         vbuf_waitrequest,
    output [127:0] vbuf_readdata,
    output         vbuf_readdatavalid,
    input          vbuf_read,
    input  [127:0] vbuf_writedata,
    input   [15:0] vbuf_byteenable,
    input          vbuf_write
);
    // For sim: the stub owns the clock. The TB pokes `clock` low/high via the
    // Verilated public net (see tile_tb.cpp) since clock is an output here.
    // We drive it from an internal free-running net toggled by the TB.
    reg clk_r /*verilator public*/ = 0;
    assign clock = clk_r;
    // simple reset: deassert after a couple cycles
    reg [1:0] rc = 0;
    assign reset_out = (rc != 2'b11);
    always @(posedge clock) if (rc != 2'b11) rc <= rc + 1'b1;

    // behavioural ram1 (accept in 1 cycle, no read data modelled here)
    reg busy = 0;
    always @(posedge clock) busy <= (ram1_read | ram1_write) & ~busy;
    assign ram1_waitrequest   = busy;
    assign ram1_readdata      = 64'd0;
    assign ram1_readdatavalid = 1'b0;

    assign ram2_waitrequest = 1'b0;
    assign ram2_readdata = 64'd0;
    assign ram2_readdatavalid = 1'b0;
    assign vbuf_waitrequest = 1'b0;
    assign vbuf_readdata = 128'd0;
    assign vbuf_readdatavalid = 1'b0;
endmodule
