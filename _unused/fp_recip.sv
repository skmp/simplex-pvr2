//
// fp_recip - clocked reciprocal y = 1/x; the single "inverter" in the design.
// ISP computes recip_C once and shares it with TSP, so all ddx/ddy become a
// multiply by recip_C (the C optimization) - no per-attribute divide.
//
// Two builds (selected by `SYNTHESIS):
//   sim   (Verilator): behavioral 1/x via the proven fp_div, 1-cycle latency.
//   synth (Quartus)  : altera_fp_rcp IP (LAT_RCP cycles), DSP/LUT-table based.
//                      No valid handshake on the IP -> a shift register delays
//                      'in_valid' by the pipeline latency to make 'out_valid'.
//
module fp_recip #(
    parameter LAT_RCP = 14   // altera_fp_rcp latency (cycles)
)(
    input         clk,
    input         reset,
    input         in_valid,
    input  [31:0] x,
    output        out_valid,
    output [31:0] y
);
`ifdef SYNTHESIS
    reg [31:0] rx;
    always @(posedge clk) if (in_valid) rx <= x;
    altera_fp_rcp u_rcp (.clk(clk), .areset(reset), .a(rx), .q(y));

    // rx latches 1 cycle after in_valid, then LAT_RCP through the IP.
    localparam VLEN = LAT_RCP + 1;
    reg [VLEN-1:0] vpipe;
    always @(posedge clk) begin
        if (reset) vpipe <= 0;
        else       vpipe <= {vpipe[VLEN-2:0], in_valid};
    end
    assign out_valid = vpipe[VLEN-1];
`else
    wire [31:0] recip_comb;
    fp_div u_div (.a(32'h3f800000 /*1.0f*/), .b(x), .y(recip_comb));
    reg ov; reg [31:0] yr;
    assign out_valid = ov; assign y = yr;
    always @(posedge clk) begin
        if (reset) ov <= 0;
        else begin ov <= in_valid; yr <= recip_comb; end
    end
`endif
endmodule
