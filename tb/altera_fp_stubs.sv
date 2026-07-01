//
// altera_fp_stubs - Verilator-only behavioral stand-ins for the Altera FP IP.
//
// The real altera_fp_* cores are encrypted VHDL/.vo that Verilator can't
// elaborate. These stubs implement the SAME ports and the SAME pipeline
// latency using the project's proven combinational FP units, so the synth-path
// wrappers (fmac_seq / fp_recip under `SYNTHESIS) can be exercised in Verilator
// to validate the latency/valid-alignment bookkeeping and the neg/sub sign
// handling. The arithmetic here is round-to-nearest-even (the project units);
// the real IP rounds ties-away-from-zero, so on hardware expect ~1 ULP extra
// drift vs. these stubs - which is exactly why the golden check uses tolerance.
//
// LAT_* MUST match the latencies configured in fmac_seq / fp_recip.
//
// NOTE: compile these ONLY for the Verilator synth-path test, never for the
// Quartus build (Quartus uses the real IP via the .qip files).

module altera_fp_mul #(parameter LAT=7) (
    input clk, input areset, input [31:0] a, input [31:0] b, output [31:0] q
);
    wire [31:0] p; fp_mul u(.a(a), .b(b), .y(p));
    reg [31:0] pipe [0:LAT-1]; integer i;
    always @(posedge clk) begin
        pipe[0] <= p;
        for (i=1;i<LAT;i=i+1) pipe[i] <= pipe[i-1];
    end
    assign q = pipe[LAT-1];
endmodule

module altera_fp_add #(parameter LAT=7) (
    input clk, input areset, input [31:0] a, input [31:0] b, output [31:0] q
);
    wire [31:0] s; fp_add u(.a(a), .b_in(b), .sub(1'b0), .y(s));
    reg [31:0] pipe [0:LAT-1]; integer i;
    always @(posedge clk) begin
        pipe[0] <= s;
        for (i=1;i<LAT;i=i+1) pipe[i] <= pipe[i-1];
    end
    assign q = pipe[LAT-1];
endmodule

module altera_fp_short_add #(parameter LAT=7) (
    input clk, input areset, input [31:0] a, input [31:0] b, output [31:0] q
);
    wire [31:0] s; fp_add u(.a(a), .b_in(b), .sub(1'b0), .y(s));
    reg [31:0] pipe [0:LAT-1]; integer i;
    always @(posedge clk) begin
        pipe[0] <= s;
        for (i=1;i<LAT;i=i+1) pipe[i] <= pipe[i-1];
    end
    assign q = pipe[LAT-1];
endmodule

module altera_fp_rcp #(parameter LAT=14) (
    input clk, input areset, input [31:0] a, output [31:0] q
);
    wire [31:0] r; fp_div u(.a(32'h3f800000), .b(a), .y(r));
    reg [31:0] pipe [0:LAT-1]; integer i;
    always @(posedge clk) begin
        pipe[0] <= r;
        for (i=1;i<LAT;i=i+1) pipe[i] <= pipe[i-1];
    end
    assign q = pipe[LAT-1];
endmodule
