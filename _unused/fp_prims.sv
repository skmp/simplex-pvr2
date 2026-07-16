// Thin wrapper exposing the three FP primitives for unit fuzzing.
module fp_prims (
    input  [31:0] a,
    input  [31:0] b,
    input         sub,
    output [31:0] y_add,
    output [31:0] y_mul,
    output [31:0] y_div
);
    fp_add u_add (.a(a), .b_in(b), .sub(sub), .y(y_add));
    fp_mul u_mul (.a(a), .b(b), .y(y_mul));
    fp_div u_div (.a(a), .b(b), .y(y_div));
endmodule
