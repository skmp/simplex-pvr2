// fp_mul_c9_tb_top - expose fp_mul_c9 combinationally for a C++ fuzz vs float*int.
module fp_mul_c9_tb_top (
    input  [31:0] f,
    input  signed [8:0] k,
    output [31:0] y
);
    fp_mul_c9 u (.f(f), .k(k), .y(y));
endmodule
