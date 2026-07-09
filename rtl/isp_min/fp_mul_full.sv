//
// fp_mul_full - full-mantissa multiply on the extended setup float (xf) bus.
//
//   y = a * b,  all operands in the 41-bit xf format (see fp_ext.svh):
//       {sign[40], exp[39:32], mant[31:0]}, hidden-1 at mant bit31 -> 1.mant[30:0]
//
// Unlike fp_mul16 (which truncates each operand's significand to 16 bits so the
// core multiply maps to a single 18x18 DSP), this keeps the FULL 32-bit
// significands and forms the exact 32x32 -> 64 product, then retains the top 32
// bits as the result mantissa. This is the precision-critical path for triangle
// setup: area/plane sub-products no longer drop 16 mantissa bits.
//
// Cost: a 32x32 multiply (~4 DSP 18x18 partial products + adders on Cyclone V).
// Only a handful of these are needed in setup (not the per-pixel interpolator,
// which stays on fp_mul16), so the DSP cost is acceptable.
//
// Non-IEEE, matching the datapath: DaZ (exp==0 operand -> 0), no inf/NaN,
// truncate (no rounding), overflow saturates, underflow flushes. Combinational.
//
module fp_mul_full (
    input  [40:0] a,
    input  [40:0] b,
    output [40:0] y
);
    wire        sa = a[40];
    wire        sb = b[40];
    wire [7:0]  ea = a[39:32];
    wire [7:0]  eb = b[39:32];

    wire a_zero = (ea == 8'd0);   // DaZ
    wire b_zero = (eb == 8'd0);

    // full 32-bit significands, hidden 1 already present at bit31.
    wire [31:0] sig_a = a[31:0];
    wire [31:0] sig_b = b[31:0];

    wire res_sign = sa ^ sb;

    // exact 32x32 -> 64 product. With both leading ones at bit31, the product's
    // leading one lands at bit62 (product in [1,2)) or bit63 (product in [2,4)).
    wire [63:0] prod = sig_a * sig_b;

    // exponent: ea + eb - bias
    wire signed [10:0] e_sum = $signed({3'b0, ea}) + $signed({3'b0, eb}) - 11'sd127;

    // product in [2,4) -> bit63 set -> shift right 1, exp+1.
    wire top = prod[63];
    // keep the top 32 bits below (and including) the leading one as the mantissa.
    //   top : leading 1 at bit63 -> mant = prod[63:32]
    //   else: leading 1 at bit62 -> mant = prod[62:31]
    wire [31:0] mant  = top ? prod[63:32] : prod[62:31];
    wire signed [10:0] e_adj = top ? (e_sum + 11'sd1) : e_sum;

    wire is_zero   = a_zero | b_zero;
    wire underflow = (e_adj <= 0);
    wire overflow  = (e_adj >= 255);

    assign y = is_zero   ? {res_sign, 8'd0, 32'd0}
             : underflow ? {res_sign, 8'd0, 32'd0}
             : overflow  ? {res_sign, 8'hFE, 32'hFFFFFFFF}  // saturate finite
                         : {res_sign, e_adj[7:0], mant};
endmodule
