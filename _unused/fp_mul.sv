//
// fp_mul - IEEE-754 single-precision multiplier (combinational), RNE.
//
// Handles normals and signed zero. Subnormal inputs are flushed by treating a
// zero biased-exponent operand as zero (the setup math here multiplies coords
// and small attribute values that stay in the normal range). Inf/NaN are not
// specially handled; valid setup inputs never reach them.
//
module fp_mul (
    input  [31:0] a,
    input  [31:0] b,
    output [31:0] y
);
    wire        sa = a[31];
    wire        sb = b[31];
    wire [7:0]  ea = a[30:23];
    wire [7:0]  eb = b[30:23];
    wire [22:0] ma = a[22:0];
    wire [22:0] mb = b[22:0];

    wire a_zero = (ea == 8'd0);   // flush subnormals to zero
    wire b_zero = (eb == 8'd0);

    wire [23:0] sig_a = {1'b1, ma};
    wire [23:0] sig_b = {1'b1, mb};

    wire res_sign = sa ^ sb;

    // 24x24 -> 48 bit product
    wire [47:0] prod = sig_a * sig_b;

    // exponent: ea + eb - bias
    wire signed [10:0] e_sum = $signed({3'b0, ea}) + $signed({3'b0, eb}) - 11'sd127;

    // product is in [1,4): bit47 set means >=2 -> shift right 1, exp+1
    wire        top = prod[47];
    // mantissa field: take 23 bits below the leading one, plus G/R/S
    // If top: leading 1 at bit47 -> mant = prod[46:24], guard=prod[23], round=prod[22], sticky=|prod[21:0]
    // else  : leading 1 at bit46 -> mant = prod[45:23], guard=prod[22], round=prod[21], sticky=|prod[20:0]
    wire [22:0] mant   = top ? prod[46:24] : prod[45:23];
    wire        guard  = top ? prod[23]    : prod[22];
    wire        round  = top ? prod[22]    : prod[21];
    wire        sticky = top ? (|prod[21:0]) : (|prod[20:0]);
    wire signed [10:0] e_adj = top ? (e_sum + 11'sd1) : e_sum;

    wire lsb = mant[0];
    wire round_up = guard & (round | sticky | lsb);

    wire [23:0] mant_rnd = {1'b0, mant} + (round_up ? 24'd1 : 24'd0);
    wire        mant_carry = mant_rnd[23];
    wire [22:0] final_mant = mant_carry ? mant_rnd[22:0] /*=0*/ : mant_rnd[22:0];
    wire signed [10:0] e_final = mant_carry ? (e_adj + 11'sd1) : e_adj;

    wire is_zero = a_zero | b_zero;
    // underflow/overflow clamping (best-effort; not expected in setup math)
    wire underflow = (e_final <= 0);
    wire overflow  = (e_final >= 255);

    assign y = is_zero    ? {res_sign, 31'd0}
             : underflow  ? {res_sign, 31'd0}
             : overflow   ? {res_sign, 8'hFF, 23'd0}
                          : {res_sign, e_final[7:0], final_mant};
endmodule
