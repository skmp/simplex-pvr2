//
// fp_div - IEEE-754 single-precision divider (combinational), RNE.
//
// Computes y = a / b. Handles normals and signed zero numerator. Division by
// zero is not expected (refsw2 forces triangle area C=1 when zero before the
// divide), but is clamped to a signed zero result here rather than producing
// Inf, to keep simulation deterministic.
//
// Algorithm: integer long division of the 24-bit significands. We left-shift
// the dividend by 26 so the quotient lands with the leading 1 plus guard/round
// bits; the remainder gives the sticky bit.
//
module fp_div (
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

    wire a_zero = (ea == 8'd0);
    wire b_zero = (eb == 8'd0);

    wire [23:0] sig_a = {1'b1, ma};
    wire [23:0] sig_b = {1'b1, mb};

    wire res_sign = sa ^ sb;

    // Dividend scaled up: sig_a (24 bits) << 26.
    // quot = (sig_a<<26)/sig_b. Since sig_a,sig_b in [2^23,2^24), the ratio is
    // in (0.5,2), so quot is in (2^25, 2^27): up to 27 bits. Leading 1 is at
    // bit 26 when sig_a>=sig_b (ratio>=1), else at bit 25.
    wire [49:0] num = {sig_a, 26'd0};
    wire [26:0] quot = num / {3'b0, sig_b};        // 27-bit quotient
    wire [49:0] rem  = num - (quot * {23'd0, sig_b});
    wire        sticky_rem = |rem;

    // exponent
    wire signed [10:0] e_sub = $signed({3'b0, ea}) - $signed({3'b0, eb}) + 11'sd127;

    // Normalize: if bit26 set, ratio in [1,2) at 2^26 scale.
    //   top : significand = quot[26:3] (24 bits incl hidden), G=quot[2],R=quot[1],S=quot[0]|rem
    //   !top: leading 1 at bit25, significand = quot[25:2], G=quot[1],R=quot[0],S=rem
    wire top = quot[26];
    wire [23:0] sig_q   = top ? quot[26:3] : quot[25:2];
    wire        guard   = top ? quot[2]    : quot[1];
    wire        round   = top ? quot[1]    : quot[0];
    wire        sticky  = top ? (quot[0] | sticky_rem) : sticky_rem;
    // top (ratio>=1): exponent unchanged. !top (ratio<1): exponent -1.
    wire signed [10:0] e_adj = top ? e_sub : (e_sub - 11'sd1);

    // sig_q[23] is the hidden bit (=1). mantissa = sig_q[22:0].
    wire [22:0] mant = sig_q[22:0];
    wire        lsb  = sig_q[0];
    wire round_up = guard & (round | sticky | lsb);

    wire [23:0] mant_rnd = {1'b0, mant} + (round_up ? 24'd1 : 24'd0);
    wire        mant_carry = mant_rnd[23];
    wire [22:0] final_mant = mant_carry ? 23'd0 : mant_rnd[22:0];
    wire signed [10:0] e_final = mant_carry ? (e_adj + 11'sd1) : e_adj;

    wire is_zero    = a_zero;          // 0 / x = 0
    wire div_by_zero = b_zero;
    wire underflow  = (e_final <= 0);
    wire overflow   = (e_final >= 255);

    assign y = (is_zero | div_by_zero) ? {res_sign, 31'd0}
             : underflow               ? {res_sign, 31'd0}
             : overflow                ? {res_sign, 8'hFF, 23'd0}
                                       : {res_sign, e_final[7:0], final_mant};
endmodule
