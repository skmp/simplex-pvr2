//
// fp_add - IEEE-754 single-precision adder (combinational), round-to-nearest-even.
//
// Subtraction is a = a, b = b ^ sign-flip handled by caller (or set `sub`).
// Handles: normals, signed zero, basic subnormal flush-to-zero on inputs is
// NOT done (subnormals are treated as their actual value via leading-zero
// normalization of the significand sum). Inf/NaN are passed through in a
// best-effort manner; the setup math here never produces them for valid input.
//
// This is plain combinational logic intended for simulation parity with the C
// golden model. On real silicon it would be pipelined; latency is abstracted
// behind a 1-cycle register in the setup datapath.
//
module fp_add (
    input  [31:0] a,
    input  [31:0] b_in,
    input         sub,     // when 1, compute a - b_in
    output [31:0] y
);
    wire [31:0] b = sub ? {~b_in[31], b_in[30:0]} : b_in;

    wire        sa = a[31];
    wire        sb = b[31];
    wire [7:0]  ea = a[30:23];
    wire [7:0]  eb = b[30:23];
    wire [22:0] ma = a[22:0];
    wire [22:0] mb = b[22:0];

    // significands with implicit/hidden bit (0 for zero/subnormal exp)
    wire [23:0] sig_a = {(ea != 8'd0), ma};
    wire [23:0] sig_b = {(eb != 8'd0), mb};

    // Treat exponent of a true zero/subnormal as 1 so the bias math lines up
    // (hidden bit already cleared above).
    wire [7:0]  exa = (ea == 8'd0) ? 8'd1 : ea;
    wire [7:0]  exb = (eb == 8'd0) ? 8'd1 : eb;

    // ---- align to the larger exponent ----
    wire        a_ge = (exa > exb) || ((exa == exb) && (sig_a >= sig_b));
    wire [7:0]  e_big  = a_ge ? exa : exb;
    wire [7:0]  e_sml  = a_ge ? exb : exa;
    wire        s_big  = a_ge ? sa : sb;
    wire        s_sml  = a_ge ? sb : sa;

    wire [7:0]  shamt8 = e_big - e_sml;
    // Extend significands with 3 guard/round/sticky bits.
    wire [26:0] big_ext = {(a_ge ? sig_a : sig_b), 3'b000};
    wire [26:0] sml_pre = {(a_ge ? sig_b : sig_a), 3'b000};

    // Shift smaller right by shamt, capturing sticky.
    reg  [26:0] sml_sh;
    reg         sticky_lost;
    integer k;
    always @(*) begin
        if (shamt8 >= 8'd27) begin
            sml_sh = 27'd0;
            sticky_lost = |sml_pre;
        end else begin
            sml_sh = sml_pre >> shamt8;
            // sticky = OR of bits shifted out
            sticky_lost = |(sml_pre & ((27'd1 << shamt8) - 27'd1));
        end
    end
    wire [26:0] sml_ext = {sml_sh[26:1], sml_sh[0] | sticky_lost};

    wire same_sign = (s_big == s_sml);

    // ---- add or subtract magnitudes ----
    wire [27:0] sum_mag = same_sign ? ({1'b0, big_ext} + {1'b0, sml_ext})
                                    : ({1'b0, big_ext} - {1'b0, sml_ext});

    // result sign is sign of the larger magnitude operand
    wire res_sign = s_big;

    // ---- normalize ----
    reg  [27:0] norm;
    reg  [9:0]  e_res;   // signed-ish, room for adjustment
    always @(*) begin
        e_res = {2'b00, e_big};
        if (sum_mag[27]) begin
            // carry out: shift right 1
            norm  = sum_mag >> 1;
            e_res = e_res + 10'd1;
            // recover sticky from bit dropped
            norm[0] = norm[0] | sum_mag[0];
        end else if (sum_mag[26]) begin
            norm = sum_mag;
        end else begin
            // leading-zero normalize (subtraction cancellation)
            norm = sum_mag;
            // find first 1 from bit 26 down
            if (sum_mag[26:0] == 27'd0) begin
                norm  = 28'd0;
                e_res = 10'd0;
            end else begin
                for (k = 26; k >= 0; k = k - 1) begin
                    if (norm[26] == 1'b0) begin
                        norm  = norm << 1;
                        e_res = e_res - 10'd1;
                    end
                end
            end
        end
    end

    // norm[26] = hidden bit (1.xxxx), norm[25:3]=mantissa, [2]=G,[1]=R,[0]=S
    wire guard  = norm[2];
    wire round  = norm[1];
    wire sticky = norm[0];
    wire lsb    = norm[3];
    wire round_up = guard & (round | sticky | lsb);

    wire [24:0] mant_rnd = {1'b0, norm[26:3]} + (round_up ? 25'd1 : 25'd0);

    // rounding can overflow mantissa -> bump exponent
    wire        mant_carry = mant_rnd[24];
    wire [22:0] final_mant = mant_carry ? mant_rnd[23:1] : mant_rnd[22:0];
    wire [9:0]  final_exp  = mant_carry ? (e_res + 10'd1) : e_res;

    // zero result
    wire is_zero = (norm == 28'd0);

    assign y = is_zero ? 32'h00000000
                       : {res_sign, final_exp[7:0], final_mant};
endmodule
