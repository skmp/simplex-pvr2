//
// fp_add32 - extended-precision add/sub on the setup float (xf) bus.
//   y = a (sub ? - : +) b,  all operands in the 41-bit xf format (fp_ext.svh):
//       {sign[40], exp[39:32], mant[31:0]}, hidden-1 at mant bit31.
//
// Same structure as fp_add24 but with FULL 32-bit significands (31 frac bits),
// so intermediate setup sums keep the precision produced by fp_mul_full instead
// of collapsing to 24 bits. Split into two combinational halves so callers can
// pipeline:
//   fp_add32_s1 : sign-fix, DaZ, order, exp-diff, align, add/sub -> {sum,e_big,s_big}
//   fp_add32_s2 : normalize (leading-zero) + pack -> y
// fp_add32 is the combinational whole for non-pipelined callers.
//
// Non-IEEE: DaZ, no inf/NaN, truncate, overflow saturates / underflow flushes.
//
// ---- stage 1: align + add/sub ----
module fp_add32_s1 (
    input  [40:0] a,
    input  [40:0] b_in,
    input         sub,
    output [32:0] sum,      // pre-normalize significand sum (bit32 = carry)
    output [7:0]  e_big,    // exponent of the larger operand
    output        s_big     // sign of the result (larger operand's sign)
);
    wire [40:0] b = sub ? {~b_in[40], b_in[39:0]} : b_in;
    wire       sa = a[40],  sb = b[40];
    wire [7:0] ea = a[39:32], eb = b[39:32];

    // 32-bit significands: DaZ operand -> hidden bit 0 so it contributes nothing.
    wire [31:0] sig_a = (ea != 8'd0) ? a[31:0] : 32'd0;
    wire [31:0] sig_b = (eb != 8'd0) ? b[31:0] : 32'd0;
    wire [7:0]  exa   = (ea == 8'd0) ? 8'd1 : ea;
    wire [7:0]  exb   = (eb == 8'd0) ? 8'd1 : eb;

    wire        a_ge  = (exa > exb) || ((exa == exb) && (sig_a >= sig_b));
    wire [31:0] sig_big = a_ge ? sig_a : sig_b;
    wire [31:0] sig_sml = a_ge ? sig_b : sig_a;
    wire        s_sml   = a_ge ? sb : sa;
    wire [7:0]  e_sml_e = a_ge ? exb : exa;
    wire [7:0]  shamt   = e_big - e_sml_e;

    assign e_big = a_ge ? exa : exb;
    assign s_big = a_ge ? sa : sb;

    wire [31:0] sml_sh = (shamt >= 8'd32) ? 32'd0 : (sig_sml >> shamt);
    wire same_sign = (s_big == s_sml);
    assign sum = same_sign ? ({1'b0, sig_big} + {1'b0, sml_sh})
                           : ({1'b0, sig_big} - {1'b0, sml_sh});
endmodule

// ---- stage 2: normalize + pack ----
module fp_add32_s2 (
    input  [32:0] sum,
    input  [7:0]  e_big,
    input         s_big,
    output [40:0] y
);
    reg  [31:0] norm_sig;
    reg  signed [10:0] e_norm;
    integer i; reg found;
    always @(*) begin
        found = 1'b0;
        if (sum[32]) begin                 // carry out of the add
            norm_sig = sum[32:1];
            e_norm   = $signed({3'b0, e_big}) + 11'sd1;
        end else if (sum[31]) begin        // already normalized
            norm_sig = sum[31:0];
            e_norm   = $signed({3'b0, e_big});
        end else begin                     // cancellation: leading-zero normalize
            norm_sig = sum[31:0];
            e_norm   = $signed({3'b0, e_big});
            for (i = 1; i < 32; i = i + 1)
                if (!found && sum[31-i]) begin
                    norm_sig = sum[31:0] << i;
                    e_norm   = $signed({3'b0, e_big}) - i;
                    found    = 1'b1;
                end
        end
    end
    wire res_zero  = (sum == 33'd0);
    wire underflow = (e_norm <= 0);
    wire overflow  = (e_norm >= 255);
    assign y = res_zero  ? {1'b0, 8'd0, 32'd0}
             : underflow ? {s_big, 8'd0, 32'd0}
             : overflow  ? {s_big, 8'hFE, 32'hFFFFFFFF}
                         : {s_big, e_norm[7:0], norm_sig};
endmodule

// ---- combinational whole (for non-pipelined callers) ----
module fp_add32 (
    input  [40:0] a,
    input  [40:0] b_in,
    input         sub,
    output [40:0] y
);
    wire [32:0] sum; wire [7:0] e_big; wire s_big;
    fp_add32_s1 s1 (.a(a), .b_in(b_in), .sub(sub), .sum(sum), .e_big(e_big), .s_big(s_big));
    fp_add32_s2 s2 (.sum(sum), .e_big(e_big), .s_big(s_big), .y(y));
endmodule
