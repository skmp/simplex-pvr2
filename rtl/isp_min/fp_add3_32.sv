//
// fp_add3_32 - extended-precision 3-input adder on the setup float (xf) bus:
//   y = a + b + c,  all operands in the 41-bit xf format (fp_ext.svh).
//
// Same idea as fp_add3_24 (fold two dependent adds into ONE align + ONE
// normalize) but with FULL 32-bit significands. Used for the true 3-way setup
// sums (e.g. the tile-local plane constant c = a1 - ddx*XL - ddy*YT) so no
// intermediate round/normalize is taken and no mantissa bits are dropped between
// fp_mul_full and the final pack.
//
// Guard bits: the classic 3-way hazard is catastrophic cancellation - two large
// operands nearly cancel and the result is dominated by a small third operand
// whose low bits were shifted out during alignment. To retain them we carry G=8
// guard bits below the significand LSB through align + sum + normalize, so up to
// 8 bits of cancellation are recovered exactly before the final 32-bit extract.
//
// Non-IEEE: DaZ, no inf/NaN, truncate, overflow saturates, underflow flushes.
// Split into s1 (align+sum) / s2 (normalize+pack) for pipelining; fp_add3_32 is
// the combinational whole.
//
// ---- stage 1: align three operands to the max exponent (with guard), signed sum ----
module fp_add3_32_s1 (
    input  [40:0] a,
    input  [40:0] b,
    input  [40:0] c,
    output signed [43:0] ssum,   // signed pre-norm sum: 40 mag (32 + 8 guard) + sign + 3 headroom
    output        [7:0]  e_max   // common (largest) exponent operands aligned to
);
    localparam integer G = 8;    // guard bits retained below the significand LSB

    // decode: DaZ (biased-exp 0 -> significand 0 so it adds nothing)
    wire sa = a[40], sb = b[40], sc = c[40];
    wire [7:0] ea = a[39:32], eb = b[39:32], ec = c[39:32];
    wire za = (ea == 8'd0), zb = (eb == 8'd0), zc = (ec == 8'd0);

    // significands widened with G guard bits: {sig[31:0], G'b0} -> leading 1 @ bit 39
    wire [39:0] sig_a = za ? 40'd0 : {a[31:0], {G{1'b0}}};
    wire [39:0] sig_b = zb ? 40'd0 : {b[31:0], {G{1'b0}}};
    wire [39:0] sig_c = zc ? 40'd0 : {c[31:0], {G{1'b0}}};
    // effective exponents (flushed operands treated as the min so they never win max)
    wire [7:0]  exa = za ? 8'd0 : ea;
    wire [7:0]  exb = zb ? 8'd0 : eb;
    wire [7:0]  exc = zc ? 8'd0 : ec;

    // max exponent of the three
    wire [7:0] e_ab  = (exa >= exb) ? exa : exb;
    assign     e_max = (e_ab >= exc) ? e_ab : exc;

    // align each significand right by (e_max - its exp); >=40 shifts to 0
    wire [7:0] sha = e_max - exa;
    wire [7:0] shb = e_max - exb;
    wire [7:0] shc = e_max - exc;
    wire [39:0] al_a = (sha >= 8'd40) ? 40'd0 : (sig_a >> sha);
    wire [39:0] al_b = (shb >= 8'd40) ? 40'd0 : (sig_b >> shb);
    wire [39:0] al_c = (shc >= 8'd40) ? 40'd0 : (sig_c >> shc);

    // signed contributions (sign-magnitude -> 2's complement). 44 bits: 40 mag +
    // sign + headroom for summing three (up to ~3x -> +2 integer bits).
    wire signed [43:0] va = sa ? -$signed({4'b0, al_a}) : $signed({4'b0, al_a});
    wire signed [43:0] vb = sb ? -$signed({4'b0, al_b}) : $signed({4'b0, al_b});
    wire signed [43:0] vc = sc ? -$signed({4'b0, al_c}) : $signed({4'b0, al_c});
    assign ssum = va + vb + vc;
endmodule

// ---- stage 2: sign/abs, normalize (leading-zero), pack ----
module fp_add3_32_s2 (
    input  signed [43:0] ssum,
    input         [7:0]  e_max,
    output        [40:0] y
);
    wire        s_res = ssum[43];                        // result sign (G=8 guard, matches s1)
    wire [42:0] mag   = s_res ? (~ssum[42:0] + 43'd1) : ssum[42:0];  // |sum|, 43b

    // With G guard bits the leading 1 of a single aligned operand sits at bit 39.
    // Summing three can carry up to bit 41 (3 * ~2^40 < 2^42). Normalize: find the
    // leading 1, take the 32 bits below it, and set exponent = e_max + (bit-39).
    // The 8 guard bits below the 32-bit window are the recovered cancellation bits.
    reg  [31:0] norm_sig;
    reg  signed [10:0] e_norm;
    integer i; reg found;
    always @(*) begin
        norm_sig = 32'd0;
        e_norm   = $signed({3'b0, e_max});
        found    = 1'b0;
        // headroom carry-outs above bit 39 first, then bit 39, then LZ search down.
        if (mag[41]) begin
            norm_sig = mag[41:10];                       // 32 bits below leading 1
            e_norm   = $signed({3'b0, e_max}) + 11'sd2;
            found    = 1'b1;
        end else if (mag[40]) begin
            norm_sig = mag[40:9];
            e_norm   = $signed({3'b0, e_max}) + 11'sd1;
            found    = 1'b1;
        end else if (mag[39]) begin
            norm_sig = mag[39:8];                        // exactly aligned: drop G guard
            e_norm   = $signed({3'b0, e_max});
            found    = 1'b1;
        end else begin
            // cancellation: leading 1 below bit 39 -> shift up, lower exponent.
            // Search across the full magnitude INCLUDING the guard bits so up to G
            // bits of cancelled precision are recovered exactly. For a leading 1 at
            // bit p (p = 39-i), the 32-bit significand is the 32 bits at [p -: 32];
            // left-justify mag by (39-p) then take the top 32 (handles p < 31).
            for (i = 1; i <= 39; i = i + 1)
                if (!found && mag[39-i]) begin
                    norm_sig = (mag << i) >> 8;          // realign leading 1 to bit 39, drop 8 guard
                    e_norm   = $signed({3'b0, e_max}) - i;
                    found    = 1'b1;
                end
        end
    end

    wire res_zero  = (mag == 43'd0);
    wire underflow = (e_norm <= 0);
    wire overflow  = (e_norm >= 255);
    assign y = res_zero  ? {1'b0, 8'd0, 32'd0}
             : underflow ? {s_res, 8'd0, 32'd0}
             : overflow  ? {s_res, 8'hFE, 32'hFFFFFFFF}
                         : {s_res, e_norm[7:0], norm_sig};
endmodule

// ---- combinational whole (1-cycle fat step: a + b + c) ----
module fp_add3_32 (
    input  [40:0] a,
    input  [40:0] b,
    input  [40:0] c,
    output [40:0] y
);
    wire signed [43:0] ssum; wire [7:0] e_max;
    fp_add3_32_s1 s1 (.a(a), .b(b), .c(c), .ssum(ssum), .e_max(e_max));
    fp_add3_32_s2 s2 (.ssum(ssum), .e_max(e_max), .y(y));
endmodule
