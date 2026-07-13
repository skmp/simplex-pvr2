//
// fp_add24_spp_ro - STREAMING PIPELINED, REGISTERED-OUTPUT variant of fp_add24.
//
// Same reduced-precision non-IEEE add/sub as the combinational fp_add24 (DaZ, no
// inf/NaN, truncate, overflow saturates, underflow flushes) and BIT-EXACT to it for the
// same (a, b_in, sub) - but split into a 3-clock streaming pipeline whose OUTPUT is
// registered. The deep part of fp_add24 is the normalize: a 24-way leading-zero scan
// feeding a variable shift + pack. That single combinational cloud (stacked after the
// align+add) is the Fmax limiter, so here it is SPLIT across two stages:
//   S1 : align + add/sub  (== fp_add24_s1)            -> {sum, e_big, s_big}
//   S2 : leading-1 SEARCH only (priority encode -> shift amount / case), no shifting
//   S3 : apply the shift + exponent adjust + pack     -> registered y
//
// CONVENTION (matches fp_rcp_fast / the streaming units):
//   ports (clk, reset, stall, in_valid, a, b_in, sub, out_valid, y).
//   in_valid @N -> out_valid @N+3, y @N+3 (registered). stall=1 freezes all stages.
//   one result/clock throughput when !stall.
//
module fp_add24_spp_ro (
    input             clk,
    input             reset,
    input             stall,
    input             in_valid,
    input      [31:0] a,
    input      [31:0] b_in,
    input             sub,
    output reg        out_valid,
    output reg [31:0] y
);
    // ======================================================================
    // S1 combinational: align + add/sub (identical to fp_add24_s1).
    // ======================================================================
    wire [31:0] b = sub ? {~b_in[31], b_in[30:0]} : b_in;
    wire       sa = a[31],  sb = b[31];
    wire [7:0] ea = a[30:23], eb = b[30:23];

    wire [23:0] sig_a = {(ea != 8'd0), a[22:0]};
    wire [23:0] sig_b = {(eb != 8'd0), b[22:0]};
    wire [7:0]  exa   = (ea == 8'd0) ? 8'd1 : ea;
    wire [7:0]  exb   = (eb == 8'd0) ? 8'd1 : eb;

    wire        a_ge  = (exa > exb) || ((exa == exb) && (sig_a >= sig_b));
    wire [23:0] sig_big = a_ge ? sig_a : sig_b;
    wire [23:0] sig_sml = a_ge ? sig_b : sig_a;
    wire        s_sml   = a_ge ? sb : sa;
    wire [7:0]  e_sml_e = a_ge ? exb : exa;
    wire [7:0]  e_big_c = a_ge ? exa : exb;
    wire        s_big_c = a_ge ? sa : sb;
    wire [7:0]  shamt   = e_big_c - e_sml_e;

    wire [23:0] sml_sh = (shamt >= 8'd24) ? 24'd0 : (sig_sml >> shamt);
    wire same_sign = (s_big_c == s_sml);
    wire [24:0] sum_c = same_sign ? ({1'b0, sig_big} + {1'b0, sml_sh})
                                  : ({1'b0, sig_big} - {1'b0, sml_sh});

    // ---- S1 registers ----
    reg               v1;
    reg [24:0]        s1_sum;
    reg [7:0]         s1_ebig;
    reg               s1_sbig;
    always @(posedge clk) begin
        if (reset) v1 <= 1'b0;
        else if (!stall) begin
            v1      <= in_valid;
            s1_sum  <= sum_c;
            s1_ebig <= e_big_c;
            s1_sbig <= s_big_c;
        end
    end

    // ======================================================================
    // S2 combinational: leading-1 SEARCH ONLY. Determine how the significand must be
    // normalized (carry-out, already-normal, or a cancellation left-shift by `lz`),
    // WITHOUT doing the shift here - that is the deep part and moves to S3.
    //   sh_sel : 0 = carry (sum[24])   -> shift right 1, exp+1
    //            1 = normal (sum[23])  -> no shift
    //            2 = cancel            -> left shift by `lz` (1..23), exp-lz
    //   lz     : leading-zero count in [1..23] for the cancel case (else 0).
    // ======================================================================
    reg  [1:0] sh_sel_c;
    reg  [4:0] lz_c;
    integer si; reg found_c;
    always @(*) begin
        found_c = 1'b0;
        if (s1_sum[24]) begin
            sh_sel_c = 2'd0; lz_c = 5'd0;
        end else if (s1_sum[23]) begin
            sh_sel_c = 2'd1; lz_c = 5'd0;
        end else begin
            sh_sel_c = 2'd2; lz_c = 5'd0;
            for (si = 1; si < 24; si = si + 1)
                if (!found_c && s1_sum[23-si]) begin
                    lz_c = si[4:0]; found_c = 1'b1;
                end
        end
    end
    wire s1_zero_c = (s1_sum == 25'd0);

    // ---- S2 registers ----
    reg        v2;
    reg [24:0] s2_sum;
    reg [7:0]  s2_ebig;
    reg        s2_sbig, s2_zero;
    reg [1:0]  s2_sel;
    reg [4:0]  s2_lz;
    always @(posedge clk) begin
        if (reset) v2 <= 1'b0;
        else if (!stall) begin
            v2      <= v1;
            s2_sum  <= s1_sum;
            s2_ebig <= s1_ebig;
            s2_sbig <= s1_sbig;
            s2_zero <= s1_zero_c;
            s2_sel  <= sh_sel_c;
            s2_lz   <= lz_c;
        end
    end

    // ======================================================================
    // S3 combinational: apply the shift + exponent adjust + pack.
    // ======================================================================
    reg  [23:0] norm_sig;
    reg  signed [10:0] e_norm;
    always @(*) begin
        case (s2_sel)
            2'd0: begin norm_sig = s2_sum[24:1];      e_norm = $signed({3'b0, s2_ebig}) + 11'sd1; end
            2'd1: begin norm_sig = s2_sum[23:0];      e_norm = $signed({3'b0, s2_ebig});          end
            default: begin
                norm_sig = s2_sum[23:0] << s2_lz;
                e_norm   = $signed({3'b0, s2_ebig}) - $signed({6'd0, s2_lz});
            end
        endcase
    end
    wire underflow = (e_norm <= 0);
    wire overflow  = (e_norm >= 255);
    wire [31:0] y_c = s2_zero   ? 32'd0
                    : underflow ? {s2_sbig, 31'd0}
                    : overflow  ? {s2_sbig, 8'hFE, 23'h7FFFFF}
                                : {s2_sbig, e_norm[7:0], norm_sig[22:0]};

    // ---- S3 register: the module's registered output ----
    always @(posedge clk) begin
        if (reset) out_valid <= 1'b0;
        else if (!stall) begin
            out_valid <= v2;
            y         <= y_c;
        end
    end
endmodule
