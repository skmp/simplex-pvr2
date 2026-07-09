//
// fp_rcp_fast - non-IEEE reciprocal y ~= 1/x. PIPELINED (3 cycles) so it closes
// timing: no runtime divider (a precomputed seed ROM), and the Newton-step
// multiplies are split across pipeline stages.
//
// FULL-MANTISSA version: an 11-bit index (2048-entry ROM) seeds ~1/m to ~11 bits,
// then ONE Newton step doubles that to ~22 bits - enough to fill the whole 23-bit
// float32 mantissa. (The previous 256-entry / Q16 version only carried ~15 bits,
// which lost precision on setup's tiny 1/area for huge guard-band triangles.)
//
// Method, Q24 fixed point on the significand:
//   m  = 1.mx  in [1,2)                             (Q1.23, the full mantissa)
//   r0 = SEED_ROM[idx] ~ 1/m in (0.5,1]             (Q0.24, 2048-entry ROM)
//   r1 = r0 * (2 - m*r0)   (one Newton step)        -> ~1/m to ~22 bits
//   1/x = r1 * 2^(127-ex), packed to float32 (full 23-bit mantissa).
//
// Pipeline: s1 ROM lookup + capture ex/sign/m; s2 mr=m*r0, two_m=2-mr;
//           s3 r1=r0*two_m, normalize+pack.  in_valid@N -> out_valid@N+3.
// DaZ input -> saturates to a large finite value (no inf/NaN).
//
module fp_rcp_fast (
    input             clk,
    input             reset,
    input             stall,     // 1 stalls (freezes) all pipeline stages
    input             in_valid,
    input      [31:0] x,
    output reg        out_valid,
    output reg [31:0] y
);
    // ---- decompose ----
    wire        sx = x[31];
    wire [7:0]  ex = x[30:23];
    wire        x_zero = (ex == 8'd0);
    wire [24:0] m_q24  = {1'b1, x[22:0], 1'b0};    // Q1.24 significand (hidden 1 + 23)
    wire [10:0] idx    = x[22:12];                 // top 11 mantissa bits -> ROM index

    // ---- seed ROM: r0 = round(2^48 / (2^24 + idx*2^13)) in Q0.24 ----
    // A genuine 2048-entry ROM initialized at elaboration (NOT a runtime divide).
    // idx selects m's top 11 fraction bits; the ROM entry approximates 1/m for the
    // interval's midpoint, in Q0.24 (value in (0.5, 1]).
    reg [24:0] seed_rom [0:2047];
    integer ri;
    initial for (ri = 0; ri < 2048; ri = ri + 1)
        // m_mid = 1 + (idx + 0.5)/2048  in Q1.24 = 2^24 + idx*2^13 + 2^12
        seed_rom[ri] = 25'((64'h1000000000000) / (64'd16777216 + ri * 64'd8192 + 64'd4096));

    // ---- stage 1: ROM lookup + carry ex/sign/m ----
    reg [24:0] s1_r0, s1_m;
    reg [7:0]  s1_ex;
    reg        s1_s, s1_xz, v1;
    always @(posedge clk) begin
        if (reset) v1 <= 0;
        else if (!stall) begin
            s1_r0 <= seed_rom[idx];
            s1_m  <= m_q24;
            s1_ex <= ex;  s1_s <= sx;  s1_xz <= x_zero;
            v1    <= in_valid;
        end
    end

    // ---- stage 2: mr = m*r0 (Q1.48) ; two_m = 2.0 - mr  (Q2.24) ----
    reg [24:0] s2_r0;
    reg [25:0] s2_two_m;
    reg [7:0]  s2_ex;
    reg        s2_s, s2_xz, v2;
    reg [49:0] mr_c;
    always @(*) mr_c = s1_m * s1_r0;               // 25b * 25b = 50b, Q1.48
    always @(posedge clk) begin
        if (reset) v2 <= 0;
        else if (!stall) begin
            // 2.0 (Q2.24 = 26'h2000000) minus the Q2.24 top of mr (bits [49:24]).
            s2_two_m <= 26'h2000000 - mr_c[49:24];
            s2_r0    <= s1_r0;
            s2_ex    <= s1_ex; s2_s <= s1_s; s2_xz <= s1_xz;
            v2       <= v1;
        end
    end

    // ---- stage 3: r1 = r0*two_m ; normalize + pack ----
    always @(posedge clk) begin
        if (reset) out_valid <= 0;
        else if (!stall) begin
            out_valid <= v2;
            y <= pack(s2_r0, s2_two_m, s2_ex, s2_s, s2_xz);
        end
    end

    function [31:0] pack(input [24:0] r0, input [25:0] two_m,
                         input [7:0] exf, input s, input xz);
        reg [50:0] r1_full; reg [24:0] r1; reg [22:0] frac; reg signed [10:0] e;
        begin
            r1_full = r0 * two_m;                  // 25b * 26b = 51b, Q1.48-ish
            r1      = r1_full[48:24];              // 1/m in Q0.24 (0.5,1], 25 bits
            // r1[24] set -> 1/m == 1.0 (m was 1.0); else leading 1 at bit23.
            frac    = r1[24] ? 23'd0 : r1[22:0];
            e       = (r1[24] ? 11'sd254 : 11'sd253) - $signed({3'b0, exf});
            if (xz)          pack = {s, 8'hFE, 23'h7FFFFF};
            else if (e <= 0) pack = {s, 31'd0};
            else if (e>=255) pack = {s, 8'hFE, 23'h7FFFFF};
            else             pack = {s, e[7:0], frac};
        end
    endfunction
endmodule
