//
// fp_ge - signed-float ordering compare a >= b, REGISTERED output (1 cycle).
//
// Bit-exact replacement for "sign of fp_add24(a, b, sub=1) is clear": the sign
// of a float subtract IS the exact ordering predicate - rounding/normalization
// only change the magnitude, never the sign. fp_add24 corner cases match too:
// exact cancellation is only possible at shamt==0 (i.e. a == b, packed +0 ->
// "ge"), and underflow flushes to {s_big, 0} keeping the exact sign. So this
// is one magnitude comparator instead of align shifter + adder + normalizer.
//
// Raw-bit magnitude compare on [30:0] is consistent with fp_add24_s1's
// ordering (denormals: exp field 0 ranks as exp 1 with leading bit 0, which is
// exactly how the raw bits already order). No inf/NaN; +/-0 compare equal.
//
module fp_ge (
    input         clk,
    input  [31:0] a,
    input  [31:0] b,
    output reg    ge      // a >= b, valid 1 cycle after a/b
);
    wire az = (a[30:0] == 31'd0);
    wire bz = (b[30:0] == 31'd0);
    always @(posedge clk)
        ge <= (az && bz)      ? 1'b1                    // +/-0 == +/-0
            : (a[31] ^ b[31]) ? b[31]                   // one negative: ge iff it's b
            : (~a[31])        ? (a[30:0] >= b[30:0])    // both >= 0
                              : (a[30:0] <= b[30:0]);   // both <= 0
endmodule
