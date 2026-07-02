//
// f2u8 - float32 -> unsigned 8-bit (0..255), combinational.
//
// Truncating conversion matching the TSP datapath (DaZ, no round, clamp):
//   negative or |v| < 1.0   -> 0
//   v >= 256.0               -> 255
//   else  integer part of v  (0..255)
//
// {1'b1, f[22:15]} is the mantissa (implicit 1 + top 8 frac bits) scaled by
// 2^8; the integer value of v = 1.f * 2^(exp-127) is that >> (8 - (exp-127))
// = >> (135 - exp), for exp in 127..134.
//
// Used by tsp_shade / tsp_shade_pp to pack interpolated 0..255 colour channels.
//
module f2u8 (
    input  [31:0] f,
    output [7:0]  u
);
    // integer part of |v| via the mantissa scaled by 2^8, shifted by (135-exp).
    // shift is 1..8 in the in-range case; >=256 and <1 are handled separately.
    wire [8:0] iv = {1'b1, f[22:15]} >> (8'd135 - f[30:23]);
    assign u = (f[31] || f[30:23] < 8'd127) ? 8'd0
             : (f[30:23] >= 8'd135)          ? 8'd255
             : (iv > 9'd255)                 ? 8'd255
                                             : iv[7:0];
endmodule
