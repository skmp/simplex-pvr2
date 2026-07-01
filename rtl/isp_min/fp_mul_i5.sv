//
// fp_mul_i5 - cheap float32 * small-integer multiply, y = f * k, k in 0..31.
//
// The rasterizer only ever multiplies the setup coefficients (DX/DY/ddx/ddy) by
// a tile-local pixel index x or y, which is a 5-bit integer (0..31). So instead
// of a full float*float multiply we do a 16-bit-mantissa * 5-bit-integer
// product: sig(1.15) * k(5b) -> 21 bits, then normalize (k<=31 -> <=5 bit
// growth) and adjust the exponent. Much cheaper than fp_mul16.
//
// Non-IEEE, matching the rest of the setup datapath: DaZ input, no inf/NaN,
// truncate, k==0 -> +0. Combinational.
//
module fp_mul_i5 (
    input  [31:0] f,
    input  [4:0]  k,
    output [31:0] y
);
    wire        sf = f[31];
    wire [7:0]  ef = f[30:23];
    wire        f_zero = (ef == 8'd0);          // DaZ
    wire        k_zero = (k == 5'd0);

    // 16-bit significand (1 hidden + 15) times 5-bit k -> up to 21 bits.
    wire [15:0] sig = {1'b1, f[22:8]};
    wire [20:0] prod = sig * {11'd0, k};         // 16 x 5 -> 21 bits

    // Leading-one position of prod. sig has its 1 at bit15; multiplying by
    // k in [1,31] adds 0..5 bits, so the leading one is at bit 15..20.
    // Find it and normalize so the hidden 1 sits just above 15 fraction bits.
    reg  [4:0]  msb;      // index of leading one (15..20)
    integer i;
    always @(*) begin
        msb = 5'd15;
        for (i = 15; i <= 20; i = i + 1)
            if (prod[i]) msb = i[4:0];
    end

    // shift so leading one -> bit15, take low 15 as fraction (pad to 23).
    wire [5:0]  sh   = msb - 5'd15;              // 0..5
    wire [20:0] norm = prod >> sh;               // leading one now at bit15
    wire [22:0] mant = {norm[14:0], 8'b0};       // 15 real frac bits, padded

    // exponent: ef + (msb-15). The 16-bit sig already represents 1.f * 2^(ef-127)
    // scaled by k; each extra leading-bit is one more power of two.
    wire signed [10:0] e = $signed({3'b0, ef}) + $signed({6'b0, sh});

    wire is_zero  = f_zero | k_zero;
    wire overflow = (e >= 255);

    assign y = is_zero  ? {sf, 31'd0}
             : overflow ? {sf, 8'hFE, 23'h7FFFFF}
                        : {sf, e[7:0], mant};
endmodule
