//
// tex_uv2texel - convert interpolated (float) u,v to fixed texel coords and the
// four bilinear corner integer coords (ClampFlip'd), combinational + testable.
//
// refsw: sizeU=8<<TexU, sizeV=8<<TexV (no mipmap here).
//   ui = u*sizeU*256 (+halfpixel, 0 here) ; vi = v*sizeV*256
//   base texel = (ui>>8, vi>>8); corners: (+1,+1)(+0,+1)(+1,+0)(+0,+0)
//   each corner ClampFlip(coord,size). fractions ufrac=ui&255, vfrac=vi&255.
//
// ui = u * 2^(11+TexU) since sizeU*256 = 2^(11+TexU). So ui is u reinterpreted
// with the binary point shifted - a float->fixed extraction, no multiplier.
// We produce ui as unsigned Q?.8 (negatives clamp to 0 via the wrap/clamp).
//
module tex_uv2texel (
    input      [31:0] u,          // float
    input      [31:0] v,          // float
    input      [2:0]  texu,
    input      [2:0]  texv,
    input             clampu, clampv, flipu, flipv,

    output [10:0] c00u, output [10:0] c00v,   // (u+1,v+1)
    output [10:0] c01u, output [10:0] c01v,   // (u+0,v+1)
    output [10:0] c10u, output [10:0] c10v,   // (u+1,v+0)
    output [10:0] c11u, output [10:0] c11v,   // (u+0,v+0)
    output [7:0]  ufrac, output [7:0]  vfrac
);
    // float -> fixed: value * 2^shift, take as integer.frac with 8 fraction bits.
    // For u = 1.m * 2^(e-127), u*2^shift = 1.m * 2^(e-127+shift). We want a
    // fixed-point with 8 fraction bits: result_fixed = round-toward-zero of
    // (1.m << (e-127+shift+8)) >> ... i.e. place the leading 1 at bit
    // (e-127+shift+8). We form a wide value and shift.
    // shift_u = 11+texu, shift_v = 11+texv. Max e-127+shift+8 ~ up to ~30.
    function [26:0] to_fixed(input [31:0] f, input [4:0] shift);
        reg [7:0] e; reg signed [9:0] p; reg [23:0] sig; reg [55:0] wide;
        begin
            if (f[31] || f[30:23]==8'd0) to_fixed = 27'd0;      // neg/zero -> 0
            else begin
                e   = f[30:23];
                sig = {1'b1, f[22:0]};                          // 1.m (24b, .23)
                // ui = u * sizeU * 256 = u * 2^(11+TexU) = shift. The *256 is the
                // 8 fraction bits, already inside `shift`. So:
                //   ui = sig * 2^(e-127-23+shift)
                p = $signed({2'b0,e}) - 10'sd127 - 10'sd23 + $signed({5'b0,shift});
                wide = {32'd0, sig};
                if (p >= 0) wide = wide << p[5:0];
                else        wide = wide >> (-p);
                to_fixed = wide[26:0];       // ui = texel<<8 (Q19.8)
            end
        end
    endfunction

    wire [26:0] ui = to_fixed(u, {2'b0,texu} + 5'd11);
    wire [26:0] vi = to_fixed(v, {2'b0,texv} + 5'd11);

    wire [18:0] uint = ui[26:8];         // ui>>8
    wire [18:0] vint = vi[26:8];
    assign ufrac = ui[7:0];
    assign vfrac = vi[7:0];

    wire [10:0] sizeU = 11'd8 << texu;
    wire [10:0] sizeV = 11'd8 << texv;

    // ClampFlip(coord, size): clamp / flip(mirror) / wrap
    function [10:0] clampflip(input clamp, input flip, input signed [20:0] coord, input [10:0] size);
        reg signed [20:0] c; reg [10:0] m;
        begin
            if (clamp) begin
                if (coord < 0)               clampflip = 11'd0;
                else if (coord >= size)      clampflip = size - 11'd1;
                else                         clampflip = coord[10:0];
            end else if (flip) begin
                c = coord & ((size<<1)-1);
                if (c & size) c = c ^ ((size<<1)-1);
                clampflip = c[10:0];
            end else begin
                clampflip = coord & (size-11'd1);   // wrap
            end
        end
    endfunction

    wire signed [20:0] u0 = {2'b0, uint};
    wire signed [20:0] u1 = {2'b0, uint} + 21'sd1;
    wire signed [20:0] v0 = {2'b0, vint};
    wire signed [20:0] v1 = {2'b0, vint} + 21'sd1;

    assign c00u = clampflip(clampu,flipu,u1,sizeU); assign c00v = clampflip(clampv,flipv,v1,sizeV);
    assign c01u = clampflip(clampu,flipu,u0,sizeU); assign c01v = clampflip(clampv,flipv,v1,sizeV);
    assign c10u = clampflip(clampu,flipu,u1,sizeU); assign c10v = clampflip(clampv,flipv,v0,sizeV);
    assign c11u = clampflip(clampu,flipu,u0,sizeU); assign c11v = clampflip(clampv,flipv,v0,sizeV);
endmodule
