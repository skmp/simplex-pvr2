//
// tsp_miplevel - exponent-domain mip level (LOD) selection.
//
// Approximates refsw2's faux-mip calc (refsw_tile.cpp PixelFlush_tsp):
//   ddx  = U.ddx + V.ddx ;  ddy = U.ddy + V.ddy      (interpolation plane deltas)
//   dMip = min(|ddx|,|ddy|) * W * sizeU * MipMapD/4  ;  sizeU = 8 << TexU
//   MipLevel = 0; while (dMip > 1.5 && MipLevel < 11) { MipLevel++; dMip /= 2; }
//
// Done purely in the float exponent domain (cheap: adds/subtract, no float
// multiply, no divide loop):
//   log2(dMip) ~= (e_min - 127) + (e_W - 127) + log2(sizeU) + log2(MipMapD/4)
//     e_min = min(exp(ddx), exp(ddy))    (exp of a float f = f[30:23])
//     exp(ddx=U.ddx+V.ddx) ~= max(exp(U.ddx), exp(V.ddx))   (sum ~ larger term)
//     log2(sizeU)     = 3 + TexU
//     log2(MipMapD/4) ~= msb_index(MipMapD) - 2
//   MipLevel = clamp( round(log2(dMip)), 0, TexU+3 )   (0..11 overall)
//
// This is within ~+/-1 level of refsw near boundaries (the 1.5 threshold and
// mantissa contributions are dropped), which is visually negligible for a
// point-sampled mip. miplevel is 0 when not mipmapped or when the LOD is <= 0.
//
module tsp_miplevel (
    input      [31:0] ddxU, ddxV,   // U/V interpolation-plane ddx
    input      [31:0] ddyU, ddyV,   // U/V interpolation-plane ddy
    input      [31:0] w,            // 1/invW (perspective W)
    input      [2:0]  texu,         // TSP.TexU
    input      [3:0]  mipmapd,      // TSP.MipMapD
    input             mipmapped,    // TCW.MipMapped
    output reg [3:0]  miplevel
);
    // biased exponents
    wire [7:0] eXu = ddxU[30:23], eXv = ddxV[30:23];
    wire [7:0] eYu = ddyU[30:23], eYv = ddyV[30:23];

    // top mantissa bit of each operand (the "+0.5..1.0" fractional log2 term the
    // pure-exponent estimate drops). Including these for the min-dd term and W
    // recovers the ~1-2 level underestimate of a floor()-only exponent LOD.
    wire mXu = ddxU[22], mXv = ddxV[22];
    wire mYu = ddyU[22], mYv = ddyV[22];

    // exp(ddx) ~ max of the two summed terms; pick that term's mantissa bit too.
    wire [7:0] eX = (eXu >= eXv) ? eXu : eXv;   wire mX = (eXu >= eXv) ? mXu : mXv;
    wire [7:0] eY = (eYu >= eYv) ? eYu : eYv;   wire mY = (eYu >= eYv) ? mYu : mYv;
    // min(|ddx|,|ddy|): the smaller-exponent axis; carry its mantissa bit.
    wire [7:0] eMin = (eX <= eY) ? eX : eY;     wire mMin = (eX <= eY) ? mX : mY;
    wire [7:0] eW   = w[30:23];                 wire mW   = w[22];

    // log2(MipMapD): index of the most-significant set bit (0 if MipMapD==0).
    reg [2:0] mmd_log2;
    always @* begin
        casez (mipmapd)
            4'b1???: mmd_log2 = 3'd3;
            4'b01??: mmd_log2 = 3'd2;
            4'b001?: mmd_log2 = 3'd1;
            default: mmd_log2 = 3'd0;   // 0001 or 0000
        endcase
    end

    // log2(dMip) in a signed accumulator:
    //   (eMin-127) + (eW-127) + (3+TexU) + (mmd_log2 - 2) + mMin + mW
    // = eMin + eW + TexU + mmd_log2 + mMin + mW - 253
    // (mMin/mW are the dropped top mantissa bits, ~+1 each when set.)
    // Biased down by one extra (-254) to select one lower (sharper) mip level.
    wire signed [11:0] lod =
          $signed({4'd0, eMin})
        + $signed({4'd0, eW})
        + $signed({9'd0, texu})
        + $signed({9'd0, mmd_log2})
        + $signed({11'd0, mMin})
        + $signed({11'd0, mW})
        - 12'sd254;

    // clamp to [0, TexU+3] (and the module's 0..11 range); 0 when not mipmapped.
    wire [3:0] lvl_max = 4'(3) + {1'b0, texu};   // TexU+3 (max 10)
    always @* begin
        if (!mipmapped)          miplevel = 4'd0;
        else if (lod <= 0)       miplevel = 4'd0;
        else if (lod >= $signed({8'd0, lvl_max})) miplevel = lvl_max;
        else                     miplevel = lod[3:0];
    end
endmodule
