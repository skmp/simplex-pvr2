//
// tex_decode - decode one texel to ARGB8888, per refsw DecodeTextel +
// ExpandToARGB8888 (no mipmap; MipLevel assumed 0). Bump map excluded.
//
// Inputs:
//   pixfmt  : TCW.PixelFmt (0=1555 1=565 2=4444 3=YUV 5=PAL4 6=PAL8; 4 bump n/a)
//   scan    : TCW.ScanOrder (selects ExpandToARGB8888 shuffle for 8888)
//   palsel  : TCW.PalSelect[5:0]
//   memtel  : the 64-bit memory word covering this texel (from the tex cache)
//   offset  : texel linear offset (selects sub-word: 16b lane / 8b / 4b nibble)
//   yuv_word: the relevant 32-bit YUV pair (offset&1 select) - precomputed by caller
//   pal_argb: palette ROM lookup result (caller does the PAL_RAM read)
//
// The caller (tex fetch) provides the palette lookup + picks memtel; here we do
// the format expansion math.
//
module tex_decode (
    input  [2:0]  pixfmt,
    input         scan,
    input  [63:0] memtel,
    input  [3:0]  offset_lo,   // offset[3:0], selects 16b/8b/4b sublane
    input  [31:0] pal_argb,    // palette ROM output (for PAL4/PAL8)
    output [31:0] argb
);
    // 16-bit lane select (offset & 3)
    wire [15:0] w16 = (offset_lo[1:0]==2'd0) ? memtel[15:0]
                    : (offset_lo[1:0]==2'd1) ? memtel[31:16]
                    : (offset_lo[1:0]==2'd2) ? memtel[47:32]
                                             : memtel[63:48];

    // ---- ARGB expanders (refsw ARGBxxxx_32), fields as {A,R,G,B} bytes ----
    wire [31:0] argb1555 = {(w16[15]?8'hFF:8'h00),
                            {w16[14:10],3'b0},   // R
                            {w16[9:5],3'b0},     // G
                            {w16[4:0],3'b0}};    // B
    wire [31:0] argb565  = {8'hFF,
                            {w16[15:11],3'b0},   // R
                            {w16[10:5],2'b0},    // G
                            {w16[4:0],3'b0}};    // B
    wire [31:0] argb4444 = {{w16[15:12],4'b0},   // A
                            {w16[11:8],4'b0},    // R
                            {w16[7:4],4'b0},     // G
                            {w16[3:0],4'b0}};    // B

    // ---- YUV422: two Y share U/V. offset&1 picks Y0/Y1; U,V from the pair ----
    // 32-bit yuv word covering this texel = memtel[ (offset&1)?63:31 : ...]
    wire [31:0] yuv32 = offset_lo[0] ? memtel[63:32] : memtel[31:0];
    wire [7:0]  yuv_y0 = yuv32[15:8];    // memtel_yuv8[1]
    wire [7:0]  yuv_y1 = yuv32[31:24];   // memtel_yuv8[3] (1 + (offset&2))
    wire [7:0]  yuv_u  = yuv32[7:0];     // [0]
    wire [7:0]  yuv_v  = yuv32[23:16];   // [2]
    wire [7:0]  yv_y   = offset_lo[1] ? yuv_y1 : yuv_y0;
    // R = Y + Yv*11/8 ; G = Y - (Yu*11+Yv*22)/32 ; B = Y + Yu*110/64
    wire signed [9:0]  yu = $signed({2'b0, yuv_u}) - 10'sd128;
    wire signed [9:0]  yv = $signed({2'b0, yuv_v}) - 10'sd128;
    wire signed [15:0] yR = $signed({8'b0, yv_y}) + (yv * 16'sd11) / 16'sd8;
    wire signed [15:0] yG = $signed({8'b0, yv_y}) - (yu*16'sd11 + yv*16'sd22) / 16'sd32;
    wire signed [15:0] yB = $signed({8'b0, yv_y}) + (yu * 16'sd110) / 16'sd64;
    function [7:0] clamp8(input signed [15:0] v);
        clamp8 = (v < 0) ? 8'd0 : (v > 255) ? 8'd255 : v[7:0]; endfunction
    wire [31:0] argbyuv = {8'hFF, clamp8(yR), clamp8(yG), clamp8(yB)};

    // ---- palette formats: caller supplies pal_argb already expanded via PAL ROM
    // (the ROM stores ARGB8888 placeholders), so just pass it through.

    // ---- select ----
    // GetExpandFormat: 1555->0,565->1,4444->2, YUV->3(8888 shuffle of yuv? no:
    // yuv already ARGB8888). PAL4/PAL8 expand per PAL_RAM_CTRL - here the ROM is
    // already ARGB8888 so pass through.
    assign argb = (pixfmt==3'd0) ? argb1555
                : (pixfmt==3'd1) ? argb565
                : (pixfmt==3'd2) ? argb4444
                : (pixfmt==3'd3) ? argbyuv
                : (pixfmt==3'd5 || pixfmt==3'd6) ? pal_argb
                : argb1555;  // reserved/bump -> treat as 1555
endmodule
