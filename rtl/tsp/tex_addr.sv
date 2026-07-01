//
// tex_addr - texture texel address generation (no mipmap; MipLevel 0).
// Computes the byte address in DDR3 for texel (u,v), per refsw
// TexAddressGen + TexOffsetGen + TexStride + fBitsPerPixel.
//
//   base = TexAddr<<3  (+ 256*4*2 when VQ)
//   twiddled (VQ or !ScanOrder): offset = twop(u,v,TexU,TexV) = bit-interleave
//   linear:                      offset = u + stride*v
//   stride = (StrideSel&&ScanOrder) ? (TEXT_CONTROL&31)*32 : (8<<TexU)
//   fbpp   = 16 (16bpp) / 8 (pal8) / 4 (pal4); VQ: 8*2/(64/rv)
//   byte address of the 64-bit word = base + offset*fbpp/16   (units per refsw)
//
// twop(x,y): interleave y,x bits (Morton), y first, running until both size
// counters (halved each step) reach 0 - handles non-square by stopping the
// exhausted dimension. Computed combinationally (no LUT).
//
module tex_addr (
    input  [20:0] tcw_addr,     // TCW.TexAddr
    input         vq,           // TCW.VQ_Comp
    input         scan,         // TCW.ScanOrder
    input         stride_sel,   // TCW.StrideSel
    input  [2:0]  pixfmt,       // TCW.PixelFmt
    input  [2:0]  texu,         // TSP.TexU
    input  [2:0]  texv,         // TSP.TexV
    input  [4:0]  text_ctrl,    // TEXT_CONTROL&31 (stride unit)
    input  [10:0] u,            // texel u (0..1023)
    input  [10:0] v,            // texel v

    output [28:0] byte_addr,    // DDR3 byte address of the covering 64-bit word
    output [5:0]  fbpp,         // effective bits-per-pixel (for sub-word select)
    output [19:0] offset        // linear texel offset (for sub-word lane select)
);
    // pal formats force ScanOrder/StrideSel to 0 (refsw)
    wire is_pal   = (pixfmt==3'd5) || (pixfmt==3'd6);
    wire scan_e   = scan       & ~is_pal;
    wire strd_e   = stride_sel & ~is_pal;

    // bits per pixel
    wire [5:0] rv = (pixfmt==3'd6) ? 6'd8 : (pixfmt==3'd5) ? 6'd4 : 6'd16;
    // VQ: 8*2 / (64/rv) = 16*rv/64 = rv/4
    assign fbpp = vq ? (rv >> 2) : (rv << 1);   // refsw: rv*2 non-VQ

    // stride
    wire [12:0] stride = (strd_e && scan_e) ? ({8'd0, text_ctrl} << 5)   // *32
                                            : (13'd8 << texu);

    // ---- twiddle (bit interleave) ----
    // twop: for i-th step take y bit then x bit while that dim's size>1.
    // size_x = 8<<texu, size_y = 8<<texv. We interleave up to 11 bits each.
    reg  [21:0] tw;
    integer bx, by, sh;
    reg [10:0] xr, yr;
    integer sx, sy;
    always @(*) begin
        tw = 0; sh = 0;
        xr = u; yr = v;
        sx = (8 << texu) >> 1;    // x_sz>>=1
        sy = (8 << texv) >> 1;    // y_sz>>=1
        // up to 11 interleave iterations (max 1024)
        for (integer it = 0; it < 11; it = it + 1) begin
            if (sy != 0) begin
                tw[sh] = yr[0]; yr = yr >> 1; sy = sy >> 1; sh = sh + 1;
            end
            if (sx != 0) begin
                tw[sh] = xr[0]; xr = xr >> 1; sx = sx >> 1; sh = sh + 1;
            end
        end
    end

    wire        twiddled = vq | ~scan_e;
    wire [19:0] lin_off  = u + stride * v;
    assign offset = twiddled ? tw[19:0] : lin_off;

    // base address (bytes)
    wire [28:0] base = ({8'd0, tcw_addr} << 3) + (vq ? 29'd2048 : 29'd0); // 256*4*2

    // Full (UNMASKED) byte address = base + offset*fbpp/16, matching refsw's
    // emu_vram index. Consumers derive the 64-bit word (byte_addr>>3) and the
    // byte-within-word (byte_addr[2:0]) - the latter is needed to pick the VQ
    // index byte / sub-word lane, so it must NOT be masked away here.
    wire [40:0] byte_off = (offset * fbpp) >> 4;   // = offset*fbpp/16
    assign byte_addr = base + byte_off[28:0];
endmodule
