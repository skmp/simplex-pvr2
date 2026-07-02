//
// tex_addr - texture texel address generation. Supports mip level selection
// (miplevel input; 0 = base level, matching the old behaviour). Computes the
// byte address in DDR3 for texel (u,v), per refsw TexAddressGen + TexOffsetGen +
// TexStride + fBitsPerPixel, with MipLevel folded in:
//   mip_offset = MipPoint[3 + TexU - MipLevel]   (refsw TexOffsetGen)
//   size       = (8 << TexU) >> MipLevel          (twiddle + stride)
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
    input         mipmapped,    // TCW.MipMapped
    input  [2:0]  pixfmt,       // TCW.PixelFmt
    input  [2:0]  texu,         // TSP.TexU
    input  [2:0]  texv,         // TSP.TexV
    input  [3:0]  miplevel,     // selected mip level (0 = base; 0..TexU+3)
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

    // mip-adjusted texture size: (8<<TexU) >> MipLevel (refsw TexStride/sizeU).
    wire [12:0] size_mip = (13'd8 << texu) >> miplevel;

    // stride
    wire [12:0] stride = (strd_e && scan_e) ? ({8'd0, text_ctrl} << 5)   // *32
                                            : size_mip;

    // ---- twiddle (bit interleave) ----
    // twop: for i-th step take y bit then x bit while that dim's size>1.
    // size_x = 8<<texu, size_y = 8<<texv. We interleave up to 11 bits each.
    // Twiddle sizes. refsw's mipmapped path twiddles with (TexU-MipLevel) for BOTH
    // dims (square mip levels): twop(u,v,TexU-Mip,TexU-Mip). The non-mip path keeps
    // the rectangular (TexU,TexV) sizes. mip_sx/mip_sy pick per mipmapped.
    reg  [21:0] tw;
    integer bx, by, sh;
    reg [10:0] xr, yr;
    integer sx, sy;
    integer sx0, sy0;
    always @(*) begin
        tw = 0; sh = 0;
        xr = u; yr = v;
        // mipmapped: square (8<<TexU)>>Mip on both dims; else (8<<TexU),(8<<TexV)
        sx0 = mipmapped ? ((8 << texu) >> miplevel) : (8 << texu);
        sy0 = mipmapped ? ((8 << texu) >> miplevel) : (8 << texv);
        sx = sx0 >> 1;    // x_sz>>=1
        sy = sy0 >> 1;    // y_sz>>=1
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

    // mip offset (refsw TexOffsetGen): mip_offset = MipPoint[3 + TexU - MipLevel]
    // (texels to skip the coarser mip levels). MipPoint[0..10] from refsw_tile.cpp:
    //   [0]=0x3 [1]=0x4 [2]=0x8 [3]=0x18 [4]=0x58 [5]=0x158 [6]=0x558
    //   [7]=0x1558 [8]=0x5558 [9]=0x15558 [10]=0x55558
    wire [3:0] mip_idx = 4'(3) + {1'b0,texu} - miplevel;   // 0..10
    reg [19:0] mip_off;
    always @(*) begin
        case (mip_idx)
            4'd0:  mip_off = 20'h00003;   // MipPoint[0]
            4'd1:  mip_off = 20'h00004;   // MipPoint[1]
            4'd2:  mip_off = 20'h00008;   // MipPoint[2]
            4'd3:  mip_off = 20'h00018;   // MipPoint[3]
            4'd4:  mip_off = 20'h00058;   // MipPoint[4]
            4'd5:  mip_off = 20'h00158;   // MipPoint[5]
            4'd6:  mip_off = 20'h00558;   // MipPoint[6]
            4'd7:  mip_off = 20'h01558;   // MipPoint[7]
            4'd8:  mip_off = 20'h05558;   // MipPoint[8]
            4'd9:  mip_off = 20'h15558;   // MipPoint[9]
            default: mip_off = 20'h55558; // MipPoint[10]
        endcase
    end
    wire [23:0] mip_add = mipmapped ? {4'd0, mip_off} : 24'd0;

    wire        twiddled = vq | ~scan_e;
    wire [19:0] lin_off  = u + stride * v;
    // FULL-WIDTH texel offset (24b): twiddle/linear + mip base. For tu7 mipmapped
    // the mip base (0x55558) + twiddle can exceed 20 bits, so byte_addr must use
    // this full value (the 20-bit `offset` port only feeds low-bit lane select).
    wire [23:0] off_full = {4'd0, (twiddled ? tw[19:0] : lin_off)} + mip_add;
    assign offset = off_full[19:0];

    // base address (bytes)
    wire [28:0] base = ({8'd0, tcw_addr} << 3) + (vq ? 29'd2048 : 29'd0); // 256*4*2

    // Full (UNMASKED) byte address = base + offset*fbpp/16, matching refsw's
    // emu_vram index. Consumers derive the 64-bit word (byte_addr>>3) and the
    // byte-within-word (byte_addr[2:0]) - the latter is needed to pick the VQ
    // index byte / sub-word lane, so it must NOT be masked away here.
    wire [40:0] byte_off = (off_full * fbpp) >> 4;   // = offset*fbpp/16
    assign byte_addr = base + byte_off[28:0];
endmodule
