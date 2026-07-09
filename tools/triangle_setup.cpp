//
// triangle_setup.cpp - reference PVR ISP + TSP triangle setup.
//
// do_triangle_setup() computes, for one triangle, the full set of interpolation
// planes the hardware rasterizer/shader use:
//   ISP:  4 edge equations (dx,dy,c per edge; edge 4 degenerate for a triangle),
//         the 1/w ("invW") plane (ddx,ddy,c), the winding sign and cull flag.
//   TSP:  up to 10 attribute planes (U, V, base RGBA, offset RGBA), each a
//         perspective-correct plane of z*attribute (ddx,ddy,c).
//
// Every plane is of the standard form  value(x,y) = ddx*x + ddy*y + c , evaluated
// in SCREEN space with the tile origin (xbase,ybase) subtracted (tile-local c).
// For the attribute planes the interpolated quantity is z*attr; the shader later
// multiplies by W = 1/invW(x,y) to recover the perspective-correct attribute.
//
// The math mirrors the RTL (rtl/isp_min/isp_setup_streamed.sv, tsp_setup_min.sv)
// but is done in DOUBLE precision and only rounded to 32-bit float at the very
// end (each stored field is a float). Per the request the body is fully unfolded
// straight-line arithmetic: no helper functions, no reuse.
//
// Build:  g++ -O2 -std=c++17 -o triangle_setup triangle_setup.cpp
//
#include <cstdint>
#include <cstring>
#include <cstdio>
#include "fpm.h"     // fpm<N> / fp_mul<M,N> - the reduced-precision datapath model

// ---- a vertex, exactly the words the primitive iterator / FV FSM read ----
struct Vertex {
    float x, y, z;      // screen x, screen y, z = 1/w
    uint32_t col;       // packed ARGB base colour (0xAARRGGBB)
    uint32_t off;       // packed ARGB offset colour
    float u, v;         // texture coords
};

// ---- everything do_triangle_setup produces ----
struct TriangleSetup {
    // -------- ISP: edge equations (edge i: dx_i*x + dy_i*y + c_i >= 0 inside) --
    // A triangle has 3 real edges; edge 4 is degenerate (dx41=dy41=0, c4=1).
    float dx12, dx23, dx31, dx41;
    float dy12, dy23, dy31, dy41;
    float c1, c2, c3, c4;

    // -------- ISP: 1/w (invW) plane --------
    float ddx_invw, ddy_invw, c_invw;

    // -------- ISP: winding / cull --------
    bool  sgn_neg;      // area > 0  (winding sign fed to the rasterizer)
    bool  cull;         // triangle culled by CullMode vs winding

    // -------- ISP: tile-local integer bbox (inclusive x0/y0, exclusive x1/y1) --
    int   bx0, bx1, by0, by1;

    // -------- TSP: attribute planes (perspective numerator = z*attr) ----------
    // plane index: 0=U 1=V 2..5=base R,G,B,A 6..9=offset R,G,B,A
    float attr_ddx[10];
    float attr_ddy[10];
    float attr_c[10];
    bool  attr_valid[10];   // U/V valid iff textured; offset valid iff offset bit

    // -------- flags decoded from the ISP word --------
    bool gouraud, texture, offset, uv16;
    uint8_t cull_mode;      // ISP[28:27]
};

//
// do_triangle_setup - fully unfolded, double-precision, no helpers, no reuse.
//   isp_word : the ISP control word (for Gouraud/Offset/Texture/CullMode bits)
//   xbase,ybase : tile origin (screen coords), subtracted for tile-local c
//
TriangleSetup do_triangle_setup(uint32_t isp_word,
                                const Vertex& v1, const Vertex& v2, const Vertex& v3,
                                float xbase, float ybase)
{
    TriangleSetup R;

    // ================= decode ISP flags =================
    bool     gouraud   = (isp_word >> 23) & 1u;
    bool     offset_en = (isp_word >> 24) & 1u;
    bool     texture   = (isp_word >> 25) & 1u;
    bool     uv16      = (isp_word >> 22) & 1u;
    uint8_t  cull_mode = (isp_word >> 27) & 3u;
    R.gouraud = gouraud; R.texture = texture; R.offset = offset_en;
    R.uv16 = uv16; R.cull_mode = cull_mode;

    // ================= promote inputs to double =================
    double X1 = (double)v1.x, Y1 = (double)v1.y, Z1 = (double)v1.z;
    double X2 = (double)v2.x, Y2 = (double)v2.y, Z2 = (double)v2.z;
    double X3 = (double)v3.x, Y3 = (double)v3.y, Z3 = (double)v3.z;
    double XB = (double)xbase, YB = (double)ybase;

    // ================================================================
    // ISP SETUP  (rtl/isp_min/isp_setup_streamed.sv, unfolded)
    // ================================================================

    // ---- edge / area difference terms ----
    double d_X1X3 = X1 - X3;
    double d_Y2Y3 = Y2 - Y3;
    double d_Y1Y3 = Y1 - Y3;
    double d_X2X3 = X2 - X3;
    double d_X1X2 = X1 - X2;
    double d_Y1Y2 = Y1 - Y2;
    double d_X2X1 = X2 - X1;
    double d_Y2Y1 = Y2 - Y1;
    double d_X3X1 = X3 - X1;
    double d_Y3Y1 = Y3 - Y1;
    double d_Z2Z1 = Z2 - Z1;
    double d_Z3Z1 = Z3 - Z1;

    // ---- signed triangle area (2x): (X1-X3)*(Y2-Y3) - (Y1-Y3)*(X2-X3) ----
    double tri_area = (d_X1X3 * d_Y2Y3) - (d_Y1Y3 * d_X2X3);

    // ---- 1/w (invW) plane numerators ----
    //   Aa = -(Z2-Z1)*(Y3-Y1) + (Z3-Z1)*(Y2-Y1)
    //   Ba = -(X2-X1)*(Z3-Z1) + (X3-X1)*(Z2-Z1)
    double Aa = (-(d_Z2Z1) * d_Y3Y1) + (d_Z3Z1 * d_Y2Y1);
    double Ba = (-(d_X2X1) * d_Z3Z1) + (d_X3X1 * d_Z2Z1);

    // ---- reciprocal of area, and the invW gradients ----
    double inv_area = 1.0 / tri_area;
    double ddx_invw = -(Aa * inv_area);
    double ddy_invw = -(Ba * inv_area);

    // ---- winding sign: area > 0 -> sgn = -1, else +1 (RTL fpos test) ----
    bool   area_pos = (tri_area > 0.0);
    bool   area_neg = (tri_area < 0.0);
    double sgn = area_pos ? -1.0 : 1.0;

    // ---- cull: CullMode>=2 and winding wrong for the mode ----
    bool   wrong = ((cull_mode & 1u) == 0u && area_neg)
                 || ((cull_mode & 1u) == 1u && area_pos);
    bool   cull  = (cull_mode >= 2u) && wrong;
    R.sgn_neg = area_pos;
    R.cull    = cull;

    // ---- signed edge gradients (dx = sgn*Δx, dy = sgn*Δy) ----
    double DX12 = sgn * d_X1X2;
    double DX23 = sgn * d_X2X3;
    double DX31 = sgn * d_X3X1;
    double DY12 = sgn * d_Y1Y2;
    double DY23 = sgn * d_Y2Y3;
    double DY31 = sgn * d_Y3Y1;

    // ---- tile-local anchor offsets (vertex - tile origin) ----
    double XL1 = X1 - XB;
    double XL2 = X2 - XB;
    double XL3 = X3 - XB;
    double YT1 = Y1 - YB;
    double YT2 = Y2 - YB;
    double YT3 = Y3 - YB;

    // ---- edge constant terms: c = DY*XL - DX*YT (evaluated at the edge's first
    //      vertex), with the top-left fill rule biasing the RAW value by -1 ULP. --
    double C1raw = (DY12 * XL1) - (DX12 * YT1);
    double C2raw = (DY23 * XL2) - (DX23 * YT2);
    double C3raw = (DY31 * XL3) - (DX31 * YT3);

    // top-left test per edge: istl = (dy==0 && dx>0) || dy<0
    bool tl1 = ((DY12 == 0.0 && DX12 > 0.0) || DY12 < 0.0);
    bool tl2 = ((DY23 == 0.0 && DX23 > 0.0) || DY23 < 0.0);
    bool tl3 = ((DY31 == 0.0 && DX31 > 0.0) || DY31 < 0.0);

    // The RTL applies the -1 as a raw-integer decrement of the float bit pattern
    // (subtract 1 ULP) on non-top-left edges. Reproduce that exactly.
    float c1f, c2f, c3f;
    {
        float t = (float)C1raw; uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl1) b -= 1u;
        std::memcpy(&c1f, &b, 4);
    }
    {
        float t = (float)C2raw; uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl2) b -= 1u;
        std::memcpy(&c2f, &b, 4);
    }
    {
        float t = (float)C3raw; uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl3) b -= 1u;
        std::memcpy(&c3f, &b, 4);
    }

    // ---- invW plane constant term (tile-local): c = Z1 - ddx*XL1 - ddy*YT1 ----
    double zc0    = Z1 - (ddx_invw * XL1);
    double c_invw = zc0 - (ddy_invw * YT1);

    // ---- store ISP edge / invW planes (round to float) ----
    R.dx12 = (float)DX12; R.dx23 = (float)DX23; R.dx31 = (float)DX31; R.dx41 = 0.0f;
    R.dy12 = (float)DY12; R.dy23 = (float)DY23; R.dy31 = (float)DY31; R.dy41 = 0.0f;
    R.c1 = c1f; R.c2 = c2f; R.c3 = c3f; R.c4 = 1.0f;
    R.ddx_invw = (float)ddx_invw; R.ddy_invw = (float)ddy_invw; R.c_invw = (float)c_invw;

    // ---- tile-local integer bounding box (floor of screen coords minus tile
    //      origin floor), saturated to [0,2047] then clamped to the 5-bit tile. ----
    // f2i_floor truncates toward zero (magnitude) and saturates to [-2047,2047].
    long fx1 = (long)v1.x; if (fx1 >  2047) fx1 =  2047; if (fx1 < -2047) fx1 = -2047;
    long fx2 = (long)v2.x; if (fx2 >  2047) fx2 =  2047; if (fx2 < -2047) fx2 = -2047;
    long fx3 = (long)v3.x; if (fx3 >  2047) fx3 =  2047; if (fx3 < -2047) fx3 = -2047;
    long fy1 = (long)v1.y; if (fy1 >  2047) fy1 =  2047; if (fy1 < -2047) fy1 = -2047;
    long fy2 = (long)v2.y; if (fy2 >  2047) fy2 =  2047; if (fy2 < -2047) fy2 = -2047;
    long fy3 = (long)v3.y; if (fy3 >  2047) fy3 =  2047; if (fy3 < -2047) fy3 = -2047;
    long fxb = (long)xbase; if (fxb > 2047) fxb = 2047; if (fxb < -2047) fxb = -2047;
    long fyb = (long)ybase; if (fyb > 2047) fyb = 2047; if (fyb < -2047) fyb = -2047;
    long lXa = fx1 - fxb, lXb = fx2 - fxb, lXc = fx3 - fxb;
    long lYa = fy1 - fyb, lYb = fy2 - fyb, lYc = fy3 - fyb;
    long bxmin = lXa < lXb ? (lXa < lXc ? lXa : lXc) : (lXb < lXc ? lXb : lXc);
    long bxmax = lXa > lXb ? (lXa > lXc ? lXa : lXc) : (lXb > lXc ? lXb : lXc);
    long bymin = lYa < lYb ? (lYa < lYc ? lYa : lYc) : (lYb < lYc ? lYb : lYc);
    long bymax = lYa > lYb ? (lYa > lYc ? lYa : lYc) : (lYb > lYc ? lYb : lYc);
    long bx0 = bxmin,     bx1 = bxmax + 1;
    long by0 = bymin,     by1 = bymax + 1;
    R.bx0 = bx0 < 0 ? 0 : (bx0 > 31 ? 31 : (int)bx0);
    R.bx1 = bx1 < 0 ? 0 : (bx1 > 31 ? 31 : (int)bx1);
    R.by0 = by0 < 0 ? 0 : (by0 > 31 ? 31 : (int)by0);
    R.by1 = by1 < 0 ? 0 : (by1 > 31 ? 31 : (int)by1);

    // ================================================================
    // TSP SETUP  (rtl/tsp/tsp_setup_min.sv, unfolded)
    // ================================================================
    // Each attribute plane interpolates z*attr (perspective numerator), with the
    // SAME area/rcp as ISP but recomputed from the TSP vertices (identical here).
    //   da2 = (z2*a2) - (z1*a1),  da3 = (z3*a3) - (z1*a1)
    //   Aa  = da3*(Y2-Y1) - da2*(Y3-Y1)
    //   Ba  = (X3-X1)*da2 - (X2-X1)*da3
    //   C   = (X2-X1)*(Y3-Y1) - (X3-X1)*(Y2-Y1)      [tri area, recomputed]
    //   ddx = -Aa/C,  ddy = -Ba/C
    //   c   = (z1*a1) - ddx*(X1-XB) - ddy*(Y1-YB)
    // For colour the per-vertex attribute is the channel byte (0..255). For a
    // FLAT (non-Gouraud) triangle all three vertices use v3's colour.

    // area & reciprocal, from the TSP vertices (unfolded, independent of ISP's)
    double t_Y2Y1 = Y2 - Y1;
    double t_Y3Y1 = Y3 - Y1;
    double t_X3X1 = X3 - X1;
    double t_X2X1 = X2 - X1;
    double t_area = (t_X2X1 * t_Y3Y1) - (t_X3X1 * t_Y2Y1);
    double t_inv  = 1.0 / t_area;
    double t_XL1  = X1 - XB;
    double t_YT1  = Y1 - YB;

    for (int i = 0; i < 10; i++) { R.attr_valid[i] = false; R.attr_ddx[i]=0; R.attr_ddy[i]=0; R.attr_c[i]=0; }

    // ---- plane 0: U (textured only) ----
    if (texture) {
        double a1 = (double)v1.u, a2 = (double)v2.u, a3 = (double)v3.u;
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[0] = (float)ddx; R.attr_ddy[0] = (float)ddy; R.attr_c[0] = (float)c;
        R.attr_valid[0] = true;
    }
    // ---- plane 1: V (textured only) ----
    if (texture) {
        double a1 = (double)v1.v, a2 = (double)v2.v, a3 = (double)v3.v;
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[1] = (float)ddx; R.attr_ddy[1] = (float)ddy; R.attr_c[1] = (float)c;
        R.attr_valid[1] = true;
    }
    // ---- plane 2: base colour R (byte 16..23) ----
    {
        double a1 = (double)(gouraud ? ((v1.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu));
        double a3 = (double)((v3.col >> 16) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[2] = (float)ddx; R.attr_ddy[2] = (float)ddy; R.attr_c[2] = (float)c;
        R.attr_valid[2] = true;
    }
    // ---- plane 3: base colour G (byte 8..15) ----
    {
        double a1 = (double)(gouraud ? ((v1.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu));
        double a3 = (double)((v3.col >> 8) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[3] = (float)ddx; R.attr_ddy[3] = (float)ddy; R.attr_c[3] = (float)c;
        R.attr_valid[3] = true;
    }
    // ---- plane 4: base colour B (byte 0..7) ----
    {
        double a1 = (double)(gouraud ? (v1.col & 0xFFu) : (v3.col & 0xFFu));
        double a2 = (double)(gouraud ? (v2.col & 0xFFu) : (v3.col & 0xFFu));
        double a3 = (double)(v3.col & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[4] = (float)ddx; R.attr_ddy[4] = (float)ddy; R.attr_c[4] = (float)c;
        R.attr_valid[4] = true;
    }
    // ---- plane 5: base colour A (byte 24..31) ----
    {
        double a1 = (double)(gouraud ? ((v1.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu));
        double a3 = (double)((v3.col >> 24) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[5] = (float)ddx; R.attr_ddy[5] = (float)ddy; R.attr_c[5] = (float)c;
        R.attr_valid[5] = true;
    }
    // ---- plane 6: offset colour R (byte 16..23) ----
    if (offset_en) {
        double a1 = (double)(gouraud ? ((v1.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu));
        double a3 = (double)((v3.off >> 16) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[6] = (float)ddx; R.attr_ddy[6] = (float)ddy; R.attr_c[6] = (float)c;
        R.attr_valid[6] = true;
    }
    // ---- plane 7: offset colour G (byte 8..15) ----
    if (offset_en) {
        double a1 = (double)(gouraud ? ((v1.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu));
        double a3 = (double)((v3.off >> 8) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[7] = (float)ddx; R.attr_ddy[7] = (float)ddy; R.attr_c[7] = (float)c;
        R.attr_valid[7] = true;
    }
    // ---- plane 8: offset colour B (byte 0..7) ----
    if (offset_en) {
        double a1 = (double)(gouraud ? (v1.off & 0xFFu) : (v3.off & 0xFFu));
        double a2 = (double)(gouraud ? (v2.off & 0xFFu) : (v3.off & 0xFFu));
        double a3 = (double)(v3.off & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[8] = (float)ddx; R.attr_ddy[8] = (float)ddy; R.attr_c[8] = (float)c;
        R.attr_valid[8] = true;
    }
    // ---- plane 9: offset colour A (byte 24..31) ----
    if (offset_en) {
        double a1 = (double)(gouraud ? ((v1.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu));
        double a2 = (double)(gouraud ? ((v2.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu));
        double a3 = (double)((v3.off >> 24) & 0xFFu);
        double za1 = Z1 * a1, za2 = Z2 * a2, za3 = Z3 * a3;
        double da2 = za2 - za1;
        double da3 = za3 - za1;
        double Aa_ = (da3 * t_Y2Y1) - (da2 * t_Y3Y1);
        double Ba_ = (t_X3X1 * da2) - (t_X2X1 * da3);
        double ddx = -(Aa_ * t_inv);
        double ddy = -(Ba_ * t_inv);
        double c   = za1 - (ddx * t_XL1) - (ddy * t_YT1);
        R.attr_ddx[9] = (float)ddx; R.attr_ddy[9] = (float)ddy; R.attr_c[9] = (float)c;
        R.attr_valid[9] = true;
    }

    return R;
}

//
// do_triangle_setup_pvr - the SAME setup, but run through the actual PVR datapath
// arithmetic using the reduced-precision helpers in fpm.h:
//   every multiply-accumulate is a mac16 = fp_mul16 (16-bit input mantissa) + fp_add24
//     -> modelled as fp_mul<24,16>(a,b) then fpm<24> add/sub  (both verified
//        bit-exact against rtl/isp_min/{fp_mul16,fp_add24}.sv)
//   the area reciprocal is fp_rcp_fast, a ~16-bit Newton reciprocal -> modelled by
//     quantising 1/area to fpm<16> precision (its accurate-bit budget).
// Each op mirrors the exact mac schedule of isp_setup_streamed.sv / tsp_setup_min.sv.
// Straight-line: every line is one datapath op via the fpm helper operators.
//
// Helper aliases so the body reads like the mac schedule:
//   MUL(a,b)  = fp_mul<24,16>(a,b)      (one fp_mul16 product, 24-bit output)
//   a+b / a-b / -a                       (fpm<24> add24 / negate)
// A "mac" step  d = a*b (+/-) c  is written  MUL(a,b) + c  /  MUL(a,b) - c.
//
static inline fpm<24> MUL(const fpm<24>& a, const fpm<24>& b) {
    // inputs carried at fpm<24> (full float32), multiplied at 16-bit mantissa
    // precision like fp_mul16, output back at 24-bit significand.
    return fp_mul<24, 16>(fpm<16>(a.tof32()), fpm<16>(b.tof32()));
}

TriangleSetup do_triangle_setup_pvr(uint32_t isp_word,
                                    const Vertex& v1, const Vertex& v2, const Vertex& v3,
                                    float xbase, float ybase)
{
    TriangleSetup R;

    // ================= decode ISP flags =================
    bool     gouraud   = (isp_word >> 23) & 1u;
    bool     offset_en = (isp_word >> 24) & 1u;
    bool     texture   = (isp_word >> 25) & 1u;
    bool     uv16      = (isp_word >> 22) & 1u;
    uint8_t  cull_mode = (isp_word >> 27) & 3u;
    R.gouraud = gouraud; R.texture = texture; R.offset = offset_en;
    R.uv16 = uv16; R.cull_mode = cull_mode;

    // ================= inputs as fpm<24> (float32 carried, 16-bit-mant mults) =========
    fpm<24> X1(v1.x), Y1(v1.y), Z1(v1.z);
    fpm<24> X2(v2.x), Y2(v2.y), Z2(v2.z);
    fpm<24> X3(v3.x), Y3(v3.y), Z3(v3.z);
    fpm<24> XB(xbase), YB(ybase);
    fpm<24> ONE(1.0f);

    // ================================================================
    // ISP SETUP  (mac16 schedule of isp_setup_streamed.sv)
    // ================================================================
    // c0..c2: difference terms  d = a*1 - b
    fpm<24> d_X1X3 = MUL(X1, ONE) - X3;
    fpm<24> d_Y2Y3 = MUL(Y2, ONE) - Y3;
    fpm<24> d_Y1Y3 = MUL(Y1, ONE) - Y3;
    fpm<24> d_X2X3 = MUL(X2, ONE) - X3;
    fpm<24> d_X1X2 = MUL(X1, ONE) - X2;
    fpm<24> d_Y1Y2 = MUL(Y1, ONE) - Y2;
    fpm<24> d_X2X1 = MUL(X2, ONE) - X1;
    fpm<24> d_Y2Y1 = MUL(Y2, ONE) - Y1;
    fpm<24> d_X3X1 = MUL(X3, ONE) - X1;
    fpm<24> d_Y3Y1 = MUL(Y3, ONE) - Y1;
    fpm<24> d_Z2Z1 = MUL(Z2, ONE) - Z1;
    fpm<24> d_Z3Z1 = MUL(Z3, ONE) - Z1;

    // c3: area partials + anchors
    fpm<24> P_a0 = MUL(d_X1X3, d_Y2Y3);
    fpm<24> P_a1 = MUL(d_Y1Y3, d_X2X3);
    fpm<24> XL1  = MUL(X1, ONE) - XB;
    fpm<24> YT1  = MUL(Y1, ONE) - YB;

    // c4: area, invW numerator partials, anchor
    fpm<24> tri_area = MUL(P_a0, ONE) - P_a1;
    fpm<24> Aa0 = MUL(d_Z3Z1, d_Y2Y1);
    fpm<24> Ba0 = MUL(d_X3X1, d_Z2Z1);
    fpm<24> XL2 = MUL(X2, ONE) - XB;

    // c5: Aa = -d_Z2Z1*d_Y3Y1 + Aa0 ; Ba = -d_X2X1*d_Z3Z1 + Ba0 ; anchor
    fpm<24> Aa  = MUL(-d_Z2Z1, d_Y3Y1) + Aa0;
    fpm<24> Ba  = MUL(-d_X2X1, d_Z3Z1) + Ba0;
    fpm<24> YT2 = MUL(Y2, ONE) - YB;

    // reciprocal of the area (fp_rcp_fast ~16-bit accuracy).
    fpm<16> inv_area16(1.0f / tri_area.tof32());
    fpm<24> inv_area(inv_area16.tof32());

    // winding sign / cull, from the (reduced-precision) area's sign.
    float    area_f  = tri_area.tof32();
    bool     area_pos = (area_f > 0.0f);
    bool     area_neg = (area_f < 0.0f);
    fpm<24>  sgn = area_pos ? fpm<24>(-1.0f) : fpm<24>(1.0f);
    bool     wrong = ((cull_mode & 1u) == 0u && area_neg)
                   || ((cull_mode & 1u) == 1u && area_pos);
    R.sgn_neg = area_pos;
    R.cull    = (cull_mode >= 2u) && wrong;

    // c6: anchors
    fpm<24> XL3 = MUL(X3, ONE) - XB;
    fpm<24> YT3 = MUL(Y3, ONE) - YB;

    // c7: invW gradients + signed edge dx's
    fpm<24> ddx_invw = -MUL(Aa, inv_area);
    fpm<24> ddy_invw = -MUL(Ba, inv_area);
    fpm<24> DX12 = MUL(sgn, d_X1X2);
    fpm<24> DX23 = MUL(sgn, d_X2X3);

    // c8: remaining signed edge dx/dy's
    fpm<24> DX31 = MUL(sgn, d_X3X1);
    fpm<24> DY12 = MUL(sgn, d_Y1Y2);
    fpm<24> DY23 = MUL(sgn, d_Y2Y3);
    fpm<24> DY31 = MUL(sgn, d_Y3Y1);

    // c9: C*a = DY*XL ; ddx*XL1
    fpm<24> C1a = MUL(DY12, XL1);
    fpm<24> C2a = MUL(DY23, XL2);
    fpm<24> C3a = MUL(DY31, XL3);
    fpm<24> ddxXL1 = MUL(ddx_invw, XL1);

    // c10: Craw = -DX*YT + C*a ; ddy*YT1
    fpm<24> C1raw = MUL(-DX12, YT1) + C1a;
    fpm<24> C2raw = MUL(-DX23, YT2) + C2a;
    fpm<24> C3raw = MUL(-DX31, YT3) + C3a;
    fpm<24> ddyYT1 = MUL(ddy_invw, YT1);

    // c11: top-left rule (raw -1 ULP on non-top-left edges), on the float bits.
    float dx12f = DX12.tof32(), dy12f = DY12.tof32();
    float dx23f = DX23.tof32(), dy23f = DY23.tof32();
    float dx31f = DX31.tof32(), dy31f = DY31.tof32();
    bool tl1 = ((dy12f == 0.0f && dx12f > 0.0f) || dy12f < 0.0f);
    bool tl2 = ((dy23f == 0.0f && dx23f > 0.0f) || dy23f < 0.0f);
    bool tl3 = ((dy31f == 0.0f && dx31f > 0.0f) || dy31f < 0.0f);
    float c1f, c2f, c3f;
    {
        float t = C1raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl1) b -= 1u;
        std::memcpy(&c1f, &b, 4);
    }
    {
        float t = C2raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl2) b -= 1u;
        std::memcpy(&c2f, &b, 4);
    }
    {
        float t = C3raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl3) b -= 1u;
        std::memcpy(&c3f, &b, 4);
    }

    // c11/c12: invW plane constant  zc0 = Z1 - ddx*XL1 ; c_invw = zc0 - ddy*YT1
    fpm<24> zc0    = Z1 - ddxXL1;
    fpm<24> c_invw = zc0 - ddyYT1;

    // ---- store ISP edge / invW planes ----
    R.dx12 = DX12.tof32(); R.dx23 = DX23.tof32(); R.dx31 = DX31.tof32(); R.dx41 = 0.0f;
    R.dy12 = DY12.tof32(); R.dy23 = DY23.tof32(); R.dy31 = DY31.tof32(); R.dy41 = 0.0f;
    R.c1 = c1f; R.c2 = c2f; R.c3 = c3f; R.c4 = 1.0f;
    R.ddx_invw = ddx_invw.tof32(); R.ddy_invw = ddy_invw.tof32(); R.c_invw = c_invw.tof32();

    // tile-local integer bbox (integer path, precision-independent).
    long fx1 = (long)v1.x; if (fx1 >  2047) fx1 =  2047; if (fx1 < -2047) fx1 = -2047;
    long fx2 = (long)v2.x; if (fx2 >  2047) fx2 =  2047; if (fx2 < -2047) fx2 = -2047;
    long fx3 = (long)v3.x; if (fx3 >  2047) fx3 =  2047; if (fx3 < -2047) fx3 = -2047;
    long fy1 = (long)v1.y; if (fy1 >  2047) fy1 =  2047; if (fy1 < -2047) fy1 = -2047;
    long fy2 = (long)v2.y; if (fy2 >  2047) fy2 =  2047; if (fy2 < -2047) fy2 = -2047;
    long fy3 = (long)v3.y; if (fy3 >  2047) fy3 =  2047; if (fy3 < -2047) fy3 = -2047;
    long fxb = (long)xbase; if (fxb > 2047) fxb = 2047; if (fxb < -2047) fxb = -2047;
    long fyb = (long)ybase; if (fyb > 2047) fyb = 2047; if (fyb < -2047) fyb = -2047;
    long lXa = fx1 - fxb, lXb = fx2 - fxb, lXc = fx3 - fxb;
    long lYa = fy1 - fyb, lYb = fy2 - fyb, lYc = fy3 - fyb;
    long bxmin = lXa < lXb ? (lXa < lXc ? lXa : lXc) : (lXb < lXc ? lXb : lXc);
    long bxmax = lXa > lXb ? (lXa > lXc ? lXa : lXc) : (lXb > lXc ? lXb : lXc);
    long bymin = lYa < lYb ? (lYa < lYc ? lYa : lYc) : (lYb < lYc ? lYb : lYc);
    long bymax = lYa > lYb ? (lYa > lYc ? lYa : lYc) : (lYb > lYc ? lYb : lYc);
    long bx0 = bxmin, bx1 = bxmax + 1;
    long by0 = bymin, by1 = bymax + 1;
    R.bx0 = bx0 < 0 ? 0 : (bx0 > 31 ? 31 : (int)bx0);
    R.bx1 = bx1 < 0 ? 0 : (bx1 > 31 ? 31 : (int)bx1);
    R.by0 = by0 < 0 ? 0 : (by0 > 31 ? 31 : (int)by0);
    R.by1 = by1 < 0 ? 0 : (by1 > 31 ? 31 : (int)by1);

    // ================================================================
    // TSP SETUP  (mac16 schedule of tsp_setup_min.sv)
    // ================================================================
    // area & reciprocal recomputed from the TSP vertices.
    fpm<24> t_Y2Y1 = MUL(Y2, ONE) - Y1;
    fpm<24> t_Y3Y1 = MUL(Y3, ONE) - Y1;
    fpm<24> t_X3X1 = MUL(X3, ONE) - X1;
    fpm<24> t_X2X1 = MUL(X2, ONE) - X1;
    fpm<24> t_area = MUL(t_X2X1, t_Y3Y1) - MUL(t_X3X1, t_Y2Y1);
    fpm<16> t_inv16(1.0f / t_area.tof32());
    fpm<24> t_inv(t_inv16.tof32());
    fpm<24> t_XL1 = MUL(X1, ONE) - XB;
    fpm<24> t_YT1 = MUL(Y1, ONE) - YB;

    for (int i = 0; i < 10; i++) { R.attr_valid[i] = false; R.attr_ddx[i]=0; R.attr_ddy[i]=0; R.attr_c[i]=0; }

    // plane 0: U
    if (texture) {
        fpm<24> a1(v1.u), a2(v2.u), a3(v3.u);
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[0] = ddx.tof32(); R.attr_ddy[0] = ddy.tof32(); R.attr_c[0] = c.tof32();
        R.attr_valid[0] = true;
    }
    // plane 1: V
    if (texture) {
        fpm<24> a1(v1.v), a2(v2.v), a3(v3.v);
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[1] = ddx.tof32(); R.attr_ddy[1] = ddy.tof32(); R.attr_c[1] = c.tof32();
        R.attr_valid[1] = true;
    }
    // plane 2: base R
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 16) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[2] = ddx.tof32(); R.attr_ddy[2] = ddy.tof32(); R.attr_c[2] = c.tof32();
        R.attr_valid[2] = true;
    }
    // plane 3: base G
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 8) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[3] = ddx.tof32(); R.attr_ddy[3] = ddy.tof32(); R.attr_c[3] = c.tof32();
        R.attr_valid[3] = true;
    }
    // plane 4: base B
    {
        fpm<24> a1((float)(gouraud ? (v1.col & 0xFFu) : (v3.col & 0xFFu)));
        fpm<24> a2((float)(gouraud ? (v2.col & 0xFFu) : (v3.col & 0xFFu)));
        fpm<24> a3((float)(v3.col & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[4] = ddx.tof32(); R.attr_ddy[4] = ddy.tof32(); R.attr_c[4] = c.tof32();
        R.attr_valid[4] = true;
    }
    // plane 5: base A
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 24) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[5] = ddx.tof32(); R.attr_ddy[5] = ddy.tof32(); R.attr_c[5] = c.tof32();
        R.attr_valid[5] = true;
    }
    // plane 6: offset R
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 16) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[6] = ddx.tof32(); R.attr_ddy[6] = ddy.tof32(); R.attr_c[6] = c.tof32();
        R.attr_valid[6] = true;
    }
    // plane 7: offset G
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 8) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[7] = ddx.tof32(); R.attr_ddy[7] = ddy.tof32(); R.attr_c[7] = c.tof32();
        R.attr_valid[7] = true;
    }
    // plane 8: offset B
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? (v1.off & 0xFFu) : (v3.off & 0xFFu)));
        fpm<24> a2((float)(gouraud ? (v2.off & 0xFFu) : (v3.off & 0xFFu)));
        fpm<24> a3((float)(v3.off & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[8] = ddx.tof32(); R.attr_ddy[8] = ddy.tof32(); R.attr_c[8] = c.tof32();
        R.attr_valid[8] = true;
    }
    // plane 9: offset A
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 24) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[9] = ddx.tof32(); R.attr_ddy[9] = ddy.tof32(); R.attr_c[9] = c.tof32();
        R.attr_valid[9] = true;
    }

    return R;
}


TriangleSetup do_triangle_setup_pvr_existing(uint32_t isp_word,
                                    const Vertex& v1, const Vertex& v2, const Vertex& v3,
                                    float xbase, float ybase)
{
    TriangleSetup R;

    // ================= decode ISP flags =================
    bool     gouraud   = (isp_word >> 23) & 1u;
    bool     offset_en = (isp_word >> 24) & 1u;
    bool     texture   = (isp_word >> 25) & 1u;
    bool     uv16      = (isp_word >> 22) & 1u;
    uint8_t  cull_mode = (isp_word >> 27) & 3u;
    R.gouraud = gouraud; R.texture = texture; R.offset = offset_en;
    R.uv16 = uv16; R.cull_mode = cull_mode;

    // ================= inputs as fpm<24> (float32 carried, 16-bit-mant mults) =========
    fpm<24> X1(v1.x), Y1(v1.y), Z1(v1.z);
    fpm<24> X2(v2.x), Y2(v2.y), Z2(v2.z);
    fpm<24> X3(v3.x), Y3(v3.y), Z3(v3.z);
    fpm<24> XB(xbase), YB(ybase);
    fpm<24> ONE(1.0f);

    // ================================================================
    // ISP SETUP  (mac16 schedule of isp_setup_streamed.sv)
    // ================================================================
    // c0..c2: difference terms  d = a*1 - b
    fpm<24> d_X1X3 = MUL(X1, ONE) - X3;
    fpm<24> d_Y2Y3 = MUL(Y2, ONE) - Y3;
    fpm<24> d_Y1Y3 = MUL(Y1, ONE) - Y3;
    fpm<24> d_X2X3 = MUL(X2, ONE) - X3;
    fpm<24> d_X1X2 = MUL(X1, ONE) - X2;
    fpm<24> d_Y1Y2 = MUL(Y1, ONE) - Y2;
    fpm<24> d_X2X1 = MUL(X2, ONE) - X1;
    fpm<24> d_Y2Y1 = MUL(Y2, ONE) - Y1;
    fpm<24> d_X3X1 = MUL(X3, ONE) - X1;
    fpm<24> d_Y3Y1 = MUL(Y3, ONE) - Y1;
    fpm<24> d_Z2Z1 = MUL(Z2, ONE) - Z1;
    fpm<24> d_Z3Z1 = MUL(Z3, ONE) - Z1;

    // c3: area partials + anchors
    fpm<24> P_a0 = MUL(d_X1X3, d_Y2Y3);
    fpm<24> P_a1 = MUL(d_Y1Y3, d_X2X3);
    fpm<24> XL1  = MUL(X1, ONE) - XB;
    fpm<24> YT1  = MUL(Y1, ONE) - YB;

    // c4: area, invW numerator partials, anchor
    fpm<24> tri_area = MUL(P_a0, ONE) - P_a1;
    fpm<24> Aa0 = MUL(d_Z3Z1, d_Y2Y1);
    fpm<24> Ba0 = MUL(d_X3X1, d_Z2Z1);
    fpm<24> XL2 = MUL(X2, ONE) - XB;

    // c5: Aa = -d_Z2Z1*d_Y3Y1 + Aa0 ; Ba = -d_X2X1*d_Z3Z1 + Ba0 ; anchor
    fpm<24> Aa  = MUL(-d_Z2Z1, d_Y3Y1) + Aa0;
    fpm<24> Ba  = MUL(-d_X2X1, d_Z3Z1) + Ba0;
    fpm<24> YT2 = MUL(Y2, ONE) - YB;

    // reciprocal of the area (fp_rcp_fast ~16-bit accuracy).
    fpm<16> inv_area16(1.0f / tri_area.tof32());
    fpm<24> inv_area(inv_area16.tof32());

    // winding sign / cull, from the (reduced-precision) area's sign.
    float    area_f  = tri_area.tof32();
    bool     area_pos = (area_f > 0.0f);
    bool     area_neg = (area_f < 0.0f);
    fpm<24>  sgn = area_pos ? fpm<24>(-1.0f) : fpm<24>(1.0f);
    bool     wrong = ((cull_mode & 1u) == 0u && area_neg)
                   || ((cull_mode & 1u) == 1u && area_pos);
    R.sgn_neg = area_pos;
    R.cull    = (cull_mode >= 2u) && wrong;

    // c6: anchors
    fpm<24> XL3 = MUL(X3, ONE) - XB;
    fpm<24> YT3 = MUL(Y3, ONE) - YB;

    // c7: invW gradients + signed edge dx's
    fpm<24> ddx_invw = -MUL(Aa, inv_area);
    fpm<24> ddy_invw = -MUL(Ba, inv_area);
    fpm<24> DX12 = MUL(sgn, d_X1X2);
    fpm<24> DX23 = MUL(sgn, d_X2X3);

    // c8: remaining signed edge dx/dy's
    fpm<24> DX31 = MUL(sgn, d_X3X1);
    fpm<24> DY12 = MUL(sgn, d_Y1Y2);
    fpm<24> DY23 = MUL(sgn, d_Y2Y3);
    fpm<24> DY31 = MUL(sgn, d_Y3Y1);

    // c9: C*a = DY*XL ; ddx*XL1
    fpm<24> C1a = MUL(DY12, XL1);
    fpm<24> C2a = MUL(DY23, XL2);
    fpm<24> C3a = MUL(DY31, XL3);
    fpm<24> ddxXL1 = MUL(ddx_invw, XL1);

    // c10: Craw = -DX*YT + C*a ; ddy*YT1
    fpm<24> C1raw = MUL(-DX12, YT1) + C1a;
    fpm<24> C2raw = MUL(-DX23, YT2) + C2a;
    fpm<24> C3raw = MUL(-DX31, YT3) + C3a;
    fpm<24> ddyYT1 = MUL(ddy_invw, YT1);

    // c11: top-left rule (raw -1 ULP on non-top-left edges), on the float bits.
    float dx12f = DX12.tof32(), dy12f = DY12.tof32();
    float dx23f = DX23.tof32(), dy23f = DY23.tof32();
    float dx31f = DX31.tof32(), dy31f = DY31.tof32();
    bool tl1 = ((dy12f == 0.0f && dx12f > 0.0f) || dy12f < 0.0f);
    bool tl2 = ((dy23f == 0.0f && dx23f > 0.0f) || dy23f < 0.0f);
    bool tl3 = ((dy31f == 0.0f && dx31f > 0.0f) || dy31f < 0.0f);
    float c1f, c2f, c3f;
    {
        float t = C1raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl1) b -= 1u;
        std::memcpy(&c1f, &b, 4);
    }
    {
        float t = C2raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl2) b -= 1u;
        std::memcpy(&c2f, &b, 4);
    }
    {
        float t = C3raw.tof32(); uint32_t b; std::memcpy(&b, &t, 4);
        if (!tl3) b -= 1u;
        std::memcpy(&c3f, &b, 4);
    }

    // c11/c12: invW plane constant  zc0 = Z1 - ddx*XL1 ; c_invw = zc0 - ddy*YT1
    fpm<24> zc0    = Z1 - ddxXL1;
    fpm<24> c_invw = zc0 - ddyYT1;

    // ---- store ISP edge / invW planes ----
    R.dx12 = DX12.tof32(); R.dx23 = DX23.tof32(); R.dx31 = DX31.tof32(); R.dx41 = 0.0f;
    R.dy12 = DY12.tof32(); R.dy23 = DY23.tof32(); R.dy31 = DY31.tof32(); R.dy41 = 0.0f;
    R.c1 = c1f; R.c2 = c2f; R.c3 = c3f; R.c4 = 1.0f;
    R.ddx_invw = ddx_invw.tof32(); R.ddy_invw = ddy_invw.tof32(); R.c_invw = c_invw.tof32();

    // tile-local integer bbox (integer path, precision-independent).
    long fx1 = (long)v1.x; if (fx1 >  2047) fx1 =  2047; if (fx1 < -2047) fx1 = -2047;
    long fx2 = (long)v2.x; if (fx2 >  2047) fx2 =  2047; if (fx2 < -2047) fx2 = -2047;
    long fx3 = (long)v3.x; if (fx3 >  2047) fx3 =  2047; if (fx3 < -2047) fx3 = -2047;
    long fy1 = (long)v1.y; if (fy1 >  2047) fy1 =  2047; if (fy1 < -2047) fy1 = -2047;
    long fy2 = (long)v2.y; if (fy2 >  2047) fy2 =  2047; if (fy2 < -2047) fy2 = -2047;
    long fy3 = (long)v3.y; if (fy3 >  2047) fy3 =  2047; if (fy3 < -2047) fy3 = -2047;
    long fxb = (long)xbase; if (fxb > 2047) fxb = 2047; if (fxb < -2047) fxb = -2047;
    long fyb = (long)ybase; if (fyb > 2047) fyb = 2047; if (fyb < -2047) fyb = -2047;
    long lXa = fx1 - fxb, lXb = fx2 - fxb, lXc = fx3 - fxb;
    long lYa = fy1 - fyb, lYb = fy2 - fyb, lYc = fy3 - fyb;
    long bxmin = lXa < lXb ? (lXa < lXc ? lXa : lXc) : (lXb < lXc ? lXb : lXc);
    long bxmax = lXa > lXb ? (lXa > lXc ? lXa : lXc) : (lXb > lXc ? lXb : lXc);
    long bymin = lYa < lYb ? (lYa < lYc ? lYa : lYc) : (lYb < lYc ? lYb : lYc);
    long bymax = lYa > lYb ? (lYa > lYc ? lYa : lYc) : (lYb > lYc ? lYb : lYc);
    long bx0 = bxmin, bx1 = bxmax + 1;
    long by0 = bymin, by1 = bymax + 1;
    R.bx0 = bx0 < 0 ? 0 : (bx0 > 31 ? 31 : (int)bx0);
    R.bx1 = bx1 < 0 ? 0 : (bx1 > 31 ? 31 : (int)bx1);
    R.by0 = by0 < 0 ? 0 : (by0 > 31 ? 31 : (int)by0);
    R.by1 = by1 < 0 ? 0 : (by1 > 31 ? 31 : (int)by1);

    // ================================================================
    // TSP SETUP  (mac16 schedule of tsp_setup_min.sv)
    // ================================================================
    // area & reciprocal recomputed from the TSP vertices.
    fpm<24> t_Y2Y1 = MUL(Y2, ONE) - Y1;
    fpm<24> t_Y3Y1 = MUL(Y3, ONE) - Y1;
    fpm<24> t_X3X1 = MUL(X3, ONE) - X1;
    fpm<24> t_X2X1 = MUL(X2, ONE) - X1;
    fpm<24> t_area = MUL(t_X2X1, t_Y3Y1) - MUL(t_X3X1, t_Y2Y1);
    fpm<16> t_inv16(1.0f / t_area.tof32());
    fpm<24> t_inv(t_inv16.tof32());
    fpm<24> t_XL1 = MUL(X1, ONE) - XB;
    fpm<24> t_YT1 = MUL(Y1, ONE) - YB;

    for (int i = 0; i < 10; i++) { R.attr_valid[i] = false; R.attr_ddx[i]=0; R.attr_ddy[i]=0; R.attr_c[i]=0; }

    // plane 0: U
    if (texture) {
        fpm<24> a1(v1.u), a2(v2.u), a3(v3.u);
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[0] = ddx.tof32(); R.attr_ddy[0] = ddy.tof32(); R.attr_c[0] = c.tof32();
        R.attr_valid[0] = true;
    }
    // plane 1: V
    if (texture) {
        fpm<24> a1(v1.v), a2(v2.v), a3(v3.v);
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[1] = ddx.tof32(); R.attr_ddy[1] = ddy.tof32(); R.attr_c[1] = c.tof32();
        R.attr_valid[1] = true;
    }
    // plane 2: base R
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 16) & 0xFFu) : ((v3.col >> 16) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 16) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[2] = ddx.tof32(); R.attr_ddy[2] = ddy.tof32(); R.attr_c[2] = c.tof32();
        R.attr_valid[2] = true;
    }
    // plane 3: base G
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 8) & 0xFFu) : ((v3.col >> 8) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 8) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[3] = ddx.tof32(); R.attr_ddy[3] = ddy.tof32(); R.attr_c[3] = c.tof32();
        R.attr_valid[3] = true;
    }
    // plane 4: base B
    {
        fpm<24> a1((float)(gouraud ? (v1.col & 0xFFu) : (v3.col & 0xFFu)));
        fpm<24> a2((float)(gouraud ? (v2.col & 0xFFu) : (v3.col & 0xFFu)));
        fpm<24> a3((float)(v3.col & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[4] = ddx.tof32(); R.attr_ddy[4] = ddy.tof32(); R.attr_c[4] = c.tof32();
        R.attr_valid[4] = true;
    }
    // plane 5: base A
    {
        fpm<24> a1((float)(gouraud ? ((v1.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.col >> 24) & 0xFFu) : ((v3.col >> 24) & 0xFFu)));
        fpm<24> a3((float)((v3.col >> 24) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[5] = ddx.tof32(); R.attr_ddy[5] = ddy.tof32(); R.attr_c[5] = c.tof32();
        R.attr_valid[5] = true;
    }
    // plane 6: offset R
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 16) & 0xFFu) : ((v3.off >> 16) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 16) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[6] = ddx.tof32(); R.attr_ddy[6] = ddy.tof32(); R.attr_c[6] = c.tof32();
        R.attr_valid[6] = true;
    }
    // plane 7: offset G
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 8) & 0xFFu) : ((v3.off >> 8) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 8) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[7] = ddx.tof32(); R.attr_ddy[7] = ddy.tof32(); R.attr_c[7] = c.tof32();
        R.attr_valid[7] = true;
    }
    // plane 8: offset B
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? (v1.off & 0xFFu) : (v3.off & 0xFFu)));
        fpm<24> a2((float)(gouraud ? (v2.off & 0xFFu) : (v3.off & 0xFFu)));
        fpm<24> a3((float)(v3.off & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[8] = ddx.tof32(); R.attr_ddy[8] = ddy.tof32(); R.attr_c[8] = c.tof32();
        R.attr_valid[8] = true;
    }
    // plane 9: offset A
    if (offset_en) {
        fpm<24> a1((float)(gouraud ? ((v1.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu)));
        fpm<24> a2((float)(gouraud ? ((v2.off >> 24) & 0xFFu) : ((v3.off >> 24) & 0xFFu)));
        fpm<24> a3((float)((v3.off >> 24) & 0xFFu));
        fpm<24> za1 = MUL(Z1, a1), za2 = MUL(Z2, a2), za3 = MUL(Z3, a3);
        fpm<24> da2 = za2 - za1, da3 = za3 - za1;
        fpm<24> Aa_ = MUL(da3, t_Y2Y1) - MUL(da2, t_Y3Y1);
        fpm<24> Ba_ = MUL(t_X3X1, da2) - MUL(t_X2X1, da3);
        fpm<24> ddx = -MUL(Aa_, t_inv);
        fpm<24> ddy = -MUL(Ba_, t_inv);
        fpm<24> c   = za1 - MUL(ddx, t_XL1) - MUL(ddy, t_YT1);
        R.attr_ddx[9] = ddx.tof32(); R.attr_ddy[9] = ddy.tof32(); R.attr_c[9] = c.tof32();
        R.attr_valid[9] = true;
    }

    return R;
}


// ------------------------------ divergence driver ------------------------------
// Reads the per-line text produced by tools/dump_triangles.py:
//     <isp> <tsp> <tcw> odd=<0|1>
//     <x> <y> <z> <col> <off> <u> <v>     (x3, all 8-hex-digit words)
//     <blank>
// Runs BOTH do_triangle_setup (double reference) and do_triangle_setup_pvr (the
// reduced-precision PVR datapath model, fpm.h) on each triangle and prints the
// ones whose plane coefficients diverge significantly.
//
// Divergence metric: for every plane coefficient (ISP edges, invW plane, and all
// valid attribute planes), the relative error |pvr - d| / max(|d|, tiny). The
// per-triangle score is the MAX over all coefficients. Also flagged: any triangle
// where the pvr and double cull/sgn decisions disagree (a categorical flip).
//
// Since tile origin isn't in the text dump, setup runs with xbase=ybase=0 (whole
// screen); this is exactly the wide-anchor case that stresses the c terms most,
// so it is the right worst-case for divergence hunting.
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#include <algorithm>

static uint32_t parse_hex(const char* s) { return (uint32_t)strtoul(s, nullptr, 16); }
static float u2f(uint32_t b) { float f; std::memcpy(&f, &b, 4); return f; }

// one flagged triangle's rendered report, kept so we can sort by error and emit
// the worst LAST (and write them all to a file).
struct Flagged {
    double worst;
    std::string text;
};

// append a printf-style line to a std::string.
static void appendf(std::string& s, const char* fmt, ...) {
    char buf[512];
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s += buf;
}

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "dumps/tris_menu2.txt";
    double thresh = (argc > 2) ? atof(argv[2]) : 1e-3;   // relative-error threshold

    FILE* fp = std::fopen(path, "r");
    if (!fp) { std::fprintf(stderr, "cannot open %s\n", path); return 1; }

    // flagged reports go to "<input>.pvr_diff.txt" (or arg 3 if given).
    std::string outpath = (argc > 3) ? argv[3] : (std::string(path) + ".pvr_diff.txt");

    char h_isp[64], h_tsp[64], h_tcw[64], h_odd[64];
    char l[3][256];
    long idx = 0;
    double worst_seen = 0.0;
    std::vector<Flagged> flags;

    // read one 4-line block (header + 3 verts); blank lines are skipped by fscanf's
    // %s whitespace handling.
    while (std::fscanf(fp, "%63s %63s %63s %63s", h_isp, h_tsp, h_tcw, h_odd) == 4) {
        // three vertex lines, 7 words each
        char vx[3][7][64];
        bool ok = true;
        for (int k = 0; k < 3; k++) {
            if (std::fscanf(fp, "%63s %63s %63s %63s %63s %63s %63s",
                            vx[k][0], vx[k][1], vx[k][2], vx[k][3], vx[k][4], vx[k][5], vx[k][6]) != 7) {
                ok = false; break;
            }
        }
        if (!ok) break;
        (void)h_tsp; (void)h_tcw; (void)l;

        uint32_t isp = parse_hex(h_isp);
        Vertex vtx[3];
        for (int k = 0; k < 3; k++) {
            vtx[k].x   = u2f(parse_hex(vx[k][0]));
            vtx[k].y   = u2f(parse_hex(vx[k][1]));
            vtx[k].z   = u2f(parse_hex(vx[k][2]));
            vtx[k].col =     parse_hex(vx[k][3]);
            vtx[k].off =     parse_hex(vx[k][4]);
            vtx[k].u   = u2f(parse_hex(vx[k][5]));
            vtx[k].v   = u2f(parse_hex(vx[k][6]));
        }

        TriangleSetup d = do_triangle_setup    (isp, vtx[0], vtx[1], vtx[2], 0.0f, 0.0f);
        TriangleSetup f = do_triangle_setup_pvr(isp, vtx[0], vtx[1], vtx[2], 0.0f, 0.0f);

        // ---- collect every plane as a [ddx, ddy, c] TRIPLE (order matters: the
        //      scoring loop below reads these three-at-a-time as one plane) ----
        double dref[64], fval[64]; const char* dname[64]; int n = 0;
        dref[n]=d.dx12; fval[n]=f.dx12; dname[n]="e1.dx"; n++;
        dref[n]=d.dy12; fval[n]=f.dy12; dname[n]="e1.dy"; n++;
        dref[n]=d.c1;   fval[n]=f.c1;   dname[n]="e1.c";  n++;
        dref[n]=d.dx23; fval[n]=f.dx23; dname[n]="e2.dx"; n++;
        dref[n]=d.dy23; fval[n]=f.dy23; dname[n]="e2.dy"; n++;
        dref[n]=d.c2;   fval[n]=f.c2;   dname[n]="e2.c";  n++;
        dref[n]=d.dx31; fval[n]=f.dx31; dname[n]="e3.dx"; n++;
        dref[n]=d.dy31; fval[n]=f.dy31; dname[n]="e3.dy"; n++;
        dref[n]=d.c3;   fval[n]=f.c3;   dname[n]="e3.c";  n++;
        dref[n]=d.ddx_invw; fval[n]=f.ddx_invw; dname[n]="invw.ddx"; n++;
        dref[n]=d.ddy_invw; fval[n]=f.ddy_invw; dname[n]="invw.ddy"; n++;
        dref[n]=d.c_invw;   fval[n]=f.c_invw;   dname[n]="invw.c";   n++;
        static const char* an[10] = {"U","V","colR","colG","colB","colA","ofsR","ofsG","ofsB","ofsA"};
        for (int i = 0; i < 10; i++) {
            if (!d.attr_valid[i]) continue;
            static char nb[10][3][24];
            std::snprintf(nb[i][0], 24, "%s.ddx", an[i]);
            std::snprintf(nb[i][1], 24, "%s.ddy", an[i]);
            std::snprintf(nb[i][2], 24, "%s.c",   an[i]);
            dref[n]=d.attr_ddx[i]; fval[n]=f.attr_ddx[i]; dname[n]=nb[i][0]; n++;
            dref[n]=d.attr_ddy[i]; fval[n]=f.attr_ddy[i]; dname[n]=nb[i][1]; n++;
            dref[n]=d.attr_c[i];   fval[n]=f.attr_c[i];   dname[n]=nb[i][2]; n++;
        }

        // A plane coefficient only matters as far as it moves the interpolated
        // value across the primitive. Judge each coefficient by how much its
        // float-vs-double error shifts the plane's OUTPUT over the triangle's own
        // screen extent, relative to that plane's output magnitude - so a huge
        // relative error on a ~0 gradient (a flat channel) doesn't false-alarm.
        //   span    = |dx|*W + |dy|*H + |c|   (plane value scale over the bbox)
        //   err_out = |Δdx|*W + |Δdy|*H + |Δc| (how far the outputs drift)
        //   score   = err_out / max(span, tiny)
        // Triangle screen extent (W,H) from its vertices.
        double minx = vtx[0].x, maxx = vtx[0].x, miny = vtx[0].y, maxy = vtx[0].y;
        for (int k = 1; k < 3; k++) {
            if (vtx[k].x < minx) minx = vtx[k].x;
            if (vtx[k].x > maxx) maxx = vtx[k].x;
            if (vtx[k].y < miny) miny = vtx[k].y;
            if (vtx[k].y > maxy) maxy = vtx[k].y;
        }
        double W = maxx - minx, H = maxy - miny;
        if (W < 1.0) W = 1.0;
        if (H < 1.0) H = 1.0;

        // group the flat [ddx,ddy,c] triples back into planes for the output metric.
        // n coefficients come in triples in the order pushed above.
        double worst = 0.0; const char* worst_name = "-"; double worst_d = 0, worst_f = 0;
        for (int i = 0; i + 2 < n; i += 3) {
            double ddx_d = dref[i],   ddx_f = fval[i];
            double ddy_d = dref[i+1], ddy_f = fval[i+1];
            double c_d   = dref[i+2], c_f   = fval[i+2];
            double span    = std::fabs(ddx_d)*W + std::fabs(ddy_d)*H + std::fabs(c_d);
            double err_out = std::fabs(ddx_f-ddx_d)*W + std::fabs(ddy_f-ddy_d)*H + std::fabs(c_f-c_d);
            if (span < 1e-9) span = 1e-9;
            double score = err_out / span;
            if (score > worst) {
                worst = score;
                // report the single coefficient in this plane with the largest drift.
                double e0=std::fabs(ddx_f-ddx_d)*W, e1=std::fabs(ddy_f-ddy_d)*H, e2=std::fabs(c_f-c_d);
                int j = (e0>=e1 && e0>=e2) ? i : (e1>=e2 ? i+1 : i+2);
                worst_name = dname[j]; worst_d = dref[j]; worst_f = fval[j];
            }
        }
        if (worst > worst_seen) worst_seen = worst;

        bool decision_flip = (d.cull != f.cull) || (d.sgn_neg != f.sgn_neg);

        if (worst > thresh || decision_flip) {
            // render this triangle's report into a string; collected and sorted so
            // the largest errors print LAST.
            std::string rep;
            appendf(rep, "tri #%ld  isp=%s  worst rel-err=%.3g on %s (double=%.9g pvr=%.9g)%s\n",
                    idx, h_isp, worst, worst_name, worst_d, worst_f,
                    decision_flip ? "  [CULL/SGN FLIP]" : "");
            if (decision_flip)
                appendf(rep, "        cull d/pvr=%d/%d  sgn_neg d/pvr=%d/%d\n",
                        d.cull, f.cull, d.sgn_neg, f.sgn_neg);
            for (int k = 0; k < 3; k++)
                appendf(rep, "        v%d  x=%.7g y=%.7g z=%.9g\n", k+1, vtx[k].x, vtx[k].y, vtx[k].z);

            // full plane-equation values, ONE LINE PER PLANE: the plane's three
            // coefficients (ddx ddy c) in double, then the same from the PVR model.
            // Coefficients come in triples; the plane label is the part of the first
            // coeff's name before the '.' (e.g. "colR.ddx" -> "colR"), so skipped
            // U/V/offset planes never misalign the labels.
            for (int i = 0; i + 2 < n; i += 3) {
                char pl[24]; std::snprintf(pl, sizeof(pl), "%s", dname[i]);
                char* dot = std::strchr(pl, '.'); if (dot) *dot = '\0';
                appendf(rep, "        %s  double[%.9g %.9g %.9g]\n",
                        pl, dref[i], dref[i+1], dref[i+2]);
                appendf(rep, "        %s pvr[%.9g %.9g %.9g]\n",
                        pl, fval[i], fval[i+1], fval[i+2]);
            }
            flags.push_back({worst, std::move(rep)});
        }
        idx++;
    }
    std::fclose(fp);

    // sort ascending by error so the WORST triangles are printed last.
    std::sort(flags.begin(), flags.end(),
              [](const Flagged& a, const Flagged& b){ return a.worst < b.worst; });

    // write all flagged reports to the output file, worst last.
    FILE* of = std::fopen(outpath.c_str(), "w");
    if (of) {
        for (const auto& fl : flags) std::fputs(fl.text.c_str(), of);
        std::fclose(of);
    }

    // and echo them to stdout in the same order (largest error last).
    for (const auto& fl : flags) std::fputs(fl.text.c_str(), stdout);

    std::printf("\n%ld triangles, %zu flagged (rel-err > %.g or decision flip); "
                "worst rel-err seen = %.3g\n"
                "flagged reports written to %s (worst last)\n",
                idx, flags.size(), thresh, worst_seen, outpath.c_str());
    return 0;
}
