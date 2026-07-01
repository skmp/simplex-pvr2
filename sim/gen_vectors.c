// gen_vectors.c
//
// Golden-vector generator for the refsw2 ISP / TSP triangle-setup RTL units.
//
// This file re-implements the *exact* arithmetic of refsw2's PlaneStepper3
// (refsw_tile.h) and RasterizeTriangle ISP edge setup (refsw_tile.cpp), in
// plain C using IEEE-754 single precision, so the values are bit-identical to
// what the hardware float units must produce.
//
// It emits two artifacts:
//   build/vram.hex      - the VRAM image (ISP_TSP, TSP, TCW, then 3-4 verts),
//                         one 32-bit word per line, as the testbench loads it.
//   build/expected.svh  - SystemVerilog `define's with the expected output
//                         coefficients (as raw 32-bit float bit patterns).
//
// Layout in VRAM (matches decode_pvr_vertices, refsw_tile.cpp:298):
//   word[0] = ISP_TSP
//   word[1] = TSP[0]
//   word[2] = TCW[0]
//   then each vertex: x,y,z (float), [u,v if Texture], color, [offset if Offset]
//
// We keep the vertex layout to the common case: Texture=1, Offset=1, UV float
// (UV_16b=0), single volume. That gives a fixed 9-word vertex:
//   x, y, z, u, v, base_color, offset_color   -> wait, that's 7 words.
// Vertex stride = (3 + skip*(two_volumes+1)) words. With skip configured so the
// stride covers x,y,z,u,v,col,ofs = 7 words -> skip = 4 (3 + 4 = 7).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

typedef uint32_t u32;
typedef uint8_t  u8;

// ---- the refsw2 vertex (core_structs.h:156) ----
typedef struct {
    float x, y, z;
    u8 col[4];
    u8 spc[4];
    float u, v;
} Vertex;

typedef struct { float left, top, right, bottom; } taRECT;

// ---- PlaneStepper3 (refsw_tile.h:30), float-exact ----
typedef struct { float ddx, ddy, c; } Plane;

static Plane plane_setup(const taRECT *rect,
                         const Vertex *v1, const Vertex *v2, const Vertex *v3,
                         float a1, float a2, float a3)
{
    float Aa = ((a3 - a1) * (v2->y - v1->y) - (a2 - a1) * (v3->y - v1->y));
    float Ba = ((v3->x - v1->x) * (a2 - a1) - (v2->x - v1->x) * (a3 - a1));
    float C  = ((v2->x - v1->x) * (v3->y - v1->y) - (v3->x - v1->x) * (v2->y - v1->y));
    if (C == 0) C = 1;
    Plane p;
    p.ddx = -Aa / C;
    p.ddy = -Ba / C;
    p.c   = a1 - p.ddx * (v1->x - rect->left) - p.ddy * (v1->y - rect->top);
    return p;
}

// IsTopLeft (refsw_tile.cpp:349)
static int is_top_left(float x, float y) {
    int IsTop  = (y == 0) && (x > 0);
    int IsLeft = (y < 0);
    return IsTop || IsLeft;
}

// ---- bit helpers ----
static u32 fbits(float f) { u32 u; memcpy(&u, &f, 4); return u; }

// vert_packed_color_ in refsw2 unpacks ARGB byte order into col[] = {b,g,r,a}.
// (vert_packed_color_(to,src): to[0]=src, to[1]=src>>8, to[2]=src>>16, to[3]=src>>24)
static u32 pack_color(u8 b, u8 g, u8 r, u8 a) {
    return (u32)b | ((u32)g << 8) | ((u32)r << 16) | ((u32)a << 24);
}

int main(int argc, char **argv) {
    int is_quad = (argc > 1 && strcmp(argv[1], "quad") == 0);

    // ---------------- Input scene ----------------
    // A single primitive covering part of a 32x32 tile at origin (0,0).
    taRECT area = { 0.0f, 0.0f, 32.0f, 32.0f };

    // ISP_TSP: DepthMode=4 (greater), CullMode=0 (no cull), Texture=1, Offset=1,
    //          Gouraud=1, UV_16b=0, ZWriteDis=0.
    // bit layout (core_structs.h:59): DepthMode[2:0], CullMode[4:3], ZWriteDis[5],
    //   Texture[6], Offset[7], Gouraud[8], UV_16b[9], CacheBypass[10], DCalcCtrl[11]
    u32 isp_tsp = (4u << 0)      // DepthMode = 4
                | (0u << 3)      // CullMode  = 0
                | (0u << 5)      // ZWriteDis
                | (1u << 6)      // Texture
                | (1u << 7)      // Offset
                | (1u << 8);     // Gouraud
    u32 tsp = 0x00000000;        // TexV/TexU=0 etc - opaque, not exercised by setup
    u32 tcw = 0x00000000;

    int nverts = is_quad ? 4 : 3;
    Vertex v[4];
    memset(v, 0, sizeof(v));
    // v1
    v[0].x = 4.0f;  v[0].y = 2.0f;  v[0].z = 0.5f;    // z is 1/w
    v[0].u = 0.0f;  v[0].v = 0.0f;
    v[0].col[0]=10; v[0].col[1]=20; v[0].col[2]=30; v[0].col[3]=255; // b,g,r,a
    v[0].spc[0]=1;  v[0].spc[1]=2;  v[0].spc[2]=3;  v[0].spc[3]=4;
    // v2
    v[1].x = 28.0f; v[1].y = 6.0f;  v[1].z = 0.25f;
    v[1].u = 1.0f;  v[1].v = 0.0f;
    v[1].col[0]=200; v[1].col[1]=150; v[1].col[2]=100; v[1].col[3]=255;
    v[1].spc[0]=5;  v[1].spc[1]=6;  v[1].spc[2]=7;  v[1].spc[3]=8;
    // v3
    v[2].x = 10.0f; v[2].y = 26.0f; v[2].z = 0.125f;
    v[2].u = 0.0f;  v[2].v = 1.0f;
    v[2].col[0]=60; v[2].col[1]=70; v[2].col[2]=80; v[2].col[3]=255;
    v[2].spc[0]=9;  v[2].spc[1]=10; v[2].spc[2]=11; v[2].spc[3]=12;
    // v4 (quad only) - completes the parallelogram-ish quad
    v[3].x = 26.0f; v[3].y = 30.0f; v[3].z = 0.1f;
    v[3].u = 1.0f;  v[3].v = 1.0f;
    v[3].col[0]=120; v[3].col[1]=130; v[3].col[2]=140; v[3].col[3]=255;
    v[3].spc[0]=13;  v[3].spc[1]=14;  v[3].spc[2]=15;  v[3].spc[3]=16;

    int gouraud = (isp_tsp >> 8) & 1;

    // ---------------- ISP edge setup (refsw_tile.cpp:556) ----------------
    float X1=v[0].x, X2=v[1].x, X3=v[2].x, X4=v[3].x;
    float Y1=v[0].y, Y2=v[1].y, Y3=v[2].y, Y4=v[3].y;
    int has_v4 = is_quad;

    // tri_area uses v1,v2,v3 only (same for tri & quad)
    float tri_area = ((X1 - X3) * (Y2 - Y3) - (Y1 - Y3) * (X2 - X3));
    int sgn = 1;
    if (tri_area > 0) sgn = -1;
    float sgnf = (float)sgn;

    float DX12 = sgnf * (X1 - X2);
    float DX23 = sgnf * (X2 - X3);
    float DX31 = has_v4 ? sgnf * (X3 - X4) : sgnf * (X3 - X1);
    float DX41 = has_v4 ? sgnf * (X4 - X1) : 0.0f;
    float DY12 = sgnf * (Y1 - Y2);
    float DY23 = sgnf * (Y2 - Y3);
    float DY31 = has_v4 ? sgnf * (Y3 - Y4) : sgnf * (Y3 - Y1);
    float DY41 = has_v4 ? sgnf * (Y4 - Y1) : 0.0f;

    float C1 = DY12 * (X1 - area.left) - DX12 * (Y1 - area.top);
    float C2 = DY23 * (X2 - area.left) - DX23 * (Y2 - area.top);
    float C3 = DY31 * (X3 - area.left) - DX31 * (Y3 - area.top);
    float C4 = has_v4 ? (DY41 * (X4 - area.left) - DX41 * (Y4 - area.top)) : 1.0f;

    int T1 = is_top_left(X2 - X1, Y2 - Y1);
    int T2 = is_top_left(X3 - X2, Y3 - Y2);
    int T3, T4;
    if (!has_v4) { T3 = is_top_left(X1 - X3, Y1 - Y3); T4 = 1; }
    else         { T3 = is_top_left(X4 - X3, Y4 - Y3); T4 = is_top_left(X1 - X4, Y1 - Y4); }

    // depth (invW) plane
    Plane Z = plane_setup(&area, &v[0], &v[1], &v[2], v[0].z, v[1].z, v[2].z);

    // ---------------- TSP setup (refsw_tile.h:84) ----------------
    Plane U = plane_setup(&area, &v[0], &v[1], &v[2],
                          v[0].u*v[0].z, v[1].u*v[1].z, v[2].u*v[2].z);
    Plane V = plane_setup(&area, &v[0], &v[1], &v[2],
                          v[0].v*v[0].z, v[1].v*v[1].z, v[2].v*v[2].z);
    Plane Col[4], Ofs[4];
    for (int i = 0; i < 4; i++) {
        float a1,a2,a3,o1,o2,o3;
        if (gouraud) {
            a1=v[0].col[i]*v[0].z; a2=v[1].col[i]*v[1].z; a3=v[2].col[i]*v[2].z;
            o1=v[0].spc[i]*v[0].z; o2=v[1].spc[i]*v[1].z; o3=v[2].spc[i]*v[2].z;
        } else {
            a1=v[2].col[i]*v[0].z; a2=v[2].col[i]*v[1].z; a3=v[2].col[i]*v[2].z;
            o1=v[2].spc[i]*v[0].z; o2=v[2].spc[i]*v[1].z; o3=v[2].spc[i]*v[2].z;
        }
        Col[i] = plane_setup(&area, &v[0], &v[1], &v[2], a1, a2, a3);
        Ofs[i] = plane_setup(&area, &v[0], &v[1], &v[2], o1, o2, o3);
    }

    // ---------------- emit vram.hex ----------------
    // vertex stride: x,y,z,u,v,col,ofs = 7 words (Texture=1, Offset=1, UV float)
    FILE *fh = fopen("build/vram.hex", "w");
    if (!fh) { perror("vram.hex"); return 1; }
    fprintf(fh, "%08x\n", isp_tsp);
    fprintf(fh, "%08x\n", tsp);
    fprintf(fh, "%08x\n", tcw);
    for (int i = 0; i < nverts; i++) {
        fprintf(fh, "%08x\n", fbits(v[i].x));
        fprintf(fh, "%08x\n", fbits(v[i].y));
        fprintf(fh, "%08x\n", fbits(v[i].z));
        fprintf(fh, "%08x\n", fbits(v[i].u));
        fprintf(fh, "%08x\n", fbits(v[i].v));
        fprintf(fh, "%08x\n", pack_color(v[i].col[0],v[i].col[1],v[i].col[2],v[i].col[3]));
        fprintf(fh, "%08x\n", pack_color(v[i].spc[0],v[i].spc[1],v[i].spc[2],v[i].spc[3]));
    }
    fclose(fh);

    // ---------------- emit expected.svh ----------------
    FILE *fe = fopen("build/expected.svh", "w");
    if (!fe) { perror("expected.svh"); return 1; }
    fprintf(fe, "// Auto-generated by gen_vectors.c - DO NOT EDIT\n");
    fprintf(fe, "// Expected ISP/TSP setup coefficients as raw 32-bit float bits.\n\n");

    fprintf(fe, "// ---- scene control words ----\n");
    fprintf(fe, "`define EXP_ISP_TSP   32'h%08x\n", isp_tsp);
    fprintf(fe, "`define EXP_TSP       32'h%08x\n", tsp);
    fprintf(fe, "`define EXP_TCW       32'h%08x\n", tcw);
    fprintf(fe, "`define EXP_SGN       %d\n\n", sgn);

    fprintf(fe, "// ---- ISP edge constants ----\n");
    fprintf(fe, "`define EXP_DX12 32'h%08x\n", fbits(DX12));
    fprintf(fe, "`define EXP_DX23 32'h%08x\n", fbits(DX23));
    fprintf(fe, "`define EXP_DX31 32'h%08x\n", fbits(DX31));
    fprintf(fe, "`define EXP_DX41 32'h%08x\n", fbits(DX41));
    fprintf(fe, "`define EXP_DY12 32'h%08x\n", fbits(DY12));
    fprintf(fe, "`define EXP_DY23 32'h%08x\n", fbits(DY23));
    fprintf(fe, "`define EXP_DY31 32'h%08x\n", fbits(DY31));
    fprintf(fe, "`define EXP_DY41 32'h%08x\n", fbits(DY41));
    fprintf(fe, "`define EXP_C1   32'h%08x\n", fbits(C1));
    fprintf(fe, "`define EXP_C2   32'h%08x\n", fbits(C2));
    fprintf(fe, "`define EXP_C3   32'h%08x\n", fbits(C3));
    fprintf(fe, "`define EXP_C4   32'h%08x\n", fbits(C4));
    fprintf(fe, "`define EXP_T1   %d\n", T1);
    fprintf(fe, "`define EXP_T2   %d\n", T2);
    fprintf(fe, "`define EXP_T3   %d\n", T3);
    fprintf(fe, "`define EXP_T4   %d\n\n", T4);

    fprintf(fe, "// ---- ISP depth (invW) plane ----\n");
    fprintf(fe, "`define EXP_Z_DDX 32'h%08x\n", fbits(Z.ddx));
    fprintf(fe, "`define EXP_Z_DDY 32'h%08x\n", fbits(Z.ddy));
    fprintf(fe, "`define EXP_Z_C   32'h%08x\n\n", fbits(Z.c));

    fprintf(fe, "// ---- TSP texture planes ----\n");
    fprintf(fe, "`define EXP_U_DDX 32'h%08x\n", fbits(U.ddx));
    fprintf(fe, "`define EXP_U_DDY 32'h%08x\n", fbits(U.ddy));
    fprintf(fe, "`define EXP_U_C   32'h%08x\n", fbits(U.c));
    fprintf(fe, "`define EXP_V_DDX 32'h%08x\n", fbits(V.ddx));
    fprintf(fe, "`define EXP_V_DDY 32'h%08x\n", fbits(V.ddy));
    fprintf(fe, "`define EXP_V_C   32'h%08x\n\n", fbits(V.c));

    fprintf(fe, "// ---- TSP base color planes (RGBA = col[2],col[1],col[0],col[3]) ----\n");
    const char *cn[4] = {"B","G","R","A"};
    for (int i = 0; i < 4; i++) {
        fprintf(fe, "`define EXP_COL%s_DDX 32'h%08x\n", cn[i], fbits(Col[i].ddx));
        fprintf(fe, "`define EXP_COL%s_DDY 32'h%08x\n", cn[i], fbits(Col[i].ddy));
        fprintf(fe, "`define EXP_COL%s_C   32'h%08x\n", cn[i], fbits(Col[i].c));
    }
    fprintf(fe, "\n// ---- TSP offset color planes ----\n");
    for (int i = 0; i < 4; i++) {
        fprintf(fe, "`define EXP_OFS%s_DDX 32'h%08x\n", cn[i], fbits(Ofs[i].ddx));
        fprintf(fe, "`define EXP_OFS%s_DDY 32'h%08x\n", cn[i], fbits(Ofs[i].ddy));
        fprintf(fe, "`define EXP_OFS%s_C   32'h%08x\n", cn[i], fbits(Ofs[i].c));
    }
    fclose(fe);

    // ---------------- emit vram.mif (synthesizable ROM init for Quartus) ----
    // 64 words deep x 32 bits to match tri_setup_seq_top's VRAM_WORDS=64
    // (ram_init_file depth must equal the inferred memory depth). Addresses past
    // the vertex data are zero-filled.
    {
        FILE *fm = fopen("build/vram.mif", "w");
        if (!fm) { perror("vram.mif"); return 1; }
        u32 words[64]; memset(words, 0, sizeof(words));
        int wi = 0;
        words[wi++] = isp_tsp; words[wi++] = tsp; words[wi++] = tcw;
        for (int i = 0; i < nverts; i++) {
            words[wi++] = fbits(v[i].x); words[wi++] = fbits(v[i].y); words[wi++] = fbits(v[i].z);
            words[wi++] = fbits(v[i].u); words[wi++] = fbits(v[i].v);
            words[wi++] = pack_color(v[i].col[0],v[i].col[1],v[i].col[2],v[i].col[3]);
            words[wi++] = pack_color(v[i].spc[0],v[i].spc[1],v[i].spc[2],v[i].spc[3]);
        }
        fprintf(fm, "DEPTH = 64;\nWIDTH = 32;\nADDRESS_RADIX = HEX;\nDATA_RADIX = HEX;\nCONTENT BEGIN\n");
        for (int i = 0; i < 64; i++) fprintf(fm, "  %02x : %08x;\n", i, words[i]);
        fprintf(fm, "END;\n");
        fclose(fm);
    }

    // ---------------- emit expected.h (C header for the Verilator TB) -------
    FILE *fc = fopen("build/expected.h", "w");
    if (!fc) { perror("expected.h"); return 1; }
    fprintf(fc, "// Auto-generated by gen_vectors.c - DO NOT EDIT\n");
    fprintf(fc, "#pragma once\n#include <stdint.h>\n\n");
    fprintf(fc, "static const struct { const char* name; uint32_t bits; } g_expected[] = {\n");
    #define EXP(n, val) fprintf(fc, "  {\"%s\", 0x%08xu},\n", n, (val))
    EXP("ISP_TSP", isp_tsp); EXP("TSP", tsp); EXP("TCW", tcw);
    EXP("DX12", fbits(DX12)); EXP("DX23", fbits(DX23)); EXP("DX31", fbits(DX31)); EXP("DX41", fbits(DX41));
    EXP("DY12", fbits(DY12)); EXP("DY23", fbits(DY23)); EXP("DY31", fbits(DY31)); EXP("DY41", fbits(DY41));
    EXP("C1", fbits(C1)); EXP("C2", fbits(C2)); EXP("C3", fbits(C3)); EXP("C4", fbits(C4));
    EXP("Z_DDX", fbits(Z.ddx)); EXP("Z_DDY", fbits(Z.ddy)); EXP("Z_C", fbits(Z.c));
    EXP("U_DDX", fbits(U.ddx)); EXP("U_DDY", fbits(U.ddy)); EXP("U_C", fbits(U.c));
    EXP("V_DDX", fbits(V.ddx)); EXP("V_DDY", fbits(V.ddy)); EXP("V_C", fbits(V.c));
    for (int i = 0; i < 4; i++) {
        char nm[16];
        sprintf(nm, "COL%d_DDX", i); EXP(nm, fbits(Col[i].ddx));
        sprintf(nm, "COL%d_DDY", i); EXP(nm, fbits(Col[i].ddy));
        sprintf(nm, "COL%d_C",   i); EXP(nm, fbits(Col[i].c));
    }
    for (int i = 0; i < 4; i++) {
        char nm[16];
        sprintf(nm, "OFS%d_DDX", i); EXP(nm, fbits(Ofs[i].ddx));
        sprintf(nm, "OFS%d_DDY", i); EXP(nm, fbits(Ofs[i].ddy));
        sprintf(nm, "OFS%d_C",   i); EXP(nm, fbits(Ofs[i].c));
    }
    #undef EXP
    fprintf(fc, "};\n");
    fprintf(fc, "static const int g_sgn = %d;\n", sgn);
    fprintf(fc, "static const int g_is_quad = %d;\n", is_quad);
    fprintf(fc, "static const int g_T1 = %d, g_T2 = %d, g_T3 = %d, g_T4 = %d;\n", T1, T2, T3, T4);
    fclose(fc);

    printf("Generated build/vram.hex, build/expected.svh and build/expected.h\n");
    printf("  tri_area = %g, sgn = %d\n", tri_area, sgn);
    printf("  Z plane: ddx=%g ddy=%g c=%g\n", Z.ddx, Z.ddy, Z.c);
    return 0;
}
