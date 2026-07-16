// Self-checking testbench for tri_setup_top.
//
// Loads the golden expected coefficients (build/expected.h, generated alongside
// build/vram.hex by gen_vectors.c) and asserts every ISP/TSP setup output of
// the DUT matches bit-for-bit.
#include <verilated.h>
#include "Vtri_setup_top.h"
#include "expected.h"
#include <cstdio>
#include <cstdint>
#include <cstring>

static float asf(uint32_t u){ float f; memcpy(&f,&u,4); return f; }

int main(int argc, char** argv){
    Verilated::commandArgs(argc, argv);
    Vtri_setup_top* dut = new Vtri_setup_top;

    dut->is_quad   = g_is_quad;
    dut->rect_left = 0x00000000; // 0.0f
    dut->rect_top  = 0x00000000;
    dut->eval();

    // map output name -> DUT signal
    struct { const char* name; uint32_t val; } got[] = {
        {"ISP_TSP", dut->isp_tsp}, {"TSP", dut->tsp_word}, {"TCW", dut->tcw_word},
        {"DX12", dut->dx12}, {"DX23", dut->dx23}, {"DX31", dut->dx31}, {"DX41", dut->dx41},
        {"DY12", dut->dy12}, {"DY23", dut->dy23}, {"DY31", dut->dy31}, {"DY41", dut->dy41},
        {"C1", dut->c1}, {"C2", dut->c2}, {"C3", dut->c3}, {"C4", dut->c4},
        {"Z_DDX", dut->z_ddx}, {"Z_DDY", dut->z_ddy}, {"Z_C", dut->z_c},
        {"U_DDX", dut->u_ddx}, {"U_DDY", dut->u_ddy}, {"U_C", dut->u_c},
        {"V_DDX", dut->v_ddx}, {"V_DDY", dut->v_ddy}, {"V_C", dut->v_c},
        {"COL0_DDX", dut->col0_ddx}, {"COL0_DDY", dut->col0_ddy}, {"COL0_C", dut->col0_c},
        {"COL1_DDX", dut->col1_ddx}, {"COL1_DDY", dut->col1_ddy}, {"COL1_C", dut->col1_c},
        {"COL2_DDX", dut->col2_ddx}, {"COL2_DDY", dut->col2_ddy}, {"COL2_C", dut->col2_c},
        {"COL3_DDX", dut->col3_ddx}, {"COL3_DDY", dut->col3_ddy}, {"COL3_C", dut->col3_c},
        {"OFS0_DDX", dut->ofs0_ddx}, {"OFS0_DDY", dut->ofs0_ddy}, {"OFS0_C", dut->ofs0_c},
        {"OFS1_DDX", dut->ofs1_ddx}, {"OFS1_DDY", dut->ofs1_ddy}, {"OFS1_C", dut->ofs1_c},
        {"OFS2_DDX", dut->ofs2_ddx}, {"OFS2_DDY", dut->ofs2_ddy}, {"OFS2_C", dut->ofs2_c},
        {"OFS3_DDX", dut->ofs3_ddx}, {"OFS3_DDY", dut->ofs3_ddy}, {"OFS3_C", dut->ofs3_c},
    };
    const int NG = sizeof(got)/sizeof(got[0]);
    const int NE = sizeof(g_expected)/sizeof(g_expected[0]);

    int fails = 0, checks = 0;
    auto zeq=[](uint32_t x,uint32_t y){ return x==y || ((x&0x7fffffff)==0 && (y&0x7fffffff)==0); };

    for (int e = 0; e < NE; e++) {
        int found = 0;
        for (int g = 0; g < NG; g++) {
            if (strcmp(got[g].name, g_expected[e].name) == 0) {
                found = 1; checks++;
                if (!zeq(got[g].val, g_expected[e].bits)) {
                    printf("FAIL %-9s rtl=%08x (%.9g)  exp=%08x (%.9g)\n",
                           g_expected[e].name, got[g].val, asf(got[g].val),
                           g_expected[e].bits, asf(g_expected[e].bits));
                    fails++;
                }
                break;
            }
        }
        if (!found) { printf("?? expected '%s' has no DUT signal mapping\n", g_expected[e].name); fails++; }
    }

    // boolean flags
    int sgn = dut->sgn_neg ? -1 : 1;
    if (sgn != g_sgn)  { printf("FAIL sgn rtl=%d exp=%d\n", sgn, g_sgn); fails++; } checks++;
    if (dut->t1 != g_T1) { printf("FAIL T1 rtl=%d exp=%d\n", dut->t1, g_T1); fails++; } checks++;
    if (dut->t2 != g_T2) { printf("FAIL T2 rtl=%d exp=%d\n", dut->t2, g_T2); fails++; } checks++;
    if (dut->t3 != g_T3) { printf("FAIL T3 rtl=%d exp=%d\n", dut->t3, g_T3); fails++; } checks++;
    if (dut->t4 != g_T4) { printf("FAIL T4 rtl=%d exp=%d\n", dut->t4, g_T4); fails++; } checks++;
    printf("primitive = %s, cull = %d\n", g_is_quad ? "QUAD" : "TRIANGLE", dut->cull);

    printf("\n%d checks, %d failures\n", checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");

    delete dut;
    return fails ? 1 : 0;
}
