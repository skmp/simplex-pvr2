// Self-checking TB for the sequenced (clocked, area-optimized) setup top.
//
// Edge constants (DX/DY/Cn) and flags are bit-exact vs the C golden. Every
// coefficient that involves 1/C - the invW plane and all 10 TSP planes - is
// checked with a small ULP tolerance, since the design uses recip-multiply
// instead of a true divide (the C optimization).
#include <verilated.h>
#include "Vtri_setup_seq_top.h"
#include "expected.h"
#include <cstdio>
#include <cstdint>
#include <cstring>

static float asf(uint32_t u){ float f; memcpy(&f,&u,4); return f; }

int main(int argc, char** argv){
    Verilated::commandArgs(argc, argv);
    auto* dut = new Vtri_setup_seq_top;
    auto tick=[&](){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); };

    dut->reset=1; dut->clk=0; dut->start=0; dut->is_quad=g_is_quad;
    dut->rect_left=0; dut->rect_top=0; dut->eval(); tick(); tick(); dut->reset=0;

    dut->start=1; tick(); dut->start=0;
    int cyc=0; while(!dut->done && cyc<8000){ tick(); cyc++; }
    if(!dut->done){ printf("TIMEOUT %d\n",cyc); return 1; }
    printf("primitive=%s  done in %d cycles\n", g_is_quad?"QUAD":"TRIANGLE", cyc);

    int fails=0, checks=0;
    auto exp=[&](const char*nm)->uint32_t{
        for(unsigned i=0;i<sizeof(g_expected)/sizeof(g_expected[0]);i++)
            if(!strcmp(g_expected[i].name,nm)) return g_expected[i].bits;
        printf("?? no expected %s\n",nm); fails++; return 0; };
    auto eq=[&](const char*nm,uint32_t got){ checks++; uint32_t e=exp(nm);
        if(got!=e && !(((got&0x7fffffff)==0)&&((e&0x7fffffff)==0)))
            { printf("FAIL(exact) %-9s got=%08x exp=%08x\n",nm,got,e); fails++; } };
    auto tol=[&](const char*nm,uint32_t got){ checks++; uint32_t e=exp(nm);
        int64_t d=(int64_t)got-(int64_t)e; if(d<0)d=-d;
        if(d>8){ printf("FAIL(tol) %-9s got=%08x(%.9g) exp=%08x(%.9g) ulp=%lld\n",
            nm,got,asf(got),e,asf(e),(long long)d); fails++; } };

    // exact: edges + flags
    eq("DX12",dut->dx12);eq("DX23",dut->dx23);eq("DX31",dut->dx31);eq("DX41",dut->dx41);
    eq("DY12",dut->dy12);eq("DY23",dut->dy23);eq("DY31",dut->dy31);eq("DY41",dut->dy41);
    eq("C1",dut->c1);eq("C2",dut->c2);eq("C3",dut->c3);eq("C4",dut->c4);
    // tolerance: everything through 1/C
    tol("Z_DDX",dut->z_ddx);tol("Z_DDY",dut->z_ddy);tol("Z_C",dut->z_c);
    tol("U_DDX",dut->u_ddx);tol("U_DDY",dut->u_ddy);tol("U_C",dut->u_c);
    tol("V_DDX",dut->v_ddx);tol("V_DDY",dut->v_ddy);tol("V_C",dut->v_c);
    tol("COL0_DDX",dut->col0_ddx);tol("COL0_DDY",dut->col0_ddy);tol("COL0_C",dut->col0_c);
    tol("COL1_DDX",dut->col1_ddx);tol("COL1_DDY",dut->col1_ddy);tol("COL1_C",dut->col1_c);
    tol("COL2_DDX",dut->col2_ddx);tol("COL2_DDY",dut->col2_ddy);tol("COL2_C",dut->col2_c);
    tol("COL3_DDX",dut->col3_ddx);tol("COL3_DDY",dut->col3_ddy);tol("COL3_C",dut->col3_c);
    tol("OFS0_DDX",dut->ofs0_ddx);tol("OFS0_DDY",dut->ofs0_ddy);tol("OFS0_C",dut->ofs0_c);
    tol("OFS1_DDX",dut->ofs1_ddx);tol("OFS1_DDY",dut->ofs1_ddy);tol("OFS1_C",dut->ofs1_c);
    tol("OFS2_DDX",dut->ofs2_ddx);tol("OFS2_DDY",dut->ofs2_ddy);tol("OFS2_C",dut->ofs2_c);
    tol("OFS3_DDX",dut->ofs3_ddx);tol("OFS3_DDY",dut->ofs3_ddy);tol("OFS3_C",dut->ofs3_c);

    int sgn=dut->sgn_neg?-1:1; checks++;
    if(sgn!=g_sgn){printf("FAIL sgn %d/%d\n",sgn,g_sgn);fails++;}
    checks++; if(dut->t1!=g_T1){printf("FAIL T1\n");fails++;}
    checks++; if(dut->t2!=g_T2){printf("FAIL T2\n");fails++;}
    checks++; if(dut->t3!=g_T3){printf("FAIL T3\n");fails++;}
    checks++; if(dut->t4!=g_T4){printf("FAIL T4\n");fails++;}
    printf("cull=%d\n",dut->cull);

    printf("\n%d checks, %d failures\nRESULT: %s\n", checks, fails, fails?"FAIL":"PASS");
    delete dut; return fails?1:0;
}
