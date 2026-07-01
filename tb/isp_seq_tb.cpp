// Functional check of isp_setup_seq against the C golden (build/expected.h).
// Edge constants + flags are bit-exact; the invW plane (z_ddx/z_ddy/z_c) uses
// the 1/C reciprocal-multiply, so it is checked with a small ULP tolerance.
#include <verilated.h>
#include "Visp_setup_seq.h"
#include "expected.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static float asf(uint32_t u){ float f; memcpy(&f,&u,4); return f; }

// load the VRAM hex the same way the synth ROM / sim does
#include <cstdlib>

int main(int argc, char** argv){
    Verilated::commandArgs(argc, argv);
    auto* dut = new Visp_setup_seq;

    // pull scene from the expected table + raw vram.hex
    FILE* f = fopen("build/vram.hex","r");
    if(!f){ printf("no build/vram.hex\n"); return 2; }
    uint32_t w[64]; int n=0; while(n<64 && fscanf(f,"%x",&w[n])==1) n++; fclose(f);
    const int VB=3, STR=7;
    auto VX=[&](int v){return w[VB+v*STR+0];}; auto VY=[&](int v){return w[VB+v*STR+1];};
    auto VZ=[&](int v){return w[VB+v*STR+2];};

    auto tick=[&](){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); };

    dut->reset=1; dut->clk=0; dut->start=0; dut->eval(); tick(); tick(); dut->reset=0;

    dut->isp_tsp = w[0];
    dut->is_quad = g_is_quad;
    dut->v1x=VX(0); dut->v1y=VY(0); dut->v1z=VZ(0);
    dut->v2x=VX(1); dut->v2y=VY(1); dut->v2z=VZ(1);
    dut->v3x=VX(2); dut->v3y=VY(2); dut->v3z=VZ(2);
    dut->v4x=VX(3); dut->v4y=VY(3);
    dut->rect_left=0; dut->rect_top=0;

    dut->start=1; tick(); dut->start=0;
    int cyc=0; while(!dut->done && cyc<2000){ tick(); cyc++; }
    if(!dut->done){ printf("TIMEOUT after %d cycles\n",cyc); return 1; }
    printf("done in %d cycles\n", cyc);

    int fails=0;
    auto exp=[&](const char*nm)->uint32_t{
        for(unsigned i=0;i<sizeof(g_expected)/sizeof(g_expected[0]);i++)
            if(!strcmp(g_expected[i].name,nm)) return g_expected[i].bits;
        printf("?? no expected %s\n",nm); fails++; return 0; };
    auto eq=[&](const char*nm,uint32_t got){ uint32_t e=exp(nm);
        if(got!=e && !(((got&0x7fffffff)==0)&&((e&0x7fffffff)==0))){
            printf("FAIL(exact) %-6s got=%08x exp=%08x\n",nm,got,e); fails++; } };
    // tolerance: within a few ULP (compare as float ulps)
    auto tol=[&](const char*nm,uint32_t got){ uint32_t e=exp(nm);
        int32_t d=(int32_t)got-(int32_t)e; if(d<0)d=-d;
        if(d>4){ printf("FAIL(tol) %-6s got=%08x(%.9g) exp=%08x(%.9g) ulp=%d\n",
            nm,got,asf(got),e,asf(e),d); fails++; } };

    eq("DX12",dut->dx12); eq("DX23",dut->dx23); eq("DX31",dut->dx31); eq("DX41",dut->dx41);
    eq("DY12",dut->dy12); eq("DY23",dut->dy23); eq("DY31",dut->dy31); eq("DY41",dut->dy41);
    eq("C1",dut->c1); eq("C2",dut->c2); eq("C3",dut->c3); eq("C4",dut->c4);
    tol("Z_DDX",dut->z_ddx); tol("Z_DDY",dut->z_ddy); tol("Z_C",dut->z_c);

    int sgn=dut->sgn_neg?-1:1;
    if(sgn!=g_sgn){printf("FAIL sgn %d/%d\n",sgn,g_sgn);fails++;}
    if(dut->t1!=g_T1){printf("FAIL T1\n");fails++;}
    if(dut->t2!=g_T2){printf("FAIL T2\n");fails++;}
    if(dut->t3!=g_T3){printf("FAIL T3\n");fails++;}
    if(dut->t4!=g_T4){printf("FAIL T4\n");fails++;}
    printf("cull=%d\n",dut->cull);

    printf("%s\n", fails? "RESULT: FAIL":"RESULT: PASS");
    delete dut; return fails?1:0;
}
