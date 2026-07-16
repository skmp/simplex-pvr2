// fp_mul_c9 fuzz: y ?= f * k (k = 0..255 vertex colour channel), with the
// datapath's reduced precision. We allow a relative tolerance for the 16-bit
// mantissa truncation, but flag GROSS errors (like always-max / wrong exponent).
#include "Vfp_mul_c9_tb_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static Vfp_mul_c9_tb_top* dut;
static float f32(uint32_t w){ float f; memcpy(&f,&w,4); return f; }
static uint32_t rng=0x9e3779b9;
static uint32_t rnd(){uint32_t x=rng;x^=x<<13;x^=x>>17;x^=x<<5;rng=x;return x;}

int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    dut=new Vfp_mul_c9_tb_top;
    int fails=0,total=0;

    // representative case first: z ~ 0.0026, k = 180 (gray). Expect ~0.47.
    struct { uint32_t f; int k; } fixed[] = {
        {0x3b2ce76b, 255}, {0x3b2ce76b, 180}, {0x3b2ce76b, 128}, {0x3b2ce76b, 1},
        {0x3f000000, 180}, {0x3f800000, 255}, {0x40000000, 100},
    };
    for(auto&c:fixed){
        dut->f=c.f; dut->k=c.k; dut->eval();
        float got=f32(dut->y), exp=f32(c.f)*c.k;
        printf("f=%.6g k=%d -> got=%.6g exp=%.6g  (ratio %.4f)\n",
               f32(c.f), c.k, got, exp, exp!=0?got/exp:0);
    }

    for(int t=0;t<200000;t++){
        // random positive-ish small float (like z=1/w) and k in 0..255
        uint32_t e = 100 + (rnd()%40);           // exp 100..139 -> ~1e-8 .. 1e3
        uint32_t f = (e<<23) | (rnd()&0x7fffff);
        int k = rnd()&0xff;
        dut->f=f; dut->k=k; dut->eval();
        float got=f32(dut->y), exp=f32(f)*(float)k;
        total++;
        if(k==0){ if(dut->y & 0x7fffffff){fails++; if(fails<10)printf("k=0 not zero: %08x\n",dut->y);} continue; }
        float rel = fabsf(got-exp)/fabsf(exp);
        if(rel > 0.02f){   // 2% tolerance for 16-bit-mantissa truncation
            fails++;
            if(fails<20) printf("f=%.6g(%08x) k=%d got=%.6g exp=%.6g rel=%.3f\n",
                f32(f),f,k,got,exp,rel);
        }
    }
    printf("fp_mul_c9: %d/%d within tol\n", total-fails, total);
    printf(fails?"MULC9 FAIL\n":"MULC9 OK\n");
    return fails?1:0;
}
