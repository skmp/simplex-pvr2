// Fuzz fp_add / fp_mul / fp_div against host IEEE-754.
#include <verilated.h>
#include "Vfp_prims.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static uint32_t fbits(float f){ uint32_t u; memcpy(&u,&f,4); return u; }
static float    asf(uint32_t u){ float f; memcpy(&f,&u,4); return f; }

// crude LCG so the test is deterministic
static uint64_t s = 0x123456789abcdef0ULL;
static uint32_t rnd(){ s = s*6364136223846793005ULL + 1442695040888963407ULL; return (uint32_t)(s>>32); }

// Generate "reasonable" floats (avoid Inf/NaN/subnormal which the units flush).
static uint32_t gen(){
    uint32_t sign = rnd() & 1;
    uint32_t exp  = 90 + (rnd() % 80);   // ~2^-37 .. 2^43, comfortably normal
    uint32_t man  = rnd() & 0x7fffff;
    return (sign<<31)|(exp<<23)|man;
}

int main(int argc, char** argv){
    Verilated::commandArgs(argc, argv);
    Vfp_prims* dut = new Vfp_prims;

    long N = 2000000;
    long fa=0, fm=0, fd=0;
    for(long i=0;i<N;i++){
        uint32_t ua = gen(), ub = gen();
        uint32_t sub = rnd() & 1;
        dut->a = ua; dut->b = ub; dut->sub = sub;
        dut->eval();

        float a = asf(ua), b = asf(ub);
        uint32_t exp_add = fbits(sub ? (a-b) : (a+b));
        uint32_t exp_mul = fbits(a*b);
        // skip div-by-zero corner (units clamp, host gives inf)
        uint32_t exp_div = fbits(a/b);

        // Compare; allow signed-zero equivalence (+0 vs -0).
        auto zeq=[](uint32_t x,uint32_t y){ return x==y || ((x&0x7fffffff)==0 && (y&0x7fffffff)==0); };
        if(!zeq(dut->y_add, exp_add)){ if(fa<8) printf("ADD %s a=%.9g b=%.9g  rtl=%08x exp=%08x\n", sub?"sub":"add", a,b, dut->y_add, exp_add); fa++; }
        if(!zeq(dut->y_mul, exp_mul)){ if(fm<8) printf("MUL a=%.9g b=%.9g  rtl=%08x exp=%08x\n", a,b, dut->y_mul, exp_mul); fm++; }
        if(!zeq(dut->y_div, exp_div)){ if(fd<8) printf("DIV a=%.9g b=%.9g  rtl=%08x exp=%08x\n", a,b, dut->y_div, exp_div); fd++; }
    }
    printf("fp_add mismatches: %ld / %ld\n", fa, N);
    printf("fp_mul mismatches: %ld / %ld\n", fm, N);
    printf("fp_div mismatches: %ld / %ld\n", fd, N);
    delete dut;
    return (fa||fm||fd) ? 1 : 0;
}
