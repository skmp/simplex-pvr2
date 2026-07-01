// isp_raster_stream_tb - prove the streamed rasterizer consume path produces a
// bit-identical tile (tag+depth) to the serial path, for many random triangles.
#include "Visp_raster_stream_tb_top.h"
#include "Visp_raster_stream_tb_top___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>

static Visp_raster_stream_tb_top* dut;
#define DTAG   dut->rootp->isp_raster_stream_tb_top__DOT__dt_tag
#define DDEP   dut->rootp->isp_raster_stream_tb_top__DOT__dt_depth
static void tick(){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); }
static uint32_t rng=0xC0FFEE11;
static uint32_t rnd(){uint32_t x=rng;x^=x<<13;x^=x>>17;x^=x<<5;rng=x;return x;}

// a plausible ISP plane set: small-ish edge/invW coeffs so some pixels are inside
static void set_planes(){
    // edges: signed small values; c terms centered so ~half the tile is inside
    dut->c1=0x41000000|(rnd()&0x00ffffff); dut->c2=0xc1000000|(rnd()&0x00ffffff);
    dut->c3=0x40800000|(rnd()&0x00ffffff); dut->c4=0x3f800000;
    dut->dx12=0x3f000000|(rnd()&0x003fffff); dut->dx23=0xbf000000|(rnd()&0x003fffff);
    dut->dx31=0x3e800000|(rnd()&0x003fffff); dut->dx41=0;
    dut->dy12=0xbe800000|(rnd()&0x003fffff); dut->dy23=0x3f000000|(rnd()&0x003fffff);
    dut->dy31=0xbf000000|(rnd()&0x003fffff); dut->dy41=0;
    dut->ddx=0x3c000000|(rnd()&0x0007ffff); dut->ddy=0x3c800000|(rnd()&0x0007ffff);
    dut->c_invw=0x3e000000|(rnd()&0x003fffff);
}

static void run(bool streamed){
    dut->streamed = streamed;
    dut->start=1; tick(); dut->start=0;
    int guard=0;
    while(dut->busy){ tick(); if(++guard>200000){printf("hang streamed=%d\n",streamed);break;} }
    // let any tail settle
    for(int i=0;i<12;i++) tick();
}

int fails=0,total=0;
int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    dut=new Visp_raster_stream_tb_top;
    dut->clk=0; dut->reset=1; dut->start=0; dut->streamed=0;
    for(int i=0;i<4;i++) tick(); dut->reset=0; tick();

    for(int t=0;t<200;t++){
        set_planes();
        dut->depth_mode = 7;            // "always" so every inside pixel writes
        dut->tri_tag = 0x00010000 | t;  // unique per triangle

        // clear the tile buffers (both paths start from same state)
        for(int i=0;i<1024;i++){ DTAG[i]=0; DDEP[i]=0; }
        run(false);                     // serial
        uint32_t ser_tag[1024], ser_dep[1024];
        for(int i=0;i<1024;i++){ ser_tag[i]=DTAG[i]; ser_dep[i]=DDEP[i]; }

        for(int i=0;i<1024;i++){ DTAG[i]=0; DDEP[i]=0; }
        run(true);                      // streamed
        for(int i=0;i<1024;i++){
            total++;
            if(DTAG[i]!=ser_tag[i] || DDEP[i]!=ser_dep[i]){
                fails++;
                if(fails<20) printf("t%d px(%d,%d): stream tag=%08x dep=%08x  serial tag=%08x dep=%08x\n",
                    t, i%32, i/32, DTAG[i], DDEP[i], ser_tag[i], ser_dep[i]);
            }
        }
    }
    printf("isp_raster_stream: %d/%d px match\n", total-fails, total);
    printf(fails?"RASTERSTREAM FAIL\n":"RASTERSTREAM OK\n");
    return fails?1:0;
}
