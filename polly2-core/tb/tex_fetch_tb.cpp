#include "Vtexfetch_tb_top.h"
#include "Vtexfetch_tb_top___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>

static Vtexfetch_tb_top* d;
static void tick(){ d->clk=0; d->eval(); d->clk=1; d->eval(); }
#define VRAM d->rootp->texfetch_tb_top__DOT__vram

// ---- refsw-equivalent expanders ----
static uint32_t ARGB1555(uint16_t w){ return ((w&0x8000)?0xFF000000:0)|(((w>>0)&0x1F)<<3)|(((w>>5)&0x1F)<<11)|(((w>>10)&0x1F)<<19);}
static uint32_t ARGB565(uint16_t w){ return (((w>>0)&0x1F)<<3)|(((w>>5)&0x3F)<<10)|(((w>>11)&0x1F)<<19)|0xFF000000;}
static uint32_t ARGB4444(uint16_t w){ return (((w>>12)&0xF)<<28)|(((w>>0)&0xF)<<4)|(((w>>4)&0xF)<<12)|(((w>>8)&0xF)<<20);}
static uint32_t twiddle(uint32_t x,uint32_t y,uint32_t xsz,uint32_t ysz){
    uint32_t rv=0,sh=0; xsz>>=1; ysz>>=1;
    while(xsz||ysz){ if(ysz){rv|=(y&1)<<sh; y>>=1; ysz>>=1; sh++;} if(xsz){rv|=(x&1)<<sh; x>>=1; xsz>>=1; sh++;} }
    return rv;
}
// seeded RNG (xorshift32, known seed)
static uint32_t rng_s = 0x1234abcd;
static uint32_t rnd(){ uint32_t x=rng_s; x^=x<<13; x^=x>>17; x^=x<<5; rng_s=x; return x; }

static uint32_t TA=0x100;
static uint32_t tsp;
static int fails=0, total=0;

static uint32_t do_fetch(int u,int vv,uint32_t tcw){
    // reset to clear both caches so each case is a fresh miss (tests decode, not
    // cache coherence, under random reused addresses)
    d->req=0; d->reset=1; tick(); tick(); d->reset=0; tick();
    d->u=u; d->v=vv; d->tsp=tsp; d->tcw=tcw; d->text_ctrl=0;
    d->req=1; tick(); d->req=0;
    int g=0; while(!d->ack){ tick(); if(++g>60){printf("TIMEOUT\n");exit(1);} }
    return d->argb;
}
static void put16(uint32_t wi,int lane,uint16_t val){
    uint64_t w=VRAM[wi]; w&=~((uint64_t)0xFFFF<<(16*lane)); w|=((uint64_t)val)<<(16*lane); VRAM[wi]=w;
}
static void put8(uint32_t wi,int bsel,uint8_t val){
    uint64_t w=VRAM[wi]; w&=~((uint64_t)0xFF<<(8*bsel)); w|=((uint64_t)val)<<(8*bsel); VRAM[wi]=w;
}
static void chk(const char*n,uint32_t got,uint32_t exp){
    total++; if(got!=exp){fails++; if(fails<12) printf("  %-8s got %08x exp %08x\n",n,got,exp);}
}

int main(int c,char**v){
    Verilated::commandArgs(c,v);
    d=new Vtexfetch_tb_top;
    for(int i=0;i<65536;i++) VRAM[i]=0;
    d->reset=1; d->req=0; for(int i=0;i<4;i++) tick(); d->reset=0; tick();
    tsp = (5<<0)|(5<<3);          // TexV=5,TexU=5 -> 256x256
    uint32_t base=TA<<3;

    auto pal=[&](int idx){ return 0xFF000000u | (idx<<16)|(idx<<8)|idx; };

    // ---- 64 random cases for each 16bpp twiddled format ----
    for(int fmt=0; fmt<3; fmt++){
        for(int z=0;z<65536;z++) VRAM[z]=0;
       // 0=1555,1=565,2=4444
        const char*nm[]={"1555tw","565tw","4444tw"};
        for(int i=0;i<64;i++){
            int u=rnd()&255, vv=rnd()&255; uint16_t t=rnd()&0xFFFF;
            uint32_t off=twiddle(u,vv,256,256);
            uint32_t byte=base+off*2; uint32_t wi=byte>>3; int lane=(byte>>1)&3;
            put16(wi,lane,t);
            uint32_t tcw=(TA)|((uint32_t)fmt<<27);
            uint32_t got=do_fetch(u,vv,tcw);
            uint32_t exp = fmt==0?ARGB1555(t): fmt==1?ARGB565(t): ARGB4444(t);
            chk(nm[fmt],got,exp);
        }
    }
    // ---- 64 random PAL8 twiddled ----
    for(int z=0;z<65536;z++) VRAM[z]=0;
    // pal8: rv=8 non-VQ -> fbpp=16 -> byte_addr = base + offset. byte-in-line = offset&7.
    for(int i=0;i<64;i++){
        int u=rnd()&255, vv=rnd()&255; uint8_t idx=rnd()&0xFF;
        uint32_t off=twiddle(u,vv,256,256);
        uint32_t byte=base+off; uint32_t wi=byte>>3; int bsel=off&7;
        put8(wi,bsel,idx);
        uint32_t tcw=(TA)|(6u<<27);          // PAL8
        chk("pal8", do_fetch(u,vv,tcw), pal(idx));  // palsel=0 -> idx=local
    }
    // ---- 64 random PAL4 twiddled ----
    for(int z=0;z<65536;z++) VRAM[z]=0;
    // pal4: rv=4 non-VQ -> fbpp=8 -> byte_addr = base + offset/2. nibble = offset&15.
    for(int i=0;i<64;i++){
        int u=rnd()&255, vv=rnd()&255; uint8_t nib=rnd()&0xF;
        uint32_t off=twiddle(u,vv,256,256);
        uint32_t byte=base+off/2; uint32_t wi=byte>>3; int nsel=off&15;
        uint64_t w=VRAM[wi]; w&=~((uint64_t)0xF<<(4*nsel)); w|=((uint64_t)nib)<<(4*nsel); VRAM[wi]=w;
        uint32_t tcw=(TA)|(5u<<27);          // PAL4
        chk("pal4", do_fetch(u,vv,tcw), pal(nib)); // palsel=0 -> idx=nib
    }
    // ---- 64 random VQ 565 ----
    for(int z=0;z<65536;z++) VRAM[z]=0;
    for(int i=0;i<64;i++){
        int u=rnd()&255, vv=rnd()&255; uint8_t index=rnd()&0xFF; uint16_t t=rnd()&0xFFFF;
        uint32_t off=twiddle(u,vv,256,256);
        uint32_t vqbase=(TA<<3)+2048; uint32_t byte=vqbase+off/4; uint32_t wi=byte>>3; int bsel=byte&7;
        put8(wi,bsel,index);
        uint32_t cbwi=TA+index; int lane=off&3;
        put16(cbwi,lane,t);
        uint32_t tcw=(TA)|(1u<<27)|(1u<<30); // 565 + VQ
        chk("vq565", do_fetch(u,vv,tcw), ARGB565(t));
    }

    printf("%d/%d passed  (%d fail)\n", total-fails, total, fails);
    printf(fails? "TEXFETCH FAIL\n":"TEXFETCH OK\n");
    return fails?1:0;
}
