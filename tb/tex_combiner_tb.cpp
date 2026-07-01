// color_combiner vs refsw ColorCombiner (texenv). Exhaustive over tex/offset/
// ShadInstr x a sweep of base/textel/offset colours.
#include "Vcolor_combiner.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>

static int u8_256(int v){return v+(v>>7);}
static uint32_t g_combiner(bool tex,bool ofs,int si,uint32_t base,uint32_t textel,uint32_t offset){
    if(!tex) return base;
    auto B=[&](uint32_t c,int i){return (c>>(8*i))&0xFF;};
    int r[4];
    for(int i=0;i<4;i++) r[i]=B(base,i);
    if(si==0){ for(int i=0;i<4;i++) r[i]=B(textel,i); }
    else if(si==1){ for(int i=0;i<3;i++) r[i]=B(textel,i)*u8_256(B(base,i))/256; r[3]=B(textel,3); }
    else if(si==2){ int tb=u8_256(B(textel,3)); int cb=256-tb; for(int i=0;i<3;i++) r[i]=(B(textel,i)*tb+B(base,i)*cb)/256; r[3]=B(base,3);}
    else { for(int i=0;i<4;i++) r[i]=B(textel,i)*u8_256(B(base,i))/256; }
    if(ofs){ for(int i=0;i<3;i++){ r[i]=r[i]+B(offset,i); if(r[i]>255)r[i]=255; } }
    return (r[3]<<24)|(r[2]<<16)|(r[1]<<8)|r[0];
}
static uint32_t rng=0x5A5A1234;
static uint32_t rnd(){uint32_t x=rng;x^=x<<13;x^=x>>17;x^=x<<5;rng=x;return x;}

int main(int c,char**v){
    Verilated::commandArgs(c,v);
    auto*d=new Vcolor_combiner;
    int fails=0,total=0;
    for(int t=0;t<2;t++)for(int o=0;o<2;o++)for(int si=0;si<4;si++)
      for(int i=0;i<64;i++){
        uint32_t bs=rnd(),tx=rnd(),of=rnd();
        d->pp_texture=t;d->pp_offset=o;d->shadinstr=si;d->base=bs;d->textel=tx;d->offset=of;d->eval();
        uint32_t g=g_combiner(t,o,si,bs,tx,of); total++;
        if((uint32_t)d->col!=g){fails++;if(fails<10)printf("  t%d o%d si%d %08x/%08x/%08x -> %08x exp %08x\n",t,o,si,bs,tx,of,(uint32_t)d->col,g);}
      }
    printf("color_combiner: %d/%d passed\n",total-fails,total);
    printf(fails?"COMBINER FAIL\n":"COMBINER OK\n");
    return fails?1:0;
}
