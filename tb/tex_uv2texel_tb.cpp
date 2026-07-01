#include "Vtex_uv2texel.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static uint32_t rng=0xBEEF1234;
static uint32_t rnd(){uint32_t x=rng;x^=x<<13;x^=x>>17;x^=x<<5;rng=x;return x;}
static uint32_t f2b(float f){uint32_t u;memcpy(&u,&f,4);return u;}

// refsw ClampFlip
static int clampflip(int clamp,int flip,int coord,int size){
    if(clamp){ if(coord<0)coord=0; else if(coord>=size)coord=size-1; }
    else if(flip){ coord &= size*2-1; if(coord&size) coord ^= size*2-1; }
    else { coord &= size-1; }
    return coord;
}

int main(int c,char**v){
    Verilated::commandArgs(c,v);
    auto*d=new Vtex_uv2texel;
    int fails=0,total=0;
    // all 16 clamp/flip combos, 64 random (u,v,texu,texv) each
    for(int mode=0;mode<16;mode++){
      int clampu=(mode>>0)&1, clampv=(mode>>1)&1, flipu=(mode>>2)&1, flipv=(mode>>3)&1;
      for(int i=0;i<64;i++){
        int texu=rnd()%8, texv=rnd()%8;
        int sizeU=8<<texu, sizeV=8<<texv;
        // random u,v in [0,~2) so coords land in and slightly beyond range
        float fu = (float)(rnd()%2048)/1024.0f;   // 0..~2
        float fv = (float)(rnd()%2048)/1024.0f;
        d->u=f2b(fu); d->v=f2b(fv); d->texu=texu; d->texv=texv;
        d->clampu=clampu; d->clampv=clampv; d->flipu=flipu; d->flipv=flipv; d->eval();

        // ref: ui = fu*sizeU*256 (truncate), texel = ui>>8, frac = ui&255
        long ui=(long)(fu*sizeU*256.0f), vi=(long)(fv*sizeV*256.0f);
        int u0=ui>>8, v0=vi>>8, u1=u0+1, v1=v0+1;
        int r_c00u=clampflip(clampu,flipu,u1,sizeU), r_c00v=clampflip(clampv,flipv,v1,sizeV);
        int r_c01u=clampflip(clampu,flipu,u0,sizeU), r_c01v=clampflip(clampv,flipv,v1,sizeV);
        int r_c10u=clampflip(clampu,flipu,u1,sizeU), r_c10v=clampflip(clampv,flipv,v0,sizeV);
        int r_c11u=clampflip(clampu,flipu,u0,sizeU), r_c11v=clampflip(clampv,flipv,v0,sizeV);
        int r_uf=ui&255, r_vf=vi&255;
        total++;
        bool ok = (int)d->c00u==r_c00u&&(int)d->c00v==r_c00v&&(int)d->c01u==r_c01u&&(int)d->c01v==r_c01v
                &&(int)d->c10u==r_c10u&&(int)d->c10v==r_c10v&&(int)d->c11u==r_c11u&&(int)d->c11v==r_c11v;
        // fraction tolerance +-1 (float->fixed rounding)
        int dfu=abs((int)d->ufrac-r_uf), dfv=abs((int)d->vfrac-r_vf);
        if(!ok || dfu>1 || dfv>1){
            fails++;
            if(fails<12) printf("  m%x tu%d tv%d fu%.3f fv%.3f: c11(%d,%d)/(%d,%d) frac(%d,%d)/(%d,%d)\n",
              mode,texu,texv,fu,fv,(int)d->c11u,(int)d->c11v,r_c11u,r_c11v,(int)d->ufrac,(int)d->vfrac,r_uf,r_vf);
        }
      }
    }
    printf("tex_uv2texel: %d/%d passed\n", total-fails, total);
    printf(fails?"UV FAIL\n":"UV OK\n");
    return fails?1:0;
}
