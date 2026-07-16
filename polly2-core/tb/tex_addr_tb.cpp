#include "Vtex_addr.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>

static uint32_t rng=0xC0FFEE11;
static uint32_t rnd(){uint32_t x=rng;x^=x<<13;x^=x>>17;x^=x<<5;rng=x;return x;}

// ---- refsw-equivalent reference ----
static uint32_t twiddle(uint32_t x,uint32_t y,uint32_t xs,uint32_t ys){
    uint32_t rv=0,sh=0; xs>>=1; ys>>=1;
    while(xs||ys){ if(ys){rv|=(y&1)<<sh; y>>=1; ys>>=1; sh++;} if(xs){rv|=(x&1)<<sh; x>>=1; xs>>=1; sh++;} }
    return rv;
}
// fBitsPerPixel (refsw): pal8->8, pal4->4, else 16; VQ: 8*2/(64/rv), else rv*2
static uint32_t ref_fbpp(int pixfmt,int vq){
    uint32_t rv = (pixfmt==6)?8 : (pixfmt==5)?4 : 16;
    if(vq) return 8*2/(64/rv); else return rv*2;
}
static uint32_t ref_stride(int texu,int strdsel,int scan,int text_ctrl){
    if(strdsel && scan) return (text_ctrl&31)*32; else return (8u<<texu);
}
// refsw MipPoint table (refsw_tile.cpp:493)
static const uint32_t MipPoint[11] = {
    0x3, 0x1*4,0x2*4,0x6*4,0x16*4,0x56*4,0x156*4,0x556*4,0x1556*4,0x5556*4,0x15556*4 };
// returns byte_addr, and out params fbpp/offset
static uint32_t ref_addr(uint32_t tcw_addr,int vq,int scan,int strdsel,int mip,int pixfmt,
                         int texu,int texv,int text_ctrl,int u,int v,
                         uint32_t*of_out,uint32_t*fbpp_out){
    int is_pal = (pixfmt==5||pixfmt==6);
    int scan_e = scan && !is_pal;
    int strd_e = strdsel && !is_pal;
    uint32_t fbpp = ref_fbpp(pixfmt,vq);
    uint32_t stride = ref_stride(texu,strd_e,scan_e,text_ctrl);
    // mip base offset at MipLevel 0 = MipPoint[3+TexU]
    uint32_t mip_off = mip ? MipPoint[3+texu] : 0;
    uint32_t off;
    int twiddled = vq || !scan_e;
    // refsw: mipmapped twiddle is square (TexU x TexU); else TexU x TexV
    if(twiddled) off = mip ? twiddle(u,v,8u<<texu,8u<<texu) : twiddle(u,v,8u<<texu,8u<<texv);
    else         off = u + stride*v;
    off += mip_off;
    uint32_t base = (tcw_addr<<3) + (vq?2048:0);
    uint32_t byte = base + off*fbpp/16;
    *of_out=off; *fbpp_out=fbpp;
    return byte & 0x1FFFFFFF;
}

int main(int c,char**v){
    Verilated::commandArgs(c,v);
    auto*d=new Vtex_addr;
    int fails=0,total=0;
    // 64 vectors x each pixel format (0..6, skip 4=bump), x VQ off/on where valid
    int fmts[]={0,1,2,3,5,6};
    for(int fi=0; fi<6; fi++){
      int pixfmt=fmts[fi];
      for(int i=0;i<64;i++){
        uint32_t tcw_addr=rnd()&0x1FFFFF;
        int vq = (pixfmt<=3)?(rnd()&1):0;   // VQ only meaningful for 16bpp/yuv-ish
        int scan=rnd()&1, strdsel=rnd()&1;
        int mip=rnd()&1;                    // exercise mipmapped on/off
        int texu=rnd()%8, texv=rnd()%8;
        if(mip) texv=texu;                  // mip textures are square (TexU==TexV)
        int text_ctrl=rnd()&31;
        int u=rnd()&((8<<texu)-1), vv=rnd()&((8<<texv)-1);
        d->tcw_addr=tcw_addr; d->vq=vq; d->scan=scan; d->stride_sel=strdsel;
        d->mipmapped=mip;
        d->pixfmt=pixfmt; d->texu=texu; d->texv=texv; d->text_ctrl=text_ctrl;
        d->u=u; d->v=vv; d->eval();
        uint32_t rof,rfbpp; uint32_t rbyte=ref_addr(tcw_addr,vq,scan,strdsel,mip,pixfmt,texu,texv,text_ctrl,u,vv,&rof,&rfbpp);
        total++;
        if((uint32_t)d->byte_addr!=rbyte || (uint32_t)d->fbpp!=rfbpp || (uint32_t)d->offset!=(rof&0xFFFFF)){
            fails++;
            if(fails<12) printf("  fmt%d vq%d scan%d strd%d mip%d tu%d tv%d u%d v%d: byte %x/%x fbpp %d/%d off %x/%x\n",
              pixfmt,vq,scan,strdsel,mip,texu,texv,u,vv,(uint32_t)d->byte_addr,rbyte,(int)d->fbpp,rfbpp,(uint32_t)d->offset,rof&0xFFFFF);
        }
      }
    }
    printf("tex_addr: %d/%d passed\n", total-fails, total);
    printf(fails?"ADDR FAIL\n":"ADDR OK\n");
    return fails?1:0;
}
