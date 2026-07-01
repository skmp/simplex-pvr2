// Full TSP pipeline co-sim: drive tile_engine_top through the command interface
// (regs -> ISP_SETUP -> ISP_RASTERIZE -> TSP_SETUP -> TSP_SHADE), preload the
// texture VRAM, then compare the shaded colour buffer against a refsw-equivalent
// C model. Exercises interpolation + texture fetch/filter + ColorCombiner.
#include "Vtile_engine_top.h"
#include "Vtile_engine_top___024root.h"
#include "Vtile_engine_top_tile_engine_top.h"
#include "Vtile_engine_top_sysmem_lite.h"
#include "verilated.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>

static Vtile_engine_top* dut;
static void set_clk(int v){ dut->rootp->tile_engine_top->u_sysmem->clk_r=v; }
static void tick(){ set_clk(0); dut->eval(); set_clk(1); dut->eval(); }
static uint32_t f2b(float f){uint32_t u;memcpy(&u,&f,4);return u;}
static float b2f(uint32_t u){float f;memcpy(&f,&u,4);return f;}
#define TOP dut->rootp->tile_engine_top
#define VRAM TOP->u_texmem__DOT__vram

static void submit(int cmd,unsigned data){
    while(!dut->ready)tick();
    dut->cmd=cmd;dut->data=data;dut->cmd_valid=1;tick();dut->cmd_valid=0;
    int g=0; while(!dut->cmd_done){tick();if(++g>200000){printf("TIMEOUT cmd %d\n",cmd);exit(1);}} tick();
}
// colour buffer readback: banked col_buf[bank][addr], bank=x&7, addr=(y<<2)|(x>>3)
static uint32_t col_read(int x,int y){
    int b=x&7, a=((y&31)<<2)|((x>>3)&3);
    switch(b){
      case 0:return TOP->u_col__DOT__bank__BRA__0__KET____DOT__mem[a];
      case 1:return TOP->u_col__DOT__bank__BRA__1__KET____DOT__mem[a];
      case 2:return TOP->u_col__DOT__bank__BRA__2__KET____DOT__mem[a];
      case 3:return TOP->u_col__DOT__bank__BRA__3__KET____DOT__mem[a];
      case 4:return TOP->u_col__DOT__bank__BRA__4__KET____DOT__mem[a];
      case 5:return TOP->u_col__DOT__bank__BRA__5__KET____DOT__mem[a];
      case 6:return TOP->u_col__DOT__bank__BRA__6__KET____DOT__mem[a];
      default:return TOP->u_col__DOT__bank__BRA__7__KET____DOT__mem[a];
    }
}

// ---- refsw-equivalent golden ----
static uint32_t twiddle(uint32_t x,uint32_t y,uint32_t xs,uint32_t ys){
    uint32_t rv=0,sh=0;xs>>=1;ys>>=1;
    while(xs||ys){if(ys){rv|=(y&1)<<sh;y>>=1;ys>>=1;sh++;}if(xs){rv|=(x&1)<<sh;x>>=1;xs>>=1;sh++;}}
    return rv;
}
static uint32_t ARGB565(uint16_t w){return (((w>>0)&0x1F)<<3)|(((w>>5)&0x3F)<<10)|(((w>>11)&0x1F)<<19)|0xFF000000;}
static int u8_256(int v){return v+(v>>7);}
static bool IsTopLeft(float dx,float dy){return (dy==0&&dx>0)||(dy<0);}

// plane setup helper (PlaneStepper, tile-local anchored)
struct Plane{float ddx,ddy,c;};
static Plane psetup(float X1,float Y1,float X2,float Y2,float X3,float Y3,float XB,float YB,
                    float a1,float a2,float a3){
    float Aa=((a3-a1)*(Y2-Y1)-(a2-a1)*(Y3-Y1));
    float Ba=((X3-X1)*(a2-a1)-(X2-X1)*(a3-a1));
    float C =((X2-X1)*(Y3-Y1)-(X3-X1)*(Y2-Y1));
    Plane p;p.ddx=-Aa/C;p.ddy=-Ba/C;p.c=a1-p.ddx*(X1-XB)-p.ddy*(Y1-YB);return p;
}

int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    dut=new Vtile_engine_top;
    dut->reset_req=1; dut->cold_req=0; dut->cmd_valid=0;
    for(int i=0;i<6;i++)tick(); dut->reset_req=0;
    int g=0; while(dut->core_reset){tick();if(++g>100)return 1;} tick();

    // ---- scene: one big triangle covering the whole tile, flat depth ----
    // ISP vertices (tile-local, base 0,0)
    float IX1=0,IY1=0,IZ1=1, IX2=40,IY2=0,IZ2=1, IX3=0,IY3=40,IZ3=1;
    // TSP vertices (same geometry; own regs)
    float X1=0,Y1=0,Z1=1, X2=40,Y2=0,Z2=1, X3=0,Y3=40,Z3=1, XB=0,YB=0;
    // UV across the triangle; base colour gradient; no offset
    float U1=0,V1=0, U2=1,V2=0, U3=0,V3=1;
    unsigned C1=0x00204060, C2=0x00c0a080, C3=0x00ffffff; // base (A ignored via UseAlpha=0)
    unsigned O1=0,O2=0,O3=0;
    unsigned TAG=0x001111;
    // isp word: DepthMode=7(always)[31:29], Texture[25]=1, Offset[24]=0, Gouraud[23]=1
    unsigned ISPW=(7u<<29)|(1u<<25)|(1u<<23);
    // tsp word: TexU=5,TexV=5 (256), FilterMode=0 (point), UseAlpha=0, ShadInstr=0 (replace)
    unsigned TSPW=(5<<0)|(5<<3)|(0<<13)|(0<<6);
    unsigned TA=0x100;
    unsigned TCW=(TA)|(1u<<27); // 565 twiddled, no VQ

    // ---- preload texture VRAM: 256x256 565 twiddled, colour = f(u,v) ----
    // texel(u,v) 565 = (u&0x1F)<<11 | (v&0x3F)<<5 | ((u^v)&0x1F)
    auto put16=[&](uint32_t wi,int lane,uint16_t val){
        uint64_t w=VRAM[wi]; w&=~((uint64_t)0xFFFF<<(16*lane)); w|=((uint64_t)val)<<(16*lane); VRAM[wi]=w; };
    for(int i=0;i<(1<<20);i++) VRAM[i]=0;
    for(int tv=0;tv<256;tv++)for(int tu=0;tu<256;tu++){
        uint16_t texel=((tu&0x1F)<<11)|((tv&0x3F)<<5)|((tu^tv)&0x1F);
        uint32_t off=twiddle(tu,tv,256,256);
        uint32_t byte=(TA<<3)+off*2; uint32_t wi=byte>>3; int lane=(byte>>1)&3;
        put16(wi,lane,texel);
    }

    // ---- registers ----
    // ISP verts: REG_X1..Z3 = 8..16, X/Y base 17/18, ISP_WORD 19, ISP_TAG 20
    submit(8,f2b(IX1));submit(9,f2b(IY1));submit(10,f2b(IZ1));
    submit(11,f2b(IX2));submit(12,f2b(IY2));submit(13,f2b(IZ2));
    submit(14,f2b(IX3));submit(15,f2b(IY3));submit(16,f2b(IZ3));
    submit(17,f2b(XB));submit(18,f2b(YB));
    submit(19,ISPW); submit(20,TAG);
    // TSP verts: 24..32 XYZ, 33..38 UV, 39..41 col, 42..44 ofs, 45 isp_tsp,46 tsp,47 tcw
    submit(24,f2b(X1));submit(25,f2b(Y1));submit(26,f2b(Z1));
    submit(27,f2b(X2));submit(28,f2b(Y2));submit(29,f2b(Z2));
    submit(30,f2b(X3));submit(31,f2b(Y3));submit(32,f2b(Z3));
    submit(33,f2b(U1));submit(34,f2b(V1));submit(35,f2b(U2));submit(36,f2b(V2));submit(37,f2b(U3));submit(38,f2b(V3));
    submit(39,C1);submit(40,C2);submit(41,C3);
    submit(42,O1);submit(43,O2);submit(44,O3);
    submit(45,ISPW); submit(46,TSPW); submit(47,TCW);
    submit(48,TAG);            // REG_TSP_SHADE_TAG
    submit(49,0);              // REG_TEXT_CTRL

    // clear depth (bg depth 0 so 'always' writes), then setup/raster/setup/shade
    submit(0,f2b(0.0f)); submit(1,0); submit(64,0);    // TILE_CLEAR
    submit(65,0);   // ISP_SETUP
    submit(66,0);   // ISP_RASTERIZE
    submit(67,0);   // TSP_SETUP
    submit(68,0);   // TSP_SHADE

    // ---- golden ----
    float ta=((IX1-IX3)*(IY2-IY3)-(IY1-IY3)*(IX2-IX3)); int sgn=ta>0?-1:1;
    float DX12=sgn*(IX1-IX2),DX23=sgn*(IX2-IX3),DX31=sgn*(IX3-IX1);
    float DY12=sgn*(IY1-IY2),DY23=sgn*(IY2-IY3),DY31=sgn*(IY3-IY1);
    float CC1=DY12*(IX1-XB)-DX12*(IY1-YB)+(IsTopLeft(DX12,DY12)?0:-1);
    float CC2=DY23*(IX2-XB)-DX23*(IY2-YB)+(IsTopLeft(DX23,DY23)?0:-1);
    float CC3=DY31*(IX3-XB)-DX31*(IY3-YB)+(IsTopLeft(DX31,DY31)?0:-1);
    // TSP planes: U,V, base col r/g/b/a, W = 1/invW (invW plane == z=1 here -> W=1)
    Plane pU=psetup(X1,Y1,X2,Y2,X3,Y3,XB,YB,U1*Z1,U2*Z2,U3*Z3);
    Plane pV=psetup(X1,Y1,X2,Y2,X3,Y3,XB,YB,V1*Z1,V2*Z2,V3*Z3);
    auto cch=[&](unsigned c,int i){return (float)((c>>(8*i))&0xFF);};
    Plane pC[4],pO[4];
    for(int i=0;i<4;i++) pC[i]=psetup(X1,Y1,X2,Y2,X3,Y3,XB,YB,cch(C1,i)*Z1,cch(C2,i)*Z2,cch(C3,i)*Z3);

    // nearest-sample texel for (tu,tv)
    auto texel_at=[&](int tu,int tv)->uint32_t{
        tu&=255; tv&=255;
        uint16_t t=((tu&0x1F)<<11)|((tv&0x3F)<<5)|((tu^tv)&0x1F);
        return ARGB565(t);
    };
    int mism=0,covered=0;
    for(int y=0;y<32;y++)for(int x=0;x<32;x++){
        float Xhs12=CC1+DX12*y-DY12*x, Xhs23=CC2+DX23*y-DY23*x, Xhs31=CC3+DX31*y-DY31*x;
        bool inside = Xhs12>=0&&Xhs23>=0&&Xhs31>=0;
        if(!inside) continue;
        covered++;
        uint32_t hw=col_read(x,y);
        float W=1.0f;                                  // z=1 everywhere -> W=1
        float u=(pU.ddx*x+pU.ddy*y+pU.c)*W, v=(pV.ddx*x+pV.ddy*y+pV.c)*W;
        int tu=(int)(u*256*256)>>8, tv=(int)(v*256*256)>>8;
        // Reduced-precision (16-bit-mantissa) interp can land the nearest texel
        // one off at texel boundaries; accept the 3x3 texel neighbourhood.
        bool ok=false;
        for(int du=-1;du<=1&&!ok;du++)for(int dv=-1;dv<=1&&!ok;dv++)
            if(hw==texel_at(tu+du,tv+dv)) ok=true;
        if(!ok){ mism++; if(mism<=8) printf("  x%d y%d hw=%08x exp~=%08x (tu%d tv%d)\n",x,y,hw,texel_at(tu,tv),tu,tv); }
    }
    printf("covered=%d  mismatches=%d\n",covered,mism);
    bool pass = covered>0 && mism==0;
    printf(pass?"TSP PIPE OK\n":"TSP PIPE FAIL\n");
    return pass?0:1;
}
