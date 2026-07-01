// frontend_tb - drive the render front-end (reg_file + VRAM + RA/OL/tristrip +
// faux ISP) from real PVR dumps and log the triangles.
//
// Loads:
//   dumps/pvr_regs_menu2.bin  (4 KB = 1024 x 32-bit, the PVR reg region)
//   dumps/vram_menu2.bin      (8 MB = 1M x 64-bit, physical VRAM)
// Resets for 10000 cycles, pulses `go`, runs until `done` (or a cycle cap).
#include "Vfrontend_tb_top.h"
#include "Vfrontend_tb_top___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>

static Vfrontend_tb_top* dut;
#define VRAM dut->rootp->frontend_tb_top__DOT__vram
static void tick(){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); }

// load a file; returns malloc'd buffer, sets *out_sz to the actual byte size.
static uint8_t* load(const char* path, long* out_sz){
    FILE* f=fopen(path,"rb");
    if(!f){ printf("cannot open %s\n",path); exit(1); }
    fseek(f,0,SEEK_END); long sz=ftell(f); fseek(f,0,SEEK_SET);
    uint8_t* buf=(uint8_t*)malloc(sz);
    if(fread(buf,1,sz,f)!=(size_t)sz){ printf("short read %s\n",path); exit(1); }
    fclose(f); if(out_sz)*out_sz=sz; return buf;
}

int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    dut=new Vfrontend_tb_top;
    dut->clk=0; dut->reset=1; dut->go=0; dut->wr_en=0; dut->wr_addr=0; dut->wr_data=0;

    // ---- load VRAM ----
    // The dump is the PVR "32-bit VIEW" (linear byte addressing), but the RTL
    // models PHYSICAL 64-bit interleaved VRAM (the data_cache256 de-interleaves
    // per pvr_map32 on read). So we must RE-INTERLEAVE the dump into physical
    // layout here (inverse pvr_map32): 32-bit-view word q -> bank=q[20],
    // wofs=q[19:0]; physical 64-bit word wofs gets this word in its low half
    // (bank0) or high half (bank1). Then the cache's de-interleave recovers the
    // linear view the region/object/param addresses expect.
    long vsz; uint8_t* v = load("dumps/vram_menu2.bin", &vsz);
    if(vsz != 8*1024*1024) printf("warning: vram is %ld bytes (expected 8 MB)\n", vsz);
    for(uint32_t w=0; w<1048576; w++) VRAM[w]=0;
    uint32_t nview = vsz/4;                    // number of 32-bit view words
    for(uint32_t q=0; q<nview; q++){
        uint32_t word = v[q*4] | (v[q*4+1]<<8) | (v[q*4+2]<<16) | (v[q*4+3]<<24);
        uint32_t bank = (q>>20)&1;
        uint32_t wofs = q & 0xFFFFF;
        uint64_t cur = VRAM[wofs];
        cur &= ~((uint64_t)0xFFFFFFFFu << (32*bank));
        cur |=  ((uint64_t)word) << (32*bank);
        VRAM[wofs] = cur;
    }
    free(v);

    // ---- reset for 10000 cycles ----
    for(int i=0;i<10000;i++) tick();
    dut->reset=0;
    tick();

    // ---- load PVR regs. Dumps are 4 KB or 32 KB; only the low 8 KB (0x2000,
    // = 2048 words) is valid and covers all named regs + FOG/PAL tables. ----
    long rsz; uint8_t* rg = load("dumps/pvr_regs_menu2.bin", &rsz);
    uint32_t nwords = (rsz < 0x2000 ? rsz : 0x2000) / 4;   // cap at 8 KB
    for(uint32_t i=0;i<nwords;i++){
        uint32_t val = rg[i*4] | (rg[i*4+1]<<8) | (rg[i*4+2]<<16) | (rg[i*4+3]<<24);
        dut->wr_addr = i*4;   // byte offset into the PVR reg region
        dut->wr_data = val;
        dut->wr_en   = 1;
        tick();
    }
    dut->wr_en = 0;
    tick();

    // print a couple of key regs for sanity
    printf("REGION_BASE=%08x PARAM_BASE=%08x FPU_PARAM_CFG=%08x\n",
        (uint32_t)(rg[0x2C]|(rg[0x2D]<<8)|(rg[0x2E]<<16)|(rg[0x2F]<<24)),
        (uint32_t)(rg[0x20]|(rg[0x21]<<8)|(rg[0x22]<<16)|(rg[0x23]<<24)),
        (uint32_t)(rg[0x7C]|(rg[0x7D]<<8)|(rg[0x7E]<<16)|(rg[0x7F]<<24)));
    free(rg);

    // ---- go ----
    dut->go=1; tick(); dut->go=0;
    long cyc=0;
    while(!dut->done){
        tick();
        if(++cyc > 20000000){ printf("TIMEOUT after %ld cycles\n",cyc); break; }
    }
    printf("finished in %ld cycles\n", cyc);
    return 0;
}
