// tsp_shade_v2_pp_replay_tb - replay a recorded tsp pixel-input stream (tsp_test_traces.txt,
// from the known-good peel_core's +tsppipedump) through tsp_shade_v2_pp over the scene's
// real VRAM + palette, and compare each pixel's produced ARGB/TSP against the EXPECTED
// output recorded on the same line. Any mismatch is a real shade-path divergence between
// the v2 rewrite and the known-good reference, reported with the exact failing pixel.
//
// Usage:
//   ./tb <traces.txt> <vram.bin> <regs.bin>
// Defaults: tsp_test_traces.txt  dumps/vram_menu2.bin  dumps/pvr_regs_menu2.bin
//
// The trace line format (produced by +tsppipedump) is:
//   seq id px py invw tsp tcw text_ctrl ptex pofs  ddx0..9 ddy0..9 c0..9  | out_id out_argb out_tsp
//
#include "Vtsp_shade_v2_pp_replay_tb_top.h"
#include "Vtsp_shade_v2_pp_replay_tb_top___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>

static Vtsp_shade_v2_pp_replay_tb_top* dut;
#define VRAM dut->rootp->tsp_shade_v2_pp_replay_tb_top__DOT__vram
static void tick(){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); }

struct In {
    uint32_t id, px, py, invw, tsp, tcw, tc, ptx, pof;
    uint32_t ddx[10], ddy[10], c[10];
    // expected output recorded alongside the input
    uint32_t exp_id, exp_argb, exp_tsp;
};

static void apply(const In& p){
    dut->px=p.px; dut->py=p.py; dut->invw_in=p.invw;
    dut->tsp=p.tsp; dut->tcw=p.tcw; dut->text_ctrl=p.tc;
    dut->pp_texture=p.ptx; dut->pp_offset=p.pof;
    dut->ddx0=p.ddx[0];dut->ddx1=p.ddx[1];dut->ddx2=p.ddx[2];dut->ddx3=p.ddx[3];dut->ddx4=p.ddx[4];
    dut->ddx5=p.ddx[5];dut->ddx6=p.ddx[6];dut->ddx7=p.ddx[7];dut->ddx8=p.ddx[8];dut->ddx9=p.ddx[9];
    dut->ddy0=p.ddy[0];dut->ddy1=p.ddy[1];dut->ddy2=p.ddy[2];dut->ddy3=p.ddy[3];dut->ddy4=p.ddy[4];
    dut->ddy5=p.ddy[5];dut->ddy6=p.ddy[6];dut->ddy7=p.ddy[7];dut->ddy8=p.ddy[8];dut->ddy9=p.ddy[9];
    dut->c0=p.c[0];dut->c1=p.c[1];dut->c2=p.c[2];dut->c3=p.c[3];dut->c4=p.c[4];
    dut->c5=p.c[5];dut->c6=p.c[6];dut->c7=p.c[7];dut->c8=p.c[8];dut->c9=p.c[9];
}

int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    const char* inpath   = (argc>1)? argv[1] : "tsp_test_traces.txt";
    const char* vrampath = (argc>2)? argv[2] : "dumps/vram_menu2.bin";
    const char* regspath = (argc>3)? argv[3] : "dumps/pvr_regs_menu2.bin";
    // per-channel ARGB tolerance (default 0 = bit-exact). The v2 tex_filter uses a raw
    // >>8 bilinear weight while the known-good reference uses to_u8_256/65536, so textured
    // pixels can differ by a few LSB per channel WITHOUT being a real bug. TOL=<N> counts
    // any pixel whose every channel is within N as a pass; it still reports exact hits too.
    int tol = 0;
    { const char* t = getenv("TOL"); if(t) tol = atoi(t); }

    // ---- read the trace (inputs + expected outputs) ----
    FILE* f=fopen(inpath,"r");
    if(!f){ printf("cannot open %s\n",inpath); return 2; }
    std::vector<In> pix;
    char line[4096];
    while(fgets(line,sizeof(line),f)){
        if(line[0]=='#') continue;
        // split at the ' | ' separator between inputs and expected outputs
        char* bar = strstr(line," | ");
        if(!bar) continue;
        *bar = '\0';
        char* outs = bar+3;
        In p; unsigned seq;
        int n=sscanf(line,"%u %u %u %u %x %x %x %x %u %u"
                          " %x %x %x %x %x %x %x %x %x %x"
                          " %x %x %x %x %x %x %x %x %x %x"
                          " %x %x %x %x %x %x %x %x %x %x",
            &seq,&p.id,&p.px,&p.py,&p.invw,&p.tsp,&p.tcw,&p.tc,&p.ptx,&p.pof,
            &p.ddx[0],&p.ddx[1],&p.ddx[2],&p.ddx[3],&p.ddx[4],&p.ddx[5],&p.ddx[6],&p.ddx[7],&p.ddx[8],&p.ddx[9],
            &p.ddy[0],&p.ddy[1],&p.ddy[2],&p.ddy[3],&p.ddy[4],&p.ddy[5],&p.ddy[6],&p.ddy[7],&p.ddy[8],&p.ddy[9],
            &p.c[0],&p.c[1],&p.c[2],&p.c[3],&p.c[4],&p.c[5],&p.c[6],&p.c[7],&p.c[8],&p.c[9]);
        int m=sscanf(outs,"%u %x %x",&p.exp_id,&p.exp_argb,&p.exp_tsp);
        if(n==40 && m==3) pix.push_back(p);
    }
    fclose(f);
    printf("replay: %zu pixels from %s\n", pix.size(), inpath);
    if(pix.empty()){ printf("no pixels parsed - check the trace format\n"); return 2; }

    dut=new Vtsp_shade_v2_pp_replay_tb_top;
    dut->clk=0; dut->reset=1; dut->pp_in_valid=0; dut->pp_in_id=0; dut->wr_en=0;

    // ---- load real VRAM (32-bit view -> 64-bit physical, inverse pvr_map32) ----
    FILE* vf=fopen(vrampath,"rb");
    if(!vf){ printf("cannot open %s\n",vrampath); return 2; }
    fseek(vf,0,SEEK_END); long vsz=ftell(vf); fseek(vf,0,SEEK_SET);
    std::vector<uint8_t> v(vsz); size_t rd=fread(v.data(),1,vsz,vf); fclose(vf); (void)rd;
    for(uint32_t w=0; w<1048576; w++) VRAM[w]=0;
    uint32_t nview=vsz/4;
    for(uint32_t q=0;q<nview;q++){
        uint32_t word=v[q*4]|(v[q*4+1]<<8)|(v[q*4+2]<<16)|(v[q*4+3]<<24);
        uint32_t bank=(q>>20)&1, wofs=q&0xFFFFF;
        uint64_t cur=VRAM[wofs];
        cur &= ~((uint64_t)0xFFFFFFFFu << (32*bank));
        cur |=  ((uint64_t)word) << (32*bank);
        VRAM[wofs]=cur;
    }
    printf("loaded VRAM %ld bytes from %s\n", vsz, vrampath);

    // hold reset a while so reg_file / caches settle
    for(int i=0;i<64;i++) tick();
    dut->reset=0; tick();

    // ---- load PVR regs (palette + pal_fmt) via the reg_file write path ----
    FILE* rf=fopen(regspath,"rb");
    if(!rf){ printf("cannot open %s\n",regspath); return 2; }
    fseek(rf,0,SEEK_END); long rsz=ftell(rf); fseek(rf,0,SEEK_SET);
    std::vector<uint8_t> rg(rsz); size_t rd2=fread(rg.data(),1,rsz,rf); fclose(rf); (void)rd2;
    // reg_file accepts a 13-bit byte offset; palette lives at 0x1000..0x1FFC. Write the
    // whole low region (< 0x2000) so scalars (pal_ram_ctrl) AND the palette load.
    uint32_t nwords = (rsz < 0x2000 ? (uint32_t)rsz : 0x2000) / 4;
    for(uint32_t i=0;i<nwords;i++){
        uint32_t val = rg[i*4] | (rg[i*4+1]<<8) | (rg[i*4+2]<<16) | (rg[i*4+3]<<24);
        dut->wr_addr = i*4;
        dut->wr_data = val;
        dut->wr_en   = 1;
        tick();
    }
    dut->wr_en = 0; tick();
    printf("loaded PVR regs %ld bytes from %s\n", rsz, regspath);

    // ---- stream the recorded inputs through v2, in order, honoring stall ----
    // The pipeline is in-order & lossless: the k-th accepted input yields the k-th
    // out_valid. We keep an in-order queue of accepted-pixel indices and pop on each
    // out_valid to compare against that pixel's expected output.
    std::vector<uint32_t> pending;   // indices of accepted-but-not-yet-emitted pixels
    size_t ii=0, done=0;             // ii = next input to present ; done = results checked
    size_t qhead=0;
    int total=0, fails=0, exact_bad=0, shown=0, maxdiff=0;
    long guard=0;
    dut->pp_in_valid=0;

    while(done < pix.size()){
        dut->eval();
        bool present = (ii<pix.size()) && !dut->pp_stall;
        if(present){
            apply(pix[ii]);
            dut->pp_in_id = (uint16_t)(pix[ii].id & 0x7FF);   // IDW=11
            dut->pp_in_valid = 1;
        } else {
            dut->pp_in_valid = 0;
        }
        tick();
        if(present){ pending.push_back((uint32_t)ii); ii++; }

        if(dut->pp_out_valid){
            if(qhead >= pending.size()){
                printf("FATAL: out_valid with no pending input (spurious result) @done=%zu\n", done);
                fails++; done++;   // avoid deadlock
            } else {
                uint32_t idx = pending[qhead++];
                const In& p = pix[idx];
                uint32_t got_argb = dut->pp_out_argb;
                uint32_t got_tsp  = dut->pp_out_tsp;
                uint32_t got_id   = dut->pp_out_id;
                total++;
                // per-channel absolute diff of the ARGB
                int da = abs((int)((got_argb>>24)&0xFF) - (int)((p.exp_argb>>24)&0xFF));
                int dr = abs((int)((got_argb>>16)&0xFF) - (int)((p.exp_argb>>16)&0xFF));
                int dg = abs((int)((got_argb>> 8)&0xFF) - (int)((p.exp_argb>> 8)&0xFF));
                int db = abs((int)((got_argb    )&0xFF) - (int)((p.exp_argb    )&0xFF));
                int chmax = da; if(dr>chmax)chmax=dr; if(dg>chmax)chmax=dg; if(db>chmax)chmax=db;
                if(chmax>maxdiff) maxdiff=chmax;
                bool argb_exact = (got_argb == p.exp_argb);
                bool argb_ok = (chmax <= tol);          // within tolerance
                bool tsp_ok  = (got_tsp  == p.exp_tsp);
                bool id_ok   = (got_id   == (p.exp_id & 0x7FF));
                if(!argb_exact) exact_bad++;
                if(!(argb_ok && tsp_ok && id_ok)){
                    fails++;
                    if(shown<40){
                        printf("MISMATCH idx=%u id=%u px=%u py=%u ptex=%u tcw=%08x tsp=%08x\n",
                               idx, p.id, p.px, p.py, p.ptx, p.tcw, p.tsp);
                        printf("    argb got=%08x exp=%08x %s | out_tsp got=%08x exp=%08x %s | out_id got=%u exp=%u %s\n",
                               got_argb, p.exp_argb, argb_ok?"ok":"BAD",
                               got_tsp,  p.exp_tsp,  tsp_ok ?"ok":"BAD",
                               got_id,   p.exp_id & 0x7FF, id_ok?"ok":"BAD");
                        shown++;
                    }
                }
                done++;
            }
        }

        if(++guard > 200000000){ printf("TIMEOUT: presented=%zu checked=%zu\n", ii, done); break; }
    }

    printf("\n==== tsp_shade_v2_pp replay vs recorded expected ====\n");
    printf("checked %d pixels | tolerance=%d (per channel)\n", total, tol);
    printf("  bit-exact ARGB mismatches : %d\n", exact_bad);
    printf("  out-of-tolerance failures : %d  (also counts any out_tsp/out_id mismatch)\n", fails);
    printf("  worst per-channel ARGB diff observed : %d LSB\n", maxdiff);
    dut->final();
    delete dut;
    return fails ? 1 : 0;
}
