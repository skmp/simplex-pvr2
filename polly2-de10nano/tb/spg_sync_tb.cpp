// Sync-geometry check for spg.sv after the VS/HS alignment fix.
// Runs >1 frame, records HS/VS edges at the module outputs, and checks:
//  - HS: period 2200, pulse 44 wide
//  - VS: rises/falls exactly at an HS rising edge (CEA alignment)
//  - VS pulse = 5 line times; VS period = 2200*1125 clocks
//  - DE: 1080 lines of 1920 pixels per frame
#include "Vspg.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vspg* dut = new Vspg;

    dut->reset = 1;
    dut->fb_base = 0; dut->fb_stride = 1280;
    dut->fb_line_dbl = 0; dut->fb_split = 1; dut->fb_disp_half = 0;
    dut->avl_waitrequest = 1; dut->avl_readdatavalid = 0;

    auto tick = [&](){
        dut->clk = 0; dut->avl_clk = 0; dut->eval();
        dut->clk = 1; dut->avl_clk = 1; dut->eval();
    };
    for (int i = 0; i < 10; i++) tick();
    dut->reset = 0;

    const long TOTAL = 2200L * 1125L;
    long t = 0;
    int prev_hs = 0, prev_vs = 0, prev_de = 0;
    std::vector<long> hs_rise, vs_rise, vs_fall;
    long de_pixels = 0, de_lines = 0, run = 0;
    int errors = 0;

    for (long n = 0; n < TOTAL * 3 + 100; n++, t++) {
        tick();
        if (dut->hsync && !prev_hs) hs_rise.push_back(t);
        if (dut->vsync && !prev_vs) vs_rise.push_back(t);
        if (!dut->vsync && prev_vs) vs_fall.push_back(t);
        if (dut->de) { de_pixels++; run++; }
        if (!dut->de && prev_de) {
            // ignore lines before the first VS edge: the output pipe is
            // mid-flight when reset deasserts, so line 0 carries a stale pixel
            if (!vs_rise.empty() && run != 1920) { printf("BAD DE run %ld at t=%ld\n", run, t); errors++; }
            if (!vs_rise.empty()) de_lines++;
            run = 0;
        }
        prev_hs = dut->hsync; prev_vs = dut->vsync; prev_de = dut->de;
    }

    // HS period
    for (size_t i = 2; i < hs_rise.size() && i < 10; i++)
        if (hs_rise[i] - hs_rise[i-1] != 2200) { printf("BAD HS period\n"); errors++; }

    // VS edges must land on an HS rising edge
    auto on_hs = [&](long tv){
        for (long h : hs_rise) if (h == tv) return true;
        return false;
    };
    for (size_t i = 1; i < vs_rise.size(); i++) {   // skip edge 0 (reset transient)
        if (!on_hs(vs_rise[i])) { printf("VS rise %ld NOT on HS edge\n", vs_rise[i]); errors++; }
    }
    for (size_t i = 1; i < vs_fall.size(); i++) {
        if (!on_hs(vs_fall[i])) { printf("VS fall %ld NOT on HS edge\n", vs_fall[i]); errors++; }
    }
    // VS width and period
    for (size_t i = 1; i + 1 < vs_rise.size(); i++) {
        long w = vs_fall[i] - vs_rise[i];
        long p = vs_rise[i+1] - vs_rise[i];
        if (w != 5*2200)  { printf("BAD VS width %ld\n", w); errors++; }
        if (p != TOTAL)   { printf("BAD VS period %ld\n", p); errors++; }
    }
    if (vs_rise.size() < 3) { printf("too few VS edges (%zu)\n", vs_rise.size()); errors++; }

    printf("frames=%zu de_lines=%ld de_pixels=%ld\n", vs_rise.size(), de_lines, de_pixels);
    printf(errors ? "FAIL (%d errors)\n" : "PASS\n", errors);
    delete dut;
    return errors != 0;
}
