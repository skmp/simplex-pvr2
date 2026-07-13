// tsp_setup_stream (II=2 rewrite) vs tsp_setup_min on random triangles.
//
// The two are NOT bit-exact by design (stream: no t15 delta truncation, fused 3-way c,
// clean adds), so both are compared against a double-precision reference of the same
// math; the stream must not be meaningfully worse than min. Also checks:
//   * exactly the enabled plane set is emitted, each idx once, both units
//   * per-triangle cycle counts (reports averages; sanity-bounds the stream)
#include "Vtsp_setup_stream_tb_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

static Vtsp_setup_stream_tb_top* dut;

static float    f32(uint32_t b){ float f; memcpy(&f,&b,4); return f; }
static uint32_t b32(float f){ uint32_t b; memcpy(&b,&f,4); return b; }

static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }

static float frnd(float lo, float hi) { return lo + (hi-lo) * (rand() / (float)RAND_MAX); }

struct Plane { bool got; double ddx, ddy, c; };

// run one unit (sel: 0=min, 1=stream) on the already-applied inputs; returns cycles
static int run_unit(int sel, Plane out[10]) {
    for (int i=0;i<10;i++) out[i].got = false;
    if (sel) dut->start_str = 1; else dut->start_min = 1;
    tick();
    dut->start_str = 0; dut->start_min = 0;
    for (int cyc = 1; cyc < 1200; cyc++) {
        bool pv   = sel ? dut->pv_str   : dut->pv_min;
        bool done = sel ? dut->done_str : dut->done_min;
        if (pv) {
            int idx = sel ? dut->pidx_str : dut->pidx_min;
            if (idx > 9 || out[idx].got) { printf("BAD plane idx=%d (dup or range)\n", idx); exit(1); }
            out[idx].got = true;
            out[idx].ddx = f32(sel ? dut->ddx_str : dut->ddx_min);
            out[idx].ddy = f32(sel ? dut->ddy_str : dut->ddy_min);
            out[idx].c   = f32(sel ? dut->c_str   : dut->c_min);
        }
        if (done) return cyc;
        tick();
    }
    printf("TIMEOUT (unit %d)\n", sel); exit(1);
}

int main(int argc, char** argv){
    Verilated::commandArgs(argc, argv);
    dut = new Vtsp_setup_stream_tb_top;
    srand(0x75E7);

    dut->reset = 1; dut->start_min = 0; dut->start_str = 0;
    for (int i=0;i<4;i++) tick();
    dut->reset = 0;

    long NT = 20000;
    double worst_min = 0, worst_str = 0;
    long   bad = 0, tot_planes = 0;
    long   cyc_min_sum = 0, cyc_str_sum = 0;
    int    cyc_str_max = 0;

    for (long t = 0; t < NT; t++) {
        // ---- random triangle (screen-ish coords, non-degenerate) ----
        float xs[3], ys[3], zs[3];
        double area;
        do {
            for (int i=0;i<3;i++) { xs[i]=frnd(0,640); ys[i]=frnd(0,480); }
            area = ((double)xs[1]-xs[0])*((double)ys[2]-ys[0])
                 - ((double)xs[2]-xs[0])*((double)ys[1]-ys[0]);
        } while (fabs(area) < 4.0);
        for (int i=0;i<3;i++) zs[i] = frnd(0.01f, 50.f);
        float xb = 32.f * (int)(fminf(fminf(xs[0],xs[1]),xs[2]) / 32.f);
        float yb = 32.f * (int)(fminf(fminf(ys[0],ys[1]),ys[2]) / 32.f);
        float us[3], vs[3];
        for (int i=0;i<3;i++) { us[i]=frnd(0,8); vs[i]=frnd(0,8); }
        uint32_t col[3] = { (uint32_t)rand()<<16 ^ (uint32_t)rand(),
                            (uint32_t)rand()<<16 ^ (uint32_t)rand(),
                            (uint32_t)rand()<<16 ^ (uint32_t)rand() };
        uint32_t ofs[3] = { (uint32_t)rand()<<16 ^ (uint32_t)rand(),
                            (uint32_t)rand()<<16 ^ (uint32_t)rand(),
                            (uint32_t)rand()<<16 ^ (uint32_t)rand() };
        int gour = rand()&1, tex = rand()&1, ofl = rand()&1;

        dut->x1=b32(xs[0]); dut->y1=b32(ys[0]); dut->z1=b32(zs[0]);
        dut->x2=b32(xs[1]); dut->y2=b32(ys[1]); dut->z2=b32(zs[1]);
        dut->x3=b32(xs[2]); dut->y3=b32(ys[2]); dut->z3=b32(zs[2]);
        dut->xbase=b32(xb); dut->ybase=b32(yb);
        dut->u1=b32(us[0]); dut->v1=b32(vs[0]);
        dut->u2=b32(us[1]); dut->v2=b32(vs[1]);
        dut->u3=b32(us[2]); dut->v3=b32(vs[2]);
        dut->col1=col[0]; dut->col2=col[1]; dut->col3=col[2];
        dut->ofs1=ofs[0]; dut->ofs2=ofs[1]; dut->ofs3=ofs[2];
        dut->gouraud=gour; dut->texture=tex; dut->offset=ofl;

        Plane pm[10], ps[10];
        cyc_min_sum += run_unit(0, pm);
        int cs = run_unit(1, ps);
        cyc_str_sum += cs; if (cs > cyc_str_max) cyc_str_max = cs;

        // ---- double-precision reference + comparison per enabled plane ----
        double X21=(double)xs[1]-xs[0], X31=(double)xs[2]-xs[0];
        double Y21=(double)ys[1]-ys[0], Y31=(double)ys[2]-ys[0];
        for (int idx=0; idx<10; idx++) {
            bool en = (idx<=1) ? tex : (idx<=5) ? true : ofl;
            if (pm[idx].got != en || ps[idx].got != en) {
                printf("[t=%ld] plane %d presence: en=%d min=%d str=%d\n",
                       t, idx, en, pm[idx].got, ps[idx].got); return 1;
            }
            if (!en) continue;
            tot_planes++;
            // per-vertex attribute
            double a[3];
            for (int i=0;i<3;i++) {
                if (idx<=1)      a[i] = (idx==0) ? us[i] : vs[i];
                else {
                    int ch = (idx-2)&3;
                    uint32_t w = (idx<=5) ? (gour?col[i]:col[2]) : (gour?ofs[i]:ofs[2]);
                    a[i] = (double)((w >> (8*ch)) & 0xFF);
                }
            }
            double p1=zs[0]*a[0], p2=zs[1]*a[1], p3=zs[2]*a[2];
            double da2=p2-p1, da3=p3-p1;
            double Aa = da3*Y21 - da2*Y31;
            double Ba = X31*da2 - X21*da3;
            double ddx = -Aa/area, ddy = -Ba/area;
            double c   = p1 - ddx*((double)xs[0]-xb) - ddy*((double)ys[0]-yb);
            // hybrid abs+rel error, worst over the 3 outputs
            double scale = fmax(fabs(ddx)+fabs(ddy)+fabs(c), 1e-3);
            double em=0, es=0;
            em = fmax(em, fabs(pm[idx].ddx-ddx)/fmax(fabs(ddx),scale*1e-2));
            em = fmax(em, fabs(pm[idx].ddy-ddy)/fmax(fabs(ddy),scale*1e-2));
            em = fmax(em, fabs(pm[idx].c  -c  )/fmax(fabs(c),  scale*1e-2));
            es = fmax(es, fabs(ps[idx].ddx-ddx)/fmax(fabs(ddx),scale*1e-2));
            es = fmax(es, fabs(ps[idx].ddy-ddy)/fmax(fabs(ddy),scale*1e-2));
            es = fmax(es, fabs(ps[idx].c  -c  )/fmax(fabs(c),  scale*1e-2));
            if (em > worst_min) worst_min = em;
            if (es > worst_str) worst_str = es;
            // stream must not be meaningfully worse than min on the same plane
            if (es > em*4.0 + 1e-3) {
                bad++;
                if (bad <= 10)
                    printf("[t=%ld idx=%d] str worse: es=%.3g em=%.3g "
                           "(ref ddx=%.6g ddy=%.6g c=%.6g | str %.6g %.6g %.6g | min %.6g %.6g %.6g)\n",
                           t, idx, es, em, ddx, ddy, c,
                           ps[idx].ddx, ps[idx].ddy, ps[idx].c,
                           pm[idx].ddx, pm[idx].ddy, pm[idx].c);
            }
        }
    }

    printf("tsp_setup_stream vs min: %ld triangles, %ld planes\n", NT, tot_planes);
    printf("  worst rel-err vs double ref: min=%.4g stream=%.4g\n", worst_min, worst_str);
    printf("  avg cycles/tri: min=%.1f stream=%.1f (stream max %d)\n",
           (double)cyc_min_sum/NT, (double)cyc_str_sum/NT, cyc_str_max);
    printf("  planes where stream >4x worse than min: %ld\n", bad);
    dut->final(); delete dut;
    return bad ? 1 : 0;
}
