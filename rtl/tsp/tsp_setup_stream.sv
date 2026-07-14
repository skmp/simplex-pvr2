//
// tsp_setup_stream - STREAMED (II=2) TSP setup on registered-output pipelined FP units.
//
// Same math as tsp_setup_min (refsw IPs3::Setup), same ports: one plane enters the
// stepper every 2 clocks and one (ddx,ddy,c) triple retires every 2 clocks. A 10-plane
// triangle completes in ~45 clocks (start -> done) with FULL timing closure: every
// register-to-register path goes through exactly one *_spp_ro pipeline stage, so there
// are no fat combinational float clouds and no multicycle SDC anywhere.
//
// vs the previous II=4 window version:
//   * all arithmetic on streaming registered-output units (fp_mul16_spp_ro 2clk,
//     fp_mul_c9_spp_ro 2clk, fp_add24_spp_ro 3clk, fp_add3_24_spp_ro 4clk,
//     fp_rcp_fast 3clk) - no combinational unit output is ever consumed in the same
//     cycle it is produced except through the next unit's OWN stage-1 register.
//   * min-magnitude anchor RESTORED (the old stream always anchored on vertex 1):
//     c = p_anch - ddx*XLa - ddy*YTa, anchor = argmin_k max(|XLk|,|YTk|).
//   * NO t15 truncation on the GEO deltas (the old stream truncated the first operand
//     to 15 mantissa bits to bit-match tsp_setup_min's mac16 *1.0 path). Deltas/area/
//     everything use the clean full-precision adders, so outputs differ from
//     tsp_setup_min in low mantissa bits (slightly MORE accurate).
//
// -------------------------------------------------------------------------------
// SCHEDULE. One 6-bit counter `cnt` (1 on the first RUN cycle). Everything below is
// STATIC - a fixed issue offset from either cnt (per-triangle GEO) or the plane's
// inject pulse (per-plane stepper). Unit outputs are registered, valid exactly
// LAT cycles after issue, and are read exactly then.
//
// GEO preamble (shared units, before/under the plane stream). The GEO operands go
// through a REGISTER (g?_r) before the adders - selected at cnt N, issued at cnt N+1 -
// so the wide cnt-mux is OFF the adder S1 critical path (it was reg->mux->S1-align,
// a -3.5..-4ns violator; now reg->S1). Results land one cycle later than the naive
// schedule; only the GEO timeline shifts, the plane stepper is untouched:
//   sel cnt 1..4 : the 10 delta subs on A0/A1/A2 (issue cnt 2..5, results cnt 5..8)
//                sel cnt1: Y31(A0)  Y21(A1)  X31(A2)      [results cnt 5]
//                sel cnt2: X21(A0)  XL1(A1)  YT1(A2)      [results cnt 6]
//                sel cnt3: XL2(A0)  YT2(A1)  XL3(A2)      [results cnt 7]
//                sel cnt4: YT3(A0)                        [result  cnt 8]
//   cnt 6     : area products X21*Y31 (M0), X31*Y21 (M1)  [outs cnt 8]
//   sel cnt 8 : area = M0.y - M1.y on A0 (issue cnt 9)    [out  cnt 12]
//   cnt 9     : min-magnitude anchor select (XL/YT regs all captured by now;
//               anch/aXLr/aYTr valid cnt 10)
//   cnt 12    : rcp in                                     [rcy ready cnt 16]
//
// Plane stepper (plane injected at T = 6 + 2k; all offsets relative to T):
//   +0  prime v1 (MP0) + v2 (MP1)         p_i = z_i * attr_i   (mul16 / mul_c9)
//   +1  prime v3 (MP0)
//   +2  A1: da2 = p2 - p1                 (p1,p2 on the prime outputs NOW)
//   +3  A1: da3 = p3 - p1_r ; p3 -> p3_r
//   +4  pa = anchor-mux(p1_r,p2_r,p3_r) -> pa_sr   (anchor is valid cnt 10 = T+4
//       for the FIRST plane - the extra p3_r hold buys the shifted GEO one cycle)
//   +5  M0: da2*Y31   M1: X31*da2         (da2 on A1.y NOW)
//   +6  M0: da3*Y21   M1: X21*da3
//   +8  A2: Aa = da3*Y21 - da2*Y31        (M0.y now / held)
//   +9  A2: Ba = X31*da2 - X21*da3        (both held)
//   +11 MD: ddx = -Aa * rcy               (Aa on A2.y NOW; rcy ready cnt>=16)
//   +12 MD: ddy = -Ba * rcy
//   +13 MF: f0 = ddx * XLa                (ddx on MD.y NOW)
//   +14 MF: f1 = ddy * YTa
//   +16 A3: c = pa - f0 - f1              (pa from pa_sr; f0 held; f1 on MF.y NOW;
//                                          A3 is the 4-clock unit -> result at +20)
//   +20 EMIT: o_ddx/o_ddy from the MD delay line, o_c = A3.y, idx from idx_sr
//
// The earliest inject T=6 satisfies every hazard: anchor (ready cnt 10) is first used
// at T+4=10; rcy (ready cnt 16) is first used at T+11=17. With II=2 each unit sees each
// offset parity exactly once - no structural conflicts (GEO issues on A0/A1/A2 at
// cnt 2..5 + area at 9, M0/M1 at cnt 6, all before the first stream issues at 8/9
// (A1), 14 (A2), 11 (M0/M1)).
//
// Latency: inject -> plane_valid = 21 clks. n enabled planes: done at cnt 26+2n
// (n=10 -> 46; n=4, flat untextured -> 34).
//
// plane_valid pulses at MOST every 2 clocks (was every 4) - the consumer must accept
// one plane per 2 clocks.
//
module tsp_setup_stream (
    input             clk,
    input             reset,
    input             start,
    output reg        done,

    input             gouraud,
    input             texture,
    input             offset,

    input      [31:0] x1,y1,z1, x2,y2,z2, x3,y3,z3,
    input      [31:0] xbase, ybase,
    input      [31:0] u1,v1, u2,v2, u3,v3,
    input      [31:0] col1,col2,col3,
    input      [31:0] ofs1,ofs2,ofs3,

    output reg        plane_valid,
    output reg [3:0]  plane_idx,
    output reg [31:0] o_ddx, output reg [31:0] o_ddy, output reg [31:0] o_c
);
    function [31:0] fneg(input [31:0] f); fneg = {~f[31], f[30:0]}; endfunction
    function signed [8:0] chan(input [31:0] c, input [1:0] ch);
        case (ch) 0:chan=$signed({1'b0,c[7:0]});   1:chan=$signed({1'b0,c[15:8]});
                  2:chan=$signed({1'b0,c[23:16]}); 3:chan=$signed({1'b0,c[31:24]}); endcase
    endfunction
    // anchor magnitude: max(|XL|,|YT|) reduced to biased exponent (as tsp_setup_min)
    function [7:0] amag(input [31:0] xl, input [31:0] yt);
        amag = (xl[30:23] > yt[30:23]) ? xl[30:23] : yt[30:23]; endfunction

    // ---------------- latched triangle ----------------
    reg [31:0] Xv[0:2], Yv[0:2], Zr[0:2];
    reg [31:0] U[0:2], V[0:2], CO[0:2], OF[0:2];
    reg [31:0] XB, YB;
    reg        g_r, tex_r, ofs_r;
    reg [3:0]  n_planes;

    // ---------------- per-triangle GEO constants ----------------
    reg [31:0] Y21r, Y31r, X21r, X31r;
    reg [31:0] XL1r,YT1r, XL2r,YT2r, XL3r,YT3r;
    reg [31:0] rcy, aXLr, aYTr;
    reg [1:0]  anch;

    // ---------------- control ----------------
    reg        run;
    reg [5:0]  cnt;
    reg [3:0]  nextp, emit_cnt;
    reg        fin;

    // plane scanner (next enabled plane >= nextp)
    function plane_en(input [3:0] i);
        plane_en = (i<=1) ? tex_r : (i<=5) ? 1'b1 : ofs_r; endfunction
    reg [3:0] inj_idx; reg inj_v; integer pp;
    always @(*) begin
        inj_idx = 4'd10; inj_v = 1'b0;
        for (pp = 9; pp >= 0; pp = pp - 1)
            if (pp >= nextp && plane_en(pp[3:0])) begin inj_idx = pp[3:0]; inj_v = 1'b1; end
    end
    wire       inj_uv  = (inj_idx <= 4'd1);
    wire [1:0] inj_chn = inj_idx[1:0] - 2'd2;    // col 2..5 / ofs 6..9 -> chan 0..3
    wire       inject  = run && !cnt[0] && (cnt >= 6'd6) && inj_v;

    // per-vertex attribute operands for the plane being injected (flat -> vertex 3)
    wire [31:0] uv_1 = (inj_idx==4'd0) ? U[0] : V[0];
    wire [31:0] uv_2 = (inj_idx==4'd0) ? U[1] : V[1];
    wire [31:0] uv_3 = (inj_idx==4'd0) ? U[2] : V[2];
    wire [31:0] c_w1 = (inj_idx<=4'd5) ? (g_r?CO[0]:CO[2]) : (g_r?OF[0]:OF[2]);
    wire [31:0] c_w2 = (inj_idx<=4'd5) ? (g_r?CO[1]:CO[2]) : (g_r?OF[1]:OF[2]);
    wire [31:0] c_w3 = (inj_idx<=4'd5) ? CO[2] : OF[2];
    // v3 operands are consumed one cycle AFTER inject (the scanner moves on) - latch
    reg [31:0]       uv3_q;
    reg signed [8:0] k3_q;
    reg              inj_uv_q;

    // ---------------- pulse + carry delay lines (shift every RUN clock) ----------
    reg [20:1] d;                  // d[i] high during T+i for an inject at T
    reg [3:0]  idx_sr [0:19];      // plane_idx to the emit point (T+20)
    reg [31:0] pa_sr  [0:11];      // anchor p (captured T+4) to the C issue point (T+16)
    reg [31:0] dxy_sr [0:6];       // MD.y history: ddx@[6] / ddy@[5] at emit (T+20)
    reg        kd0, kd1;           // prime kind (uv/colour) aligned to prime outputs
    reg [31:0] p1_r, p2_r, p3_r;   // this plane's primes held for the +3/+4 pa cycles
    reg [31:0] h_m0, h_m1, h2_m1, h_mf;   // 1/2-cycle output holds

    // ======================= arithmetic units =======================
    // prime: MP0 = {MU0 || MC0} (v1 then v3), MP1 = {MU1 || MC1} (v2). Both kinds are
    // driven; the registered-output mux picks by the delayed kind bit kd1.
    wire [31:0] mu0_y, mc0_y, mu1_y, mc1_y;
    fp_mul16_spp_ro  MU0 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(inject|d[1]),
        .a(inject ? Zr[0] : Zr[2]), .b(inject ? uv_1 : uv3_q),
        .out_valid(), .y(mu0_y));
    fp_mul_c9_spp_ro MC0 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(inject|d[1]),
        .f(inject ? Zr[0] : Zr[2]), .k(inject ? chan(c_w1, inj_chn) : k3_q),
        .out_valid(), .y(mc0_y));
    fp_mul16_spp_ro  MU1 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(inject),
        .a(Zr[1]), .b(uv_2), .out_valid(), .y(mu1_y));
    fp_mul_c9_spp_ro MC1 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(inject),
        .f(Zr[1]), .k(chan(c_w2, inj_chn)), .out_valid(), .y(mc1_y));
    wire [31:0] p_mp0 = kd1 ? mu0_y : mc0_y;
    wire [31:0] p_mp1 = kd1 ? mu1_y : mc1_y;

    // -------- GEO operand REGISTER (Fix #2): the cnt-keyed operand mux is captured
    // into g?_r one cycle before the add is issued, so the wide mux (and cnt's fanout)
    // is OFF the adder S1 critical path (reg->S1, not reg->mux->S1). The plane-stream
    // operands below stay direct - they are 2:1 muxes of registered unit outputs and
    // were never the violator. Select at cnt N -> issue at cnt N+1 -> result cnt N+4.
    reg [31:0] g0a_c, g0b_c, g1a_c, g1b_c, g2a_c, g2b_c;   // selected (comb)
    reg        g0v_c, g1v_c, g2v_c;
    always @(*) begin
        g0a_c = m0_y;  g0b_c = m1_y;
        g1a_c = Yv[1]; g1b_c = Yv[0];
        g2a_c = Xv[2]; g2b_c = Xv[0];
        g0v_c = 1'b0;  g1v_c = 1'b0;  g2v_c = 1'b0;
        case (cnt)
            6'd1: begin g0a_c=Yv[2]; g0b_c=Yv[0]; g0v_c=1'b1;     // Y31
                        g1a_c=Yv[1]; g1b_c=Yv[0]; g1v_c=1'b1;     // Y21
                        g2a_c=Xv[2]; g2b_c=Xv[0]; g2v_c=1'b1; end // X31
            6'd2: begin g0a_c=Xv[1]; g0b_c=Xv[0]; g0v_c=1'b1;     // X21
                        g1a_c=Xv[0]; g1b_c=XB;    g1v_c=1'b1;     // XL1
                        g2a_c=Yv[0]; g2b_c=YB;    g2v_c=1'b1; end // YT1
            6'd3: begin g0a_c=Xv[1]; g0b_c=XB;    g0v_c=1'b1;     // XL2
                        g1a_c=Yv[1]; g1b_c=YB;    g1v_c=1'b1;     // YT2
                        g2a_c=Xv[2]; g2b_c=XB;    g2v_c=1'b1; end // XL3
            6'd4: begin g0a_c=Yv[2]; g0b_c=YB;    g0v_c=1'b1; end // YT3
            6'd8: begin g0a_c=m0_y;  g0b_c=m1_y;  g0v_c=1'b1; end // area = M0-M1
            default: ;
        endcase
    end
    reg [31:0] g0a_r, g0b_r, g1a_r, g1b_r, g2a_r, g2b_r;
    reg        g0v_r, g1v_r, g2v_r;

    // A0: GEO-only (deltas + area), always via the registered operand
    wire [31:0] a0_y;
    fp_add24_spp_ro A0 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(g0v_r),
        .a(g0a_r), .b_in(g0b_r), .sub(1'b1), .out_valid(), .y(a0_y));

    // A1: GEO via g1_r, then the plane da2/da3 stream (direct - registered sources)
    wire [31:0] a1_a = g1v_r ? g1a_r : (d[3] ? p_mp0 : p_mp1);   // da3 : da2
    wire [31:0] a1_b = g1v_r ? g1b_r : (d[3] ? p1_r  : p_mp0);
    wire [31:0] a1_y;
    fp_add24_spp_ro A1 (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(g1v_r || d[2] || d[3]),
        .a(a1_a), .b_in(a1_b), .sub(1'b1), .out_valid(), .y(a1_y));

    // A2: GEO via g2_r, then the plane Aa/Ba stream
    wire [31:0] a2_a = g2v_r ? g2a_r : (d[9] ? h2_m1 : m0_y);    // Ba : Aa
    wire [31:0] a2_b = g2v_r ? g2b_r : (d[9] ? h_m1  : h_m0);
    wire [31:0] a2_y;
    fp_add24_spp_ro A2 (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(g2v_r || d[8] || d[9]),
        .a(a2_a), .b_in(a2_b), .sub(1'b1), .out_valid(), .y(a2_y));

    // M0/M1: area products (cnt 6: X21 on A0.y then, Y31r/X31r/Y21r captured cnt 5)
    // then the plane mC stream
    wire [31:0] m0_a = (cnt==6'd6) ? a0_y : a1_y;                    // X21 / da2 / da3
    wire [31:0] m0_b = (cnt==6'd6) ? Y31r : (d[6] ? Y21r : Y31r);
    wire [31:0] m1_a = (cnt==6'd6) ? X31r : (d[6] ? X21r : X31r);
    wire [31:0] m1_b = (cnt==6'd6) ? Y21r : a1_y;
    wire [31:0] m0_y, m1_y;
    fp_mul16_spp_ro M0 (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(run && cnt==6'd6 || d[5] || d[6]),
        .a(m0_a), .b(m0_b), .out_valid(), .y(m0_y));
    fp_mul16_spp_ro M1 (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(run && cnt==6'd6 || d[5] || d[6]),
        .a(m1_a), .b(m1_b), .out_valid(), .y(m1_y));

    // MD: ddx/ddy = -Aa/-Ba * rcy (fixed routing)
    wire [31:0] md_y;
    fp_mul16_spp_ro MD (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(d[11] || d[12]),
        .a(fneg(a2_y)), .b(rcy), .out_valid(), .y(md_y));

    // MF: f0/f1 = ddx*XLa / ddy*YTa
    wire [31:0] mf_y;
    fp_mul16_spp_ro MF (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(d[13] || d[14]),
        .a(md_y), .b(d[13] ? aXLr : aYTr), .out_valid(), .y(mf_y));

    // A3: c = pa - f0 - f1
    wire [31:0] a3_y;
    fp_add3_24_spp_ro A3 (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(d[16]),
        .a(pa_sr[11]), .b(fneg(h_mf)), .c(fneg(mf_y)), .out_valid(), .y(a3_y));

    // RCP: 1/area, once per triangle (area = A0 result, valid cnt 12)
    wire        rc_ack; wire [31:0] rc_y;
    fp_rcp_fast u_rcp (.clk(clk),.reset(reset),.stall(1'b0),
        .in_valid(run && cnt==6'd12), .x(a0_y), .out_valid(rc_ack), .y(rc_y));

    // anchor-p mux (correct only during a plane's T+4 cycle - p1/p2/p3 all in holds by
    // then, and the anchor regs are valid from cnt 10 = T+4 of the first plane)
    wire [31:0] pa_mux = (anch==2'd2) ? p3_r : (anch==2'd1) ? p2_r : p1_r;

    // ======================= sequential control =======================
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            run<=1'b0; done<=1'b0; plane_valid<=1'b0; fin<=1'b0;
            cnt<=6'd0; d<='0;
            g0v_r<=1'b0; g1v_r<=1'b0; g2v_r<=1'b0;
        end else begin
            done<=1'b0; plane_valid<=1'b0;

            if (!run) begin
                g0v_r<=1'b0; g1v_r<=1'b0; g2v_r<=1'b0;
                if (start) begin
                    Zr[0]<=z1; Zr[1]<=z2; Zr[2]<=z3;
                    Xv[0]<=x1; Xv[1]<=x2; Xv[2]<=x3;
                    Yv[0]<=y1; Yv[1]<=y2; Yv[2]<=y3;
                    U[0]<=u1; U[1]<=u2; U[2]<=u3;
                    V[0]<=v1; V[1]<=v2; V[2]<=v3;
                    CO[0]<=col1; CO[1]<=col2; CO[2]<=col3;
                    OF[0]<=ofs1; OF[1]<=ofs2; OF[2]<=ofs3;
                    XB<=xbase; YB<=ybase;
                    g_r<=gouraud; tex_r<=texture; ofs_r<=offset;
                    n_planes <= 4'd4 + (texture ? 4'd2 : 4'd0) + (offset ? 4'd4 : 4'd0);
                    nextp<=4'd0; emit_cnt<=4'd0; fin<=1'b0;
                    d<='0; cnt<=6'd1; run<=1'b1;
                end
            end else begin
                cnt <= (cnt==6'd63) ? cnt : cnt + 6'd1;

                // ---- GEO operand registration (Fix #2): sel cnt N -> issue cnt N+1
                g0a_r<=g0a_c; g0b_r<=g0b_c; g0v_r<=g0v_c;
                g1a_r<=g1a_c; g1b_r<=g1b_c; g1v_r<=g1v_c;
                g2a_r<=g2a_c; g2b_r<=g2b_c; g2v_r<=g2v_c;

                // ---- GEO result captures (adder result = sel + 4) ----
                if (cnt==6'd5) begin Y31r<=a0_y; Y21r<=a1_y; X31r<=a2_y; end
                if (cnt==6'd6) begin X21r<=a0_y; XL1r<=a1_y; YT1r<=a2_y; end
                if (cnt==6'd7) begin XL2r<=a0_y; YT2r<=a1_y; XL3r<=a2_y; end
                if (cnt==6'd8) begin YT3r<=a0_y; end
                // min-magnitude anchor (all six offsets captured by now)
                if (cnt==6'd9) begin
                    if (amag(XL3r,YT3r) < amag(XL1r,YT1r) &&
                        amag(XL3r,YT3r) < amag(XL2r,YT2r)) begin
                        anch<=2'd2; aXLr<=XL3r; aYTr<=YT3r;
                    end else if (amag(XL2r,YT2r) < amag(XL1r,YT1r)) begin
                        anch<=2'd1; aXLr<=XL2r; aYTr<=YT2r;
                    end else begin
                        anch<=2'd0; aXLr<=XL1r; aYTr<=YT1r;
                    end
                end
                if (rc_ack) rcy <= rc_y;

                // ---- inject ----
                if (inject) begin
                    nextp    <= inj_idx + 4'd1;
                    uv3_q    <= uv_3;
                    k3_q     <= chan(c_w3, inj_chn);
                    inj_uv_q <= inj_uv;
                end

                // ---- pulse + carry delay lines (shift every clock) ----
                d[1] <= inject;
                for (i=2; i<=20; i=i+1) d[i] <= d[i-1];
                idx_sr[0] <= inj_idx;
                for (i=1; i<=19; i=i+1) idx_sr[i] <= idx_sr[i-1];
                pa_sr[0] <= pa_mux;
                for (i=1; i<=11; i=i+1) pa_sr[i] <= pa_sr[i-1];
                dxy_sr[0] <= md_y;
                for (i=1; i<=6;  i=i+1) dxy_sr[i] <= dxy_sr[i-1];
                kd0 <= inject ? inj_uv : inj_uv_q;
                kd1 <= kd0;
                h_m0 <= m0_y;  h_m1 <= m1_y;  h2_m1 <= h_m1;  h_mf <= mf_y;
                if (d[2]) begin p1_r <= p_mp0; p2_r <= p_mp1; end
                if (d[3]) begin p3_r <= p_mp0; end   // p3 held for the T+4 pa mux

                // ---- emit (A3 is 4-clock: d[16] issue -> result at d[20]) ----
                if (d[20]) begin
                    plane_valid <= 1'b1;
                    plane_idx   <= idx_sr[19];
                    o_ddx <= dxy_sr[6];
                    o_ddy <= dxy_sr[5];
                    o_c   <= a3_y;
                    emit_cnt <= emit_cnt + 4'd1;
                    if (emit_cnt + 4'd1 == n_planes) fin <= 1'b1;
                end
                if (fin) begin fin<=1'b0; done<=1'b1; run<=1'b0; end
            end
        end
    end
endmodule
