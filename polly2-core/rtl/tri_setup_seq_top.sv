//
// tri_setup_seq_top - clocked top tying the two sequenced setup units together.
//
// Mirrors tri_setup_top, but uses the area-optimized clocked units:
//   isp_setup_seq : one mul+add + the single reciprocal; produces ISP coeffs
//                   and exports recip_C + shared geometry.
//   tsp_setup_seq : one mul+add; consumes recip_C (no second reciprocal) and
//                   produces the 10 TSP planes.
//
// Sequencing: pulse 'start' -> run ISP -> when ISP 'done', start TSP -> when TSP
// 'done', pulse top-level 'done'. All outputs are held registers.
//
// VRAM is a $readmemh image (sim) / MIF ROM (synth), same 7-word vertex stride
// as tri_setup_top.
//
module tri_setup_seq_top #(
    parameter VRAM_WORDS = 64,
    // Plain hex (one 32-bit word/line) read by $readmemh; works for Verilator
    // sim and for Quartus ROM inference. Do NOT point at a .mif.
    parameter VRAM_INIT  = "build/vram.hex"
)(
    input         clk,
    input         reset,
    input         start,
    input         is_quad,
    input  [31:0] rect_left,
    input  [31:0] rect_top,
    output        done,

    output [31:0] isp_tsp, output [31:0] tsp_word, output [31:0] tcw_word,

    output        sgn_neg, output cull,
    output [31:0] dx12,output [31:0] dx23,output [31:0] dx31,output [31:0] dx41,
    output [31:0] dy12,output [31:0] dy23,output [31:0] dy31,output [31:0] dy41,
    output [31:0] c1,output [31:0] c2,output [31:0] c3,output [31:0] c4,
    output t1,output t2,output t3,output t4,
    output [31:0] z_ddx,output [31:0] z_ddy,output [31:0] z_c,

    output [31:0] u_ddx,output [31:0] u_ddy,output [31:0] u_c,
    output [31:0] v_ddx,output [31:0] v_ddy,output [31:0] v_c,
    output [31:0] col0_ddx,output [31:0] col0_ddy,output [31:0] col0_c,
    output [31:0] col1_ddx,output [31:0] col1_ddy,output [31:0] col1_c,
    output [31:0] col2_ddx,output [31:0] col2_ddy,output [31:0] col2_c,
    output [31:0] col3_ddx,output [31:0] col3_ddy,output [31:0] col3_c,
    output [31:0] ofs0_ddx,output [31:0] ofs0_ddy,output [31:0] ofs0_c,
    output [31:0] ofs1_ddx,output [31:0] ofs1_ddy,output [31:0] ofs1_c,
    output [31:0] ofs2_ddx,output [31:0] ofs2_ddy,output [31:0] ofs2_c,
    output [31:0] ofs3_ddx,output [31:0] ofs3_ddy,output [31:0] ofs3_c
);
    // VRAM image. The ROM-init mechanism is gated by its OWN macro
    // (USE_MIF_ROM), kept SEPARATE from the FP-IP choice (`SYNTHESIS) so the
    // Altera-IP datapath can still be exercised in Verilator with $readmemh:
    //   USE_MIF_ROM : Quartus ram_init_file MIF (mirrors gsplat gauss_lut.sv).
    //                 The MIF format would crash $readmemh ("illegal char 'p'"
    //                 on the DEPTH= header), so $readmemh is compiled out here.
    //   otherwise   : $readmemh on a plain hex file (Verilator sim, incl. the
    //                 IP-stub test which defines `SYNTHESIS but not USE_MIF_ROM).
`ifdef USE_MIF_ROM
    (* ram_init_file = "build/vram.mif" *)
    reg [31:0] vram [0:VRAM_WORDS-1];
`else
    reg [31:0] vram [0:VRAM_WORDS-1];
    initial $readmemh(VRAM_INIT, vram);
`endif

    assign isp_tsp  = vram[0];
    assign tsp_word = vram[1];
    assign tcw_word = vram[2];

    localparam VB=3, STR=7;
    `define VX(n) vram[VB+(n)*STR+0]
    `define VY(n) vram[VB+(n)*STR+1]
    `define VZ(n) vram[VB+(n)*STR+2]
    `define VU(n) vram[VB+(n)*STR+3]
    `define VV(n) vram[VB+(n)*STR+4]
    `define VCOL(n) vram[VB+(n)*STR+5]
    `define VOFS(n) vram[VB+(n)*STR+6]

    wire [31:0] v1c[0:3],v2c[0:3],v3c[0:3],v1o[0:3],v2o[0:3],v3o[0:3];
    genvar gi;
    generate for (gi=0;gi<4;gi=gi+1) begin: g_cv
        u8_to_float a0(.u(`VCOL(0)[gi*8+:8]),.f(v1c[gi]));
        u8_to_float a1(.u(`VCOL(1)[gi*8+:8]),.f(v2c[gi]));
        u8_to_float a2(.u(`VCOL(2)[gi*8+:8]),.f(v3c[gi]));
        u8_to_float b0(.u(`VOFS(0)[gi*8+:8]),.f(v1o[gi]));
        u8_to_float b1(.u(`VOFS(1)[gi*8+:8]),.f(v2o[gi]));
        u8_to_float b2(.u(`VOFS(2)[gi*8+:8]),.f(v3o[gi]));
    end endgenerate

    wire gouraud = isp_tsp[8];

    // ---- ISP ----
    wire isp_done;
    wire [31:0] g_dx12,g_dx13,g_dy12,g_dy13,g_recipC,g_v1xl,g_v1yt,g_z1,g_z2,g_z3;
    reg isp_start;

    isp_setup_seq u_isp (
        .clk(clk),.reset(reset),.start(isp_start),.done(isp_done),
        .isp_tsp(isp_tsp),.is_quad(is_quad),
        .v1x(`VX(0)),.v1y(`VY(0)),.v1z(`VZ(0)),
        .v2x(`VX(1)),.v2y(`VY(1)),.v2z(`VZ(1)),
        .v3x(`VX(2)),.v3y(`VY(2)),.v3z(`VZ(2)),
        .v4x(`VX(3)),.v4y(`VY(3)),
        .rect_left(rect_left),.rect_top(rect_top),
        .sgn_neg(sgn_neg),.cull(cull),
        .dx12(dx12),.dx23(dx23),.dx31(dx31),.dx41(dx41),
        .dy12(dy12),.dy23(dy23),.dy31(dy31),.dy41(dy41),
        .c1(c1),.c2(c2),.c3(c3),.c4(c4),.t1(t1),.t2(t2),.t3(t3),.t4(t4),
        .z_ddx(z_ddx),.z_ddy(z_ddy),.z_c(z_c),
        .geo_dx12(g_dx12),.geo_dx13(g_dx13),.geo_dy12(g_dy12),.geo_dy13(g_dy13),
        .geo_recipC(g_recipC),.geo_v1x_l(g_v1xl),.geo_v1y_t(g_v1yt),
        .geo_v1z(g_z1),.geo_v2z(g_z2),.geo_v3z(g_z3)
    );

    // ---- TSP ----
    wire tsp_done;
    reg tsp_start;

    tsp_setup_seq u_tsp (
        .clk(clk),.reset(reset),.start(tsp_start),.done(tsp_done),
        .gouraud(gouraud),
        .dx12(g_dx12),.dx13(g_dx13),.dy12(g_dy12),.dy13(g_dy13),
        .recipC(g_recipC),.v1x_l(g_v1xl),.v1y_t(g_v1yt),
        .v1z(g_z1),.v2z(g_z2),.v3z(g_z3),
        .v1u(`VU(0)),.v1v(`VV(0)),.v2u(`VU(1)),.v2v(`VV(1)),.v3u(`VU(2)),.v3v(`VV(2)),
        .v1col0(v1c[0]),.v1col1(v1c[1]),.v1col2(v1c[2]),.v1col3(v1c[3]),
        .v2col0(v2c[0]),.v2col1(v2c[1]),.v2col2(v2c[2]),.v2col3(v2c[3]),
        .v3col0(v3c[0]),.v3col1(v3c[1]),.v3col2(v3c[2]),.v3col3(v3c[3]),
        .v1spc0(v1o[0]),.v1spc1(v1o[1]),.v1spc2(v1o[2]),.v1spc3(v1o[3]),
        .v2spc0(v2o[0]),.v2spc1(v2o[1]),.v2spc2(v2o[2]),.v2spc3(v2o[3]),
        .v3spc0(v3o[0]),.v3spc1(v3o[1]),.v3spc2(v3o[2]),.v3spc3(v3o[3]),
        .u_ddx(u_ddx),.u_ddy(u_ddy),.u_c(u_c),
        .v_ddx(v_ddx),.v_ddy(v_ddy),.v_c(v_c),
        .col0_ddx(col0_ddx),.col0_ddy(col0_ddy),.col0_c(col0_c),
        .col1_ddx(col1_ddx),.col1_ddy(col1_ddy),.col1_c(col1_c),
        .col2_ddx(col2_ddx),.col2_ddy(col2_ddy),.col2_c(col2_c),
        .col3_ddx(col3_ddx),.col3_ddy(col3_ddy),.col3_c(col3_c),
        .ofs0_ddx(ofs0_ddx),.ofs0_ddy(ofs0_ddy),.ofs0_c(ofs0_c),
        .ofs1_ddx(ofs1_ddx),.ofs1_ddy(ofs1_ddy),.ofs1_c(ofs1_c),
        .ofs2_ddx(ofs2_ddx),.ofs2_ddy(ofs2_ddy),.ofs2_c(ofs2_c),
        .ofs3_ddx(ofs3_ddx),.ofs3_ddy(ofs3_ddy),.ofs3_c(ofs3_c)
    );

    // ---- sequencing FSM ----
    localparam T_IDLE=0,T_ISP=1,T_TSP=2,T_DONE=3;
    reg [1:0] tstate;
    reg done_r;
    assign done = done_r;
    always @(posedge clk) begin
        if (reset) begin tstate<=T_IDLE; isp_start<=0; tsp_start<=0; done_r<=0; end
        else begin
            isp_start<=0; tsp_start<=0; done_r<=0;
            case (tstate)
              T_IDLE: if (start) begin isp_start<=1; tstate<=T_ISP; end
              T_ISP:  if (isp_done) begin tsp_start<=1; tstate<=T_TSP; end
              T_TSP:  if (tsp_done) begin done_r<=1; tstate<=T_DONE; end
              T_DONE: tstate<=T_IDLE;
            endcase
        end
    end
endmodule
