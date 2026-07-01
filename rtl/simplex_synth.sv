//
// simplex_synth - synthesizable top for standalone Quartus compilation.
//
// The ISP/TSP setup units are purely combinational and consume ~100 32-bit
// words of geometry; that can't map to physical pins directly. So this wrapper:
//   * holds the VRAM scene in a small MIF-initialized ROM (like gauss_lut),
//   * registers the setup-unit outputs into one accumulator,
//   * reduces that accumulator to a 1-bit output.
// The fitter therefore keeps ALL the setup logic (it feeds an observable
// output) and the design fits in a single clock + a couple of pins, giving an
// honest resource/timing report for the ISP and TSP setup datapaths.
//
// This is a SYNTHESIS HARNESS, not a runnable core. Functional verification is
// done in simulation against the C golden model (see tb/, make sim-tri/quad).
//
module simplex_synth (
    input         clk,
    input         reset,
    input         is_quad,
    output reg    digest      // XOR-reduction of all setup outputs (observable)
);
    // ---- VRAM scene ROM (32 x 32-bit, MIF-initialized) ----
    (* ram_init_file = "build/vram.mif" *)
    reg [31:0] vram [0:31];

    // word accessors (matches tri_setup_top layout: stride 7, base 3)
    localparam VB = 3, STR = 7;
    `define VX(n) vram[VB + (n)*STR + 0]
    `define VY(n) vram[VB + (n)*STR + 1]
    `define VZ(n) vram[VB + (n)*STR + 2]
    `define VU(n) vram[VB + (n)*STR + 3]
    `define VV(n) vram[VB + (n)*STR + 4]
    `define VCOL(n) vram[VB + (n)*STR + 5]
    `define VOFS(n) vram[VB + (n)*STR + 6]

    wire [31:0] isp_tsp = vram[0];
    wire        gouraud = isp_tsp[8];

    // ---- colour byte -> float ----
    wire [31:0] v1c [0:3], v2c [0:3], v3c [0:3];
    wire [31:0] v1o [0:3], v2o [0:3], v3o [0:3];
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_colcvt
            u8_to_float c1cv (.u(`VCOL(0)[gi*8 +: 8]), .f(v1c[gi]));
            u8_to_float c2cv (.u(`VCOL(1)[gi*8 +: 8]), .f(v2c[gi]));
            u8_to_float c3cv (.u(`VCOL(2)[gi*8 +: 8]), .f(v3c[gi]));
            u8_to_float o1cv (.u(`VOFS(0)[gi*8 +: 8]), .f(v1o[gi]));
            u8_to_float o2cv (.u(`VOFS(1)[gi*8 +: 8]), .f(v2o[gi]));
            u8_to_float o3cv (.u(`VOFS(2)[gi*8 +: 8]), .f(v3o[gi]));
        end
    endgenerate

    // ---- ISP setup ----
    wire        sgn_neg, cull, t1, t2, t3, t4;
    wire [31:0] dx12,dx23,dx31,dx41, dy12,dy23,dy31,dy41, c1,c2,c3,c4;
    wire [31:0] z_ddx,z_ddy,z_c;
    wire [31:0] geo_dx12,geo_dx13,geo_dy12,geo_dy13,geo_carea,geo_v1x_l,geo_v1y_t;

    isp_setup u_isp (
        .isp_tsp(isp_tsp), .is_quad(is_quad),
        .v1x(`VX(0)), .v1y(`VY(0)), .v1z(`VZ(0)),
        .v2x(`VX(1)), .v2y(`VY(1)), .v2z(`VZ(1)),
        .v3x(`VX(2)), .v3y(`VY(2)), .v3z(`VZ(2)),
        .v4x(`VX(3)), .v4y(`VY(3)),
        .rect_left(32'h00000000), .rect_top(32'h00000000),
        .sgn_neg(sgn_neg), .cull(cull),
        .dx12_e(dx12), .dx23_e(dx23), .dx31_e(dx31), .dx41_e(dx41),
        .dy12_e(dy12), .dy23_e(dy23), .dy31_e(dy31), .dy41_e(dy41),
        .c1(c1), .c2(c2), .c3(c3), .c4(c4),
        .t1(t1), .t2(t2), .t3(t3), .t4(t4),
        .z_ddx(z_ddx), .z_ddy(z_ddy), .z_c(z_c),
        .geo_dx12(geo_dx12), .geo_dx13(geo_dx13),
        .geo_dy12(geo_dy12), .geo_dy13(geo_dy13),
        .geo_carea(geo_carea), .geo_v1x_l(geo_v1x_l), .geo_v1y_t(geo_v1y_t)
    );

    // ---- TSP setup ----
    wire [31:0] u_ddx,u_ddy,u_c, v_ddx,v_ddy,v_c;
    wire [31:0] col0_ddx,col0_ddy,col0_c, col1_ddx,col1_ddy,col1_c;
    wire [31:0] col2_ddx,col2_ddy,col2_c, col3_ddx,col3_ddy,col3_c;
    wire [31:0] ofs0_ddx,ofs0_ddy,ofs0_c, ofs1_ddx,ofs1_ddy,ofs1_c;
    wire [31:0] ofs2_ddx,ofs2_ddy,ofs2_c, ofs3_ddx,ofs3_ddy,ofs3_c;

    tsp_setup u_tsp (
        .gouraud(gouraud),
        .dx12(geo_dx12), .dx13(geo_dx13), .dy12(geo_dy12), .dy13(geo_dy13),
        .c_area(geo_carea), .v1x_l(geo_v1x_l), .v1y_t(geo_v1y_t),
        .v1z(`VZ(0)), .v2z(`VZ(1)), .v3z(`VZ(2)),
        .v1u(`VU(0)), .v1v(`VV(0)),
        .v2u(`VU(1)), .v2v(`VV(1)),
        .v3u(`VU(2)), .v3v(`VV(2)),
        .v1col0(v1c[0]), .v1col1(v1c[1]), .v1col2(v1c[2]), .v1col3(v1c[3]),
        .v2col0(v2c[0]), .v2col1(v2c[1]), .v2col2(v2c[2]), .v2col3(v2c[3]),
        .v3col0(v3c[0]), .v3col1(v3c[1]), .v3col2(v3c[2]), .v3col3(v3c[3]),
        .v1spc0(v1o[0]), .v1spc1(v1o[1]), .v1spc2(v1o[2]), .v1spc3(v1o[3]),
        .v2spc0(v2o[0]), .v2spc1(v2o[1]), .v2spc2(v2o[2]), .v2spc3(v2o[3]),
        .v3spc0(v3o[0]), .v3spc1(v3o[1]), .v3spc2(v3o[2]), .v3spc3(v3o[3]),
        .u_ddx(u_ddx), .u_ddy(u_ddy), .u_c(u_c),
        .v_ddx(v_ddx), .v_ddy(v_ddy), .v_c(v_c),
        .col0_ddx(col0_ddx), .col0_ddy(col0_ddy), .col0_c(col0_c),
        .col1_ddx(col1_ddx), .col1_ddy(col1_ddy), .col1_c(col1_c),
        .col2_ddx(col2_ddx), .col2_ddy(col2_ddy), .col2_c(col2_c),
        .col3_ddx(col3_ddx), .col3_ddy(col3_ddy), .col3_c(col3_c),
        .ofs0_ddx(ofs0_ddx), .ofs0_ddy(ofs0_ddy), .ofs0_c(ofs0_c),
        .ofs1_ddx(ofs1_ddx), .ofs1_ddy(ofs1_ddy), .ofs1_c(ofs1_c),
        .ofs2_ddx(ofs2_ddx), .ofs2_ddy(ofs2_ddy), .ofs2_c(ofs2_c),
        .ofs3_ddx(ofs3_ddx), .ofs3_ddy(ofs3_ddy), .ofs3_c(ofs3_c)
    );

    // ---- observable digest: register the XOR of every output ----
    wire [31:0] acc =
        dx12^dx23^dx31^dx41 ^ dy12^dy23^dy31^dy41 ^ c1^c2^c3^c4 ^
        z_ddx^z_ddy^z_c ^ u_ddx^u_ddy^u_c ^ v_ddx^v_ddy^v_c ^
        col0_ddx^col0_ddy^col0_c ^ col1_ddx^col1_ddy^col1_c ^
        col2_ddx^col2_ddy^col2_c ^ col3_ddx^col3_ddy^col3_c ^
        ofs0_ddx^ofs0_ddy^ofs0_c ^ ofs1_ddx^ofs1_ddy^ofs1_c ^
        ofs2_ddx^ofs2_ddy^ofs2_c ^ ofs3_ddx^ofs3_ddy^ofs3_c ^
        {28'd0, sgn_neg, cull, 1'b0, 1'b0} ^ {28'd0, t1, t2, t3, t4};

    always @(posedge clk) begin
        if (reset) digest <= 1'b0;
        else       digest <= ^acc;   // 1-bit reduction, keeps all logic live
    end
endmodule
