//
// tsp_setup - TSP triangle setup unit (port of IPs3::Setup, refsw_tile.h:84,
//             single-volume case).
//
// Computes the perspective-correct interpolation planes for the textured /
// shaded attributes:
//
//   U, V            - texture coords, stored as (u*invW), (v*invW)
//   Col[0..3]       - base colour BGRA, stored as (col[i]*invW)
//   Ofs[0..3]       - offset colour BGRA, stored as (spc[i]*invW)
//
// Each attribute is first multiplied by the vertex's invW (z), then fed through
// a shared plane_stepper using the SAME geometry terms computed by isp_setup.
// Gouraud=1 uses per-vertex colours; Gouraud=0 (flat) uses vertex 3's colour
// for all three vertices (refsw_tile.h:95).
//
// Colour inputs arrive as floats already converted from the u8 vertex bytes
// (the testbench / vertex decoder does the u8->float widening). Channel order
// matches refsw2's col[] = {b, g, r, a}.
//
// Pure combinational; IEEE-754 throughout. Reuses geometry from isp_setup so
// the divide-by-C count stays at 2 per plane (not re-deriving C here).
//
module tsp_setup (
    input         gouraud,    // isp_tsp.Gouraud

    // shared geometry from isp_setup
    input  [31:0] dx12, input [31:0] dx13,
    input  [31:0] dy12, input [31:0] dy13,
    input  [31:0] c_area,
    input  [31:0] v1x_l, input [31:0] v1y_t,

    // per-vertex invW (= z)
    input  [31:0] v1z, input [31:0] v2z, input [31:0] v3z,

    // per-vertex texture coords (float)
    input  [31:0] v1u, input [31:0] v1v,
    input  [31:0] v2u, input [31:0] v2v,
    input  [31:0] v3u, input [31:0] v3v,

    // per-vertex base colour BGRA (float, 0..255)
    input  [31:0] v1col0, input [31:0] v1col1, input [31:0] v1col2, input [31:0] v1col3,
    input  [31:0] v2col0, input [31:0] v2col1, input [31:0] v2col2, input [31:0] v2col3,
    input  [31:0] v3col0, input [31:0] v3col1, input [31:0] v3col2, input [31:0] v3col3,

    // per-vertex offset colour BGRA (float, 0..255)
    input  [31:0] v1spc0, input [31:0] v1spc1, input [31:0] v1spc2, input [31:0] v1spc3,
    input  [31:0] v2spc0, input [31:0] v2spc1, input [31:0] v2spc2, input [31:0] v2spc3,
    input  [31:0] v3spc0, input [31:0] v3spc1, input [31:0] v3spc2, input [31:0] v3spc3,

    // texture planes
    output [31:0] u_ddx, output [31:0] u_ddy, output [31:0] u_c,
    output [31:0] v_ddx, output [31:0] v_ddy, output [31:0] v_c,

    // base colour planes (index 0..3 = B,G,R,A)
    output [31:0] col0_ddx, output [31:0] col0_ddy, output [31:0] col0_c,
    output [31:0] col1_ddx, output [31:0] col1_ddy, output [31:0] col1_c,
    output [31:0] col2_ddx, output [31:0] col2_ddy, output [31:0] col2_c,
    output [31:0] col3_ddx, output [31:0] col3_ddy, output [31:0] col3_c,

    // offset colour planes
    output [31:0] ofs0_ddx, output [31:0] ofs0_ddy, output [31:0] ofs0_c,
    output [31:0] ofs1_ddx, output [31:0] ofs1_ddy, output [31:0] ofs1_c,
    output [31:0] ofs2_ddx, output [31:0] ofs2_ddy, output [31:0] ofs2_c,
    output [31:0] ofs3_ddx, output [31:0] ofs3_ddy, output [31:0] ofs3_c
);
    // Convenience: instantiate a plane for attribute (a1,a2,a3).
    `define PLANE(NAME, A1, A2, A3, DDX, DDY, C) \
        plane_stepper NAME ( \
            .dx12(dx12), .dx13(dx13), .dy12(dy12), .dy13(dy13), \
            .c_area(c_area), .v1x_l(v1x_l), .v1y_t(v1y_t), \
            .a1(A1), .a2(A2), .a3(A3), \
            .ddx(DDX), .ddy(DDY), .c(C) )

    // ---- texture coords: u*z, v*z ----
    wire [31:0] u1z, u2z, u3z, vv1z, vv2z, vv3z;
    fp_mul mu1 (.a(v1u), .b(v1z), .y(u1z));
    fp_mul mu2 (.a(v2u), .b(v2z), .y(u2z));
    fp_mul mu3 (.a(v3u), .b(v3z), .y(u3z));
    fp_mul mv1 (.a(v1v), .b(v1z), .y(vv1z));
    fp_mul mv2 (.a(v2v), .b(v2z), .y(vv2z));
    fp_mul mv3 (.a(v3v), .b(v3z), .y(vv3z));
    `PLANE(ps_u, u1z, u2z, u3z, u_ddx, u_ddy, u_c);
    `PLANE(ps_v, vv1z, vv2z, vv3z, v_ddx, v_ddy, v_c);

    // ---- base colour. attribute source depends on gouraud ----
    // gouraud:  a1=v1col*v1z, a2=v2col*v2z, a3=v3col*v3z
    // flat   :  a1=v3col*v1z, a2=v3col*v2z, a3=v3col*v3z
    wire [31:0] c1src0 = gouraud ? v1col0 : v3col0;
    wire [31:0] c1src1 = gouraud ? v1col1 : v3col1;
    wire [31:0] c1src2 = gouraud ? v1col2 : v3col2;
    wire [31:0] c1src3 = gouraud ? v1col3 : v3col3;
    wire [31:0] c2src0 = gouraud ? v2col0 : v3col0;
    wire [31:0] c2src1 = gouraud ? v2col1 : v3col1;
    wire [31:0] c2src2 = gouraud ? v2col2 : v3col2;
    wire [31:0] c2src3 = gouraud ? v2col3 : v3col3;
    // vertex3 source is always v3col

    // base colour * z
    wire [31:0] cb0_1,cb0_2,cb0_3, cb1_1,cb1_2,cb1_3, cb2_1,cb2_2,cb2_3, cb3_1,cb3_2,cb3_3;
    fp_mul cm00 (.a(c1src0), .b(v1z), .y(cb0_1));
    fp_mul cm01 (.a(c2src0), .b(v2z), .y(cb0_2));
    fp_mul cm02 (.a(v3col0), .b(v3z), .y(cb0_3));
    fp_mul cm10 (.a(c1src1), .b(v1z), .y(cb1_1));
    fp_mul cm11 (.a(c2src1), .b(v2z), .y(cb1_2));
    fp_mul cm12 (.a(v3col1), .b(v3z), .y(cb1_3));
    fp_mul cm20 (.a(c1src2), .b(v1z), .y(cb2_1));
    fp_mul cm21 (.a(c2src2), .b(v2z), .y(cb2_2));
    fp_mul cm22 (.a(v3col2), .b(v3z), .y(cb2_3));
    fp_mul cm30 (.a(c1src3), .b(v1z), .y(cb3_1));
    fp_mul cm31 (.a(c2src3), .b(v2z), .y(cb3_2));
    fp_mul cm32 (.a(v3col3), .b(v3z), .y(cb3_3));
    `PLANE(ps_col0, cb0_1, cb0_2, cb0_3, col0_ddx, col0_ddy, col0_c);
    `PLANE(ps_col1, cb1_1, cb1_2, cb1_3, col1_ddx, col1_ddy, col1_c);
    `PLANE(ps_col2, cb2_1, cb2_2, cb2_3, col2_ddx, col2_ddy, col2_c);
    `PLANE(ps_col3, cb3_1, cb3_2, cb3_3, col3_ddx, col3_ddy, col3_c);

    // ---- offset colour. same gouraud selection ----
    wire [31:0] o1src0 = gouraud ? v1spc0 : v3spc0;
    wire [31:0] o1src1 = gouraud ? v1spc1 : v3spc1;
    wire [31:0] o1src2 = gouraud ? v1spc2 : v3spc2;
    wire [31:0] o1src3 = gouraud ? v1spc3 : v3spc3;
    wire [31:0] o2src0 = gouraud ? v2spc0 : v3spc0;
    wire [31:0] o2src1 = gouraud ? v2spc1 : v3spc1;
    wire [31:0] o2src2 = gouraud ? v2spc2 : v3spc2;
    wire [31:0] o2src3 = gouraud ? v2spc3 : v3spc3;

    wire [31:0] ob0_1,ob0_2,ob0_3, ob1_1,ob1_2,ob1_3, ob2_1,ob2_2,ob2_3, ob3_1,ob3_2,ob3_3;
    fp_mul om00 (.a(o1src0), .b(v1z), .y(ob0_1));
    fp_mul om01 (.a(o2src0), .b(v2z), .y(ob0_2));
    fp_mul om02 (.a(v3spc0), .b(v3z), .y(ob0_3));
    fp_mul om10 (.a(o1src1), .b(v1z), .y(ob1_1));
    fp_mul om11 (.a(o2src1), .b(v2z), .y(ob1_2));
    fp_mul om12 (.a(v3spc1), .b(v3z), .y(ob1_3));
    fp_mul om20 (.a(o1src2), .b(v1z), .y(ob2_1));
    fp_mul om21 (.a(o2src2), .b(v2z), .y(ob2_2));
    fp_mul om22 (.a(v3spc2), .b(v3z), .y(ob2_3));
    fp_mul om30 (.a(o1src3), .b(v1z), .y(ob3_1));
    fp_mul om31 (.a(o2src3), .b(v2z), .y(ob3_2));
    fp_mul om32 (.a(v3spc3), .b(v3z), .y(ob3_3));
    `PLANE(ps_ofs0, ob0_1, ob0_2, ob0_3, ofs0_ddx, ofs0_ddy, ofs0_c);
    `PLANE(ps_ofs1, ob1_1, ob1_2, ob1_3, ofs1_ddx, ofs1_ddy, ofs1_c);
    `PLANE(ps_ofs2, ob2_1, ob2_2, ob2_3, ofs2_ddx, ofs2_ddy, ofs2_c);
    `PLANE(ps_ofs3, ob3_1, ob3_2, ob3_3, ofs3_ddx, ofs3_ddy, ofs3_c);

    `undef PLANE
endmodule
