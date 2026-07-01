//
// isp_setup - ISP triangle setup unit (port of RasterizeTriangle setup,
//             refsw_tile.cpp:536-620, triangle case / no v4).
//
// Given the three vertex positions (x,y,z=1/w) and the tile rect origin
// (left,top), it produces everything the per-pixel ISP rasterizer needs that
// is computed once per triangle:
//
//   * sgn          - winding sign (+1, or -1 if tri_area>0)
//   * cull         - 1 if the triangle should be discarded (CullMode logic)
//   * DX12/23/31, DY12/23/31, C1/C2/C3   - half-edge constants
//   * T1/T2/T3     - top-left edge inclusion flags (IsTopLeft)
//   * z_ddx/z_ddy/z_c - the invW (depth) interpolation plane
//
// Also exposes the shared geometry terms (dx12,dx13,dy12,dy13,c_area,v1x_l,
// v1y_t) so tsp_setup can reuse them without recomputing.
//
// Pure combinational; the testbench registers the outputs. IEEE-754 throughout.
//
// ISP_TSP field layout (core_structs.h:59): DepthMode[2:0], CullMode[4:3],
//   ZWriteDis[5], Texture[6], Offset[7], Gouraud[8], UV_16b[9].
//
module isp_setup #(
    parameter [31:0] FP_FPU_CULL_VAL = 32'h00000000  // FPU_CULL_VAL (default 0)
)(
    input  [31:0] isp_tsp,     // ISP/TSP instruction word

    input         is_quad,    // 1 => use 4th vertex (quad), else triangle

    input  [31:0] v1x, input [31:0] v1y, input [31:0] v1z,
    input  [31:0] v2x, input [31:0] v2y, input [31:0] v2z,
    input  [31:0] v3x, input [31:0] v3y, input [31:0] v3z,
    input  [31:0] v4x, input [31:0] v4y,   // 4th vertex position (quad only)

    input  [31:0] rect_left,
    input  [31:0] rect_top,

    // winding / cull
    output        sgn_neg,     // 1 => sgn = -1
    output        cull,

    // half-edge constants (edge 31 is v3->v1 for tris, v3->v4 for quads;
    //  edge 41 (v4->v1) is only meaningful when is_quad)
    output [31:0] dx12_e, output [31:0] dx23_e, output [31:0] dx31_e, output [31:0] dx41_e,
    output [31:0] dy12_e, output [31:0] dy23_e, output [31:0] dy31_e, output [31:0] dy41_e,
    output [31:0] c1, output [31:0] c2, output [31:0] c3, output [31:0] c4,
    output        t1, output t2, output t3, output t4,

    // depth (invW) plane
    output [31:0] z_ddx, output [31:0] z_ddy, output [31:0] z_c,

    // shared geometry for tsp_setup
    output [31:0] geo_dx12, output [31:0] geo_dx13,
    output [31:0] geo_dy12, output [31:0] geo_dy13,
    output [31:0] geo_carea,
    output [31:0] geo_v1x_l, output [31:0] geo_v1y_t
);
    wire [2:0] depth_mode = isp_tsp[2:0];
    wire [1:0] cull_mode  = isp_tsp[4:3];

    // ---------------- tri_area (refsw_tile.cpp:556) ----------------
    // tri_area = (X1-X3)*(Y2-Y3) - (Y1-Y3)*(X2-X3)
    wire [31:0] x1x3, y2y3, y1y3, x2x3;
    fp_add s_x1x3 (.a(v1x), .b_in(v3x), .sub(1'b1), .y(x1x3));
    fp_add s_y2y3 (.a(v2y), .b_in(v3y), .sub(1'b1), .y(y2y3));
    fp_add s_y1y3 (.a(v1y), .b_in(v3y), .sub(1'b1), .y(y1y3));
    fp_add s_x2x3 (.a(v2x), .b_in(v3x), .sub(1'b1), .y(x2x3));
    wire [31:0] ta_t0, ta_t1, tri_area;
    fp_mul m_ta0 (.a(x1x3), .b(y2y3), .y(ta_t0));
    fp_mul m_ta1 (.a(y1y3), .b(x2x3), .y(ta_t1));
    fp_add a_ta  (.a(ta_t0), .b_in(ta_t1), .sub(1'b1), .y(tri_area));

    // sgn = -1 if tri_area > 0  (positive, non-zero)
    wire ta_is_pos  = (tri_area[31] == 1'b0) && (tri_area[30:0] != 31'd0);
    assign sgn_neg = ta_is_pos;
    wire [31:0] sgn_f = sgn_neg ? 32'hbf800000 /*-1.0*/ : 32'h3f800000 /*+1.0*/;

    // ---------------- cull (refsw_tile.cpp:562) ----------------
    // abs_area = |tri_area|; cull if CullMode!=0 && abs<CULL_VAL,
    // or CullMode>=2 with wrong winding.
    wire [31:0] abs_area = {1'b0, tri_area[30:0]};
    // abs_area < CULL_VAL  (both non-negative floats -> integer compare works)
    wire small_area = (abs_area < FP_FPU_CULL_VAL);
    wire ta_neg = (tri_area[31] == 1'b1) && (tri_area[30:0] != 31'd0);
    wire cm_mode = cull_mode[0];
    wire wrong_wind = (cm_mode == 1'b0 && ta_neg) || (cm_mode == 1'b1 && ta_is_pos);
    assign cull = (cull_mode != 2'd0) &&
                  ( small_area || ((cull_mode >= 2'd2) && wrong_wind) );

    // ---------------- half-edge constants (refsw_tile.cpp:594) ----------------
    // Edge 12 and 23 are identical for tri/quad. Edge 31/41 depend on v4.
    wire [31:0] x1mx2, x2mx3, y1my2, y2my3;
    fp_add e0 (.a(v1x), .b_in(v2x), .sub(1'b1), .y(x1mx2));
    fp_add e1 (.a(v2x), .b_in(v3x), .sub(1'b1), .y(x2mx3));
    fp_add e3 (.a(v1y), .b_in(v2y), .sub(1'b1), .y(y1my2));
    fp_add e4 (.a(v2y), .b_in(v3y), .sub(1'b1), .y(y2my3));

    // Edge "31": tri = X3-X1, quad = X3-X4 ; Edge "41" (quad only) = X4-X1
    wire [31:0] x3mx1, x3mx4, x4mx1, y3my1, y3my4, y4my1;
    fp_add e2  (.a(v3x), .b_in(v1x), .sub(1'b1), .y(x3mx1));
    fp_add e2q (.a(v3x), .b_in(v4x), .sub(1'b1), .y(x3mx4));
    fp_add e6  (.a(v4x), .b_in(v1x), .sub(1'b1), .y(x4mx1));
    fp_add e5  (.a(v3y), .b_in(v1y), .sub(1'b1), .y(y3my1));
    fp_add e5q (.a(v3y), .b_in(v4y), .sub(1'b1), .y(y3my4));
    fp_add e7  (.a(v4y), .b_in(v1y), .sub(1'b1), .y(y4my1));

    wire [31:0] x31_sel = is_quad ? x3mx4 : x3mx1;
    wire [31:0] y31_sel = is_quad ? y3my4 : y3my1;

    // DXnn = sgn * (..)
    fp_mul m_dx12 (.a(sgn_f), .b(x1mx2),  .y(dx12_e));
    fp_mul m_dx23 (.a(sgn_f), .b(x2mx3),  .y(dx23_e));
    fp_mul m_dx31 (.a(sgn_f), .b(x31_sel), .y(dx31_e));
    fp_mul m_dy12 (.a(sgn_f), .b(y1my2),  .y(dy12_e));
    fp_mul m_dy23 (.a(sgn_f), .b(y2my3),  .y(dy23_e));
    fp_mul m_dy31 (.a(sgn_f), .b(y31_sel), .y(dy31_e));
    // DX41/DY41: quad only, else 0
    wire [31:0] dx41_raw, dy41_raw;
    fp_mul m_dx41 (.a(sgn_f), .b(x4mx1), .y(dx41_raw));
    fp_mul m_dy41 (.a(sgn_f), .b(y4my1), .y(dy41_raw));
    assign dx41_e = is_quad ? dx41_raw : 32'h00000000;
    assign dy41_e = is_quad ? dy41_raw : 32'h00000000;

    // Cn = DYnn*(Xn-left) - DXnn*(Yn-top)
    wire [31:0] x1ml, x2ml, x3ml, x4ml, y1mt, y2mt, y3mt, y4mt;
    fp_add cx1 (.a(v1x), .b_in(rect_left), .sub(1'b1), .y(x1ml));
    fp_add cx2 (.a(v2x), .b_in(rect_left), .sub(1'b1), .y(x2ml));
    fp_add cx3 (.a(v3x), .b_in(rect_left), .sub(1'b1), .y(x3ml));
    fp_add cx4 (.a(v4x), .b_in(rect_left), .sub(1'b1), .y(x4ml));
    fp_add cy1 (.a(v1y), .b_in(rect_top),  .sub(1'b1), .y(y1mt));
    fp_add cy2 (.a(v2y), .b_in(rect_top),  .sub(1'b1), .y(y2mt));
    fp_add cy3 (.a(v3y), .b_in(rect_top),  .sub(1'b1), .y(y3mt));
    fp_add cy4 (.a(v4y), .b_in(rect_top),  .sub(1'b1), .y(y4mt));

    wire [31:0] c1a, c1b, c2a, c2b, c3a, c3b, c4a, c4b, c4_raw;
    fp_mul mc1a (.a(dy12_e), .b(x1ml), .y(c1a));
    fp_mul mc1b (.a(dx12_e), .b(y1mt), .y(c1b));
    fp_add ac1  (.a(c1a), .b_in(c1b), .sub(1'b1), .y(c1));
    fp_mul mc2a (.a(dy23_e), .b(x2ml), .y(c2a));
    fp_mul mc2b (.a(dx23_e), .b(y2mt), .y(c2b));
    fp_add ac2  (.a(c2a), .b_in(c2b), .sub(1'b1), .y(c2));
    fp_mul mc3a (.a(dy31_e), .b(x3ml), .y(c3a));
    fp_mul mc3b (.a(dx31_e), .b(y3mt), .y(c3b));
    fp_add ac3  (.a(c3a), .b_in(c3b), .sub(1'b1), .y(c3));
    // C4 = quad ? DY41*(X4-left)-DX41*(Y4-top) : 1.0
    fp_mul mc4a (.a(dy41_e), .b(x4ml), .y(c4a));
    fp_mul mc4b (.a(dx41_e), .b(y4mt), .y(c4b));
    fp_add ac4  (.a(c4a), .b_in(c4b), .sub(1'b1), .y(c4_raw));
    assign c4 = is_quad ? c4_raw : 32'h3f800000;  // 1.0 for triangles

    // ---------------- top-left flags (refsw_tile.cpp:609) ----------------
    // T1 = IsTopLeft(X2-X1, Y2-Y1)  ; IsTopLeft(x,y)= (y==0 && x>0) || (y<0)
    // tri : T3=IsTopLeft(X1-X3,..), T4=true
    // quad: T3=IsTopLeft(X4-X3,..), T4=IsTopLeft(X1-X4,..)
    wire [31:0] x2mx1, y2my1, x3mx2, y3my2, x1mx3, y1my3;
    wire [31:0] x4mx3, y4my3, x1mx4, y1my4;
    fp_add t1x (.a(v2x), .b_in(v1x), .sub(1'b1), .y(x2mx1));
    fp_add t1y (.a(v2y), .b_in(v1y), .sub(1'b1), .y(y2my1));
    fp_add t2x (.a(v3x), .b_in(v2x), .sub(1'b1), .y(x3mx2));
    fp_add t2y (.a(v3y), .b_in(v2y), .sub(1'b1), .y(y3my2));
    fp_add t3x (.a(v1x), .b_in(v3x), .sub(1'b1), .y(x1mx3));
    fp_add t3y (.a(v1y), .b_in(v3y), .sub(1'b1), .y(y1my3));
    fp_add t3xq (.a(v4x), .b_in(v3x), .sub(1'b1), .y(x4mx3));
    fp_add t3yq (.a(v4y), .b_in(v3y), .sub(1'b1), .y(y4my3));
    fp_add t4xq (.a(v1x), .b_in(v4x), .sub(1'b1), .y(x1mx4));
    fp_add t4yq (.a(v1y), .b_in(v4y), .sub(1'b1), .y(y1my4));

    function automatic is_top_left(input [31:0] fx, input [31:0] fy);
        // y==0  : exponent+mantissa all zero (either sign of zero)
        // y<0   : sign bit set and not zero
        // x>0   : sign clear and not zero
        logic y_zero, y_neg, x_pos;
        begin
            y_zero = (fy[30:0] == 31'd0);
            y_neg  = (fy[31] == 1'b1) && (fy[30:0] != 31'd0);
            x_pos  = (fx[31] == 1'b0) && (fx[30:0] != 31'd0);
            is_top_left = (y_zero && x_pos) || y_neg;
        end
    endfunction

    assign t1 = is_top_left(x2mx1, y2my1);
    assign t2 = is_top_left(x3mx2, y3my2);
    assign t3 = is_quad ? is_top_left(x4mx3, y4my3) : is_top_left(x1mx3, y1my3);
    assign t4 = is_quad ? is_top_left(x1mx4, y1my4) : 1'b1;

    // ---------------- shared geometry for the plane solver ----------------
    // PlaneStepper3 deltas: dx12=v2x-v1x, dx13=v3x-v1x, dy12=v2y-v1y, dy13=v3y-v1y
    wire [31:0] g_dx12, g_dx13, g_dy12, g_dy13, g_carea;
    fp_add g0 (.a(v2x), .b_in(v1x), .sub(1'b1), .y(g_dx12));
    fp_add g1 (.a(v3x), .b_in(v1x), .sub(1'b1), .y(g_dx13));
    fp_add g2 (.a(v2y), .b_in(v1y), .sub(1'b1), .y(g_dy12));
    fp_add g3 (.a(v3y), .b_in(v1y), .sub(1'b1), .y(g_dy13));

    // C = dx12*dy13 - dx13*dy12 ; if C==0 -> 1.0
    wire [31:0] carea_t0, carea_t1, carea_raw;
    fp_mul mca0 (.a(g_dx12), .b(g_dy13), .y(carea_t0));
    fp_mul mca1 (.a(g_dx13), .b(g_dy12), .y(carea_t1));
    fp_add aca  (.a(carea_t0), .b_in(carea_t1), .sub(1'b1), .y(carea_raw));
    assign g_carea = (carea_raw[30:0] == 31'd0) ? 32'h3f800000 : carea_raw;

    // v1x_l = v1x - left ; v1y_t = v1y - top  (reuse x1ml/y1mt above)
    assign geo_dx12  = g_dx12;
    assign geo_dx13  = g_dx13;
    assign geo_dy12  = g_dy12;
    assign geo_dy13  = g_dy13;
    assign geo_carea = g_carea;
    assign geo_v1x_l = x1ml;
    assign geo_v1y_t = y1mt;

    // ---------------- depth (invW) plane ----------------
    plane_stepper ps_z (
        .dx12(g_dx12), .dx13(g_dx13), .dy12(g_dy12), .dy13(g_dy13),
        .c_area(g_carea), .v1x_l(x1ml), .v1y_t(y1mt),
        .a1(v1z), .a2(v2z), .a3(v3z),
        .ddx(z_ddx), .ddy(z_ddy), .c(z_c)
    );
endmodule
