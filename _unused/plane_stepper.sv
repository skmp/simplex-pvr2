//
// plane_stepper - one PlaneStepper3::Setup (refsw_tile.h:35), combinational.
//
// Solves the plane equation  f(x,y) = ddx*x + ddy*y + c  that interpolates an
// attribute 'a' (given at the three triangle vertices) across the tile, exactly
// as refsw2 does:
//
//   Aa = (a3-a1)*(v2y-v1y) - (a2-a1)*(v3y-v1y)
//   Ba = (v3x-v1x)*(a2-a1) - (v2x-v1x)*(a3-a1)
//   C  = (v2x-v1x)*(v3y-v1y) - (v3x-v1x)*(v2y-v1y)   ; if C==0, C=1
//   ddx = -Aa/C ;  ddy = -Ba/C
//   c   = a1 - ddx*(v1x-left) - ddy*(v1y-top)
//
// The pure-geometry terms (the four vertex deltas, C, and v1x-left / v1y-top)
// are identical for every attribute of a triangle, so they are computed ONCE in
// the parent (isp_setup / tsp_setup) and fed in here. This module only does the
// attribute-dependent work: the two cross products for Aa/Ba, the divides, and
// the final 'c'. All arithmetic is IEEE-754 single precision.
//
// NOTE: ddx/ddy use a true float divide by C (not reciprocal-multiply) so the
// rounding is bit-identical to the C reference.
//
module plane_stepper (
    // shared geometry (precomputed by parent, same for all attributes)
    input  [31:0] dx12,     // v2.x - v1.x
    input  [31:0] dx13,     // v3.x - v1.x
    input  [31:0] dy12,     // v2.y - v1.y
    input  [31:0] dy13,     // v3.y - v1.y
    input  [31:0] c_area,   // triangle area C (already 1.0 if degenerate)
    input  [31:0] v1x_l,    // v1.x - rect.left
    input  [31:0] v1y_t,    // v1.y - rect.top
    // attribute values at the three vertices
    input  [31:0] a1,
    input  [31:0] a2,
    input  [31:0] a3,
    // plane coefficients out
    output [31:0] ddx,
    output [31:0] ddy,
    output [31:0] c
);
    // da2 = a2 - a1 ;  da3 = a3 - a1
    wire [31:0] da2, da3;
    fp_add sub_da2 (.a(a2), .b_in(a1), .sub(1'b1), .y(da2));
    fp_add sub_da3 (.a(a3), .b_in(a1), .sub(1'b1), .y(da3));

    // Aa = da3*dy12 - da2*dy13
    wire [31:0] aa_t0, aa_t1, aa;
    fp_mul m_aa0 (.a(da3), .b(dy12), .y(aa_t0));
    fp_mul m_aa1 (.a(da2), .b(dy13), .y(aa_t1));
    fp_add a_aa  (.a(aa_t0), .b_in(aa_t1), .sub(1'b1), .y(aa));

    // Ba = dx13*da2 - dx12*da3
    wire [31:0] ba_t0, ba_t1, ba;
    fp_mul m_ba0 (.a(dx13), .b(da2), .y(ba_t0));
    fp_mul m_ba1 (.a(dx12), .b(da3), .y(ba_t1));
    fp_add a_ba  (.a(ba_t0), .b_in(ba_t1), .sub(1'b1), .y(ba));

    // ddx = -Aa/C ; ddy = -Ba/C  (true IEEE divide, matching refsw2 exactly)
    wire [31:0] neg_aa = {~aa[31], aa[30:0]};
    wire [31:0] neg_ba = {~ba[31], ba[30:0]};
    fp_div d_ddx (.a(neg_aa), .b(c_area), .y(ddx));
    fp_div d_ddy (.a(neg_ba), .b(c_area), .y(ddy));

    // c = a1 - ddx*v1x_l - ddy*v1y_t
    wire [31:0] t0, t1, c_tmp;
    fp_mul m_c0 (.a(ddx), .b(v1x_l), .y(t0));
    fp_mul m_c1 (.a(ddy), .b(v1y_t), .y(t1));
    fp_add a_c0 (.a(a1),    .b_in(t0), .sub(1'b1), .y(c_tmp));  // a1 - ddx*v1x_l
    fp_add a_c1 (.a(c_tmp), .b_in(t1), .sub(1'b1), .y(c));      //   - ddy*v1y_t
endmodule
