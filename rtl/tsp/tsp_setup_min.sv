//
// tsp_setup_min - SIMPLE full-precision model (mirrors the C++ reference
// do_triangle_setup_pvr's TSP section in tools/triangle_setup.cpp).
//
// Replaces the mac16-interleaved version (kept as tsp_setup_min.sv.macbak). Every
// multiply is a full 24-bit fp_mul24, the perspective-numerator accumulations run
// through the 32-bit WIDE datapath (fp_mul24_w / fp_add32_w / fp_add3_32_w in
// rtl/isp_min/fp_wide.sv), matching the fpm<32> chains in the C++ model. No MAC
// scheduling: one attribute plane is computed combinationally per FSM step and
// streamed out on plane_valid, in index order 0..9 (disabled planes skipped).
//
// Plane math (per plane, attribute a1/a2/a3 per vertex):
//   za_i = z_i * a_i                                   (fp_mul24)
//   da2 = za2-za1 ; da3 = za3-za1                      (fp_add24)
//   Aa = da3*(Y2-Y1) - da2*(Y3-Y1)  [wide]             (fp_mul24_w + fp_add32_w)
//   Ba = (X3-X1)*da2 - (X2-X1)*da3  [wide]
//   ddx = -Aa/area ; ddy = -Ba/area                    (fp_mul24, area recip)
//   c = za1 - ddx*(X1-XB) - ddy*(Y1-YB)  [wide 3-add]  (fp_add3_32_w)
//
// Ports UNCHANGED from the mac version so the consumer / TB link as-is.
//
module tsp_setup_min (
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
    localparam [31:0] ZERO = 32'd0;
    function [31:0] fneg32(input [31:0] f); fneg32={~f[31],f[30:0]}; endfunction

    // ---------------- latched inputs ----------------
    reg [31:0] X1,Y1,Z1,X2,Y2,Z2,X3,Y3,Z3,XB,YB;
    reg [31:0] U1,V1,U2,V2,U3,V3, COL1,COL2,COL3, OFS1,OFS2,OFS3;
    reg        g_r, tex_r, ofs_r;

    // ---------------- geometry (combinational off latched inputs) ----------------
    wire [31:0] Y2Y1, Y3Y1, X3X1, X2X1, area, XL1, YT1;
    fp_add24 s_y2y1(.a(Y2),.b_in(Y1),.sub(1'b1),.y(Y2Y1));
    fp_add24 s_y3y1(.a(Y3),.b_in(Y1),.sub(1'b1),.y(Y3Y1));
    fp_add24 s_x3x1(.a(X3),.b_in(X1),.sub(1'b1),.y(X3X1));
    fp_add24 s_x2x1(.a(X2),.b_in(X1),.sub(1'b1),.y(X2X1));
    wire [31:0] pAr0, pAr1;
    fp_mul24 m_ar0(.a(X2X1),.b(Y3Y1),.y(pAr0));
    fp_mul24 m_ar1(.a(X3X1),.b(Y2Y1),.y(pAr1));
    fp_add24 a_area(.a(pAr0),.b_in(pAr1),.sub(1'b1),.y(area));
    fp_add24 s_xl1(.a(X1),.b_in(XB),.sub(1'b1),.y(XL1));
    fp_add24 s_yt1(.a(Y1),.b_in(YB),.sub(1'b1),.y(YT1));

    // reciprocal of the area (fp_rcp_fast, ~16-bit) - requested once, latched.
    reg        rc_req; reg [31:0] rc_in; wire rc_ack; wire [31:0] rc_y;
    reg [31:0] inv_area;
    fp_rcp_fast u_rcp (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(rc_req),.x(rc_in),
                       .out_valid(rc_ack),.y(rc_y));

    // ================= per-plane datapath (combinational off cur attr a1/a2/a3) ========
    // cur_idx selects which plane we're computing; the a1/a2/a3 float operands are
    // chosen below. za, da, Aa/Ba (wide), ddx/ddy, and c are all combinational.
    reg  [3:0]  cur_idx;

    // colour/offset channel -> float (u8_to_float); UV planes use the raw floats.
    // channel byte index within the packed word for plane idx 2..5 / 6..9:
    //   idx 2/6 -> R (bits 16..23), 3/7 -> G (8..15), 4/8 -> B (0..7), 5/9 -> A (24..31)
    // plane_idx maps 2=R,3=G,4=B,5=A (offset 6=R..9=A); chn = idx-2, so
    //   chn 0->R(16..23) 1->G(8..15) 2->B(0..7) 3->A(24..31).
    // Flat (non-gouraud) uses v3's colour for all three vertices.
    reg        is_uv;
    reg [31:0] uva1, uva2, uva3;      // UV float operands when is_uv
    reg [7:0]  cb1, cb2, cb3;         // colour/offset channel bytes otherwise
    reg [31:0] c1r, c2r, c3r; reg [1:0] chn;
    always @(*) begin
        is_uv=1'b0; uva1=ZERO; uva2=ZERO; uva3=ZERO;
        cb1=8'd0; cb2=8'd0; cb3=8'd0;
        c1r=32'd0; c2r=32'd0; c3r=32'd0; chn=2'd0;
        if (cur_idx == 4'd0) begin is_uv=1'b1; uva1=U1; uva2=U2; uva3=U3; end
        else if (cur_idx == 4'd1) begin is_uv=1'b1; uva1=V1; uva2=V2; uva3=V3; end
        else begin
            if (cur_idx <= 4'd5) begin c1r=g_r?COL1:COL3; c2r=g_r?COL2:COL3; c3r=COL3; end
            else                 begin c1r=g_r?OFS1:OFS3; c2r=g_r?OFS2:OFS3; c3r=OFS3; end
            chn = cur_idx[1:0] - 2'd2;
            case (chn)
              2'd0: begin cb1=c1r[23:16]; cb2=c2r[23:16]; cb3=c3r[23:16]; end // R
              2'd1: begin cb1=c1r[15:8];  cb2=c2r[15:8];  cb3=c3r[15:8];  end // G
              2'd2: begin cb1=c1r[7:0];   cb2=c2r[7:0];   cb3=c3r[7:0];   end // B
              2'd3: begin cb1=c1r[31:24]; cb2=c2r[31:24]; cb3=c3r[31:24]; end // A
            endcase
        end
    end
    wire [31:0] cf1, cf2, cf3;
    u8_to_float uc1(.u(cb1),.f(cf1));
    u8_to_float uc2(.u(cb2),.f(cf2));
    u8_to_float uc3(.u(cb3),.f(cf3));
    wire [31:0] a1 = is_uv ? uva1 : cf1;
    wire [31:0] a2 = is_uv ? uva2 : cf2;
    wire [31:0] a3 = is_uv ? uva3 : cf3;

    // za_i = z_i * a_i
    wire [31:0] za1, za2, za3;
    fp_mul24 m_za1(.a(Z1),.b(a1),.y(za1));
    fp_mul24 m_za2(.a(Z2),.b(a2),.y(za2));
    fp_mul24 m_za3(.a(Z3),.b(a3),.y(za3));
    // da2 = za2-za1 ; da3 = za3-za1
    wire [31:0] da2, da3;
    fp_add24 s_da2(.a(za2),.b_in(za1),.sub(1'b1),.y(da2));
    fp_add24 s_da3(.a(za3),.b_in(za1),.sub(1'b1),.y(da3));

    // Aa = da3*Y2Y1 - da2*Y3Y1  (WIDE) ; Ba = X3X1*da2 - X2X1*da3  (WIDE)
    wire aa0_s,aa1_s,Aa_s, ba0_s,ba1_s,Ba_s;
    wire [7:0]  aa0_e,aa1_e,Aa_e, ba0_e,ba1_e,Ba_e;
    wire [31:0] aa0_m,aa1_m,Aa_m, ba0_m,ba1_m,Ba_m;
    wire [31:0] Aa, Ba;
    fp_mul24_w mw_aa0(.a(da3),.b(Y2Y1),.o_sgn(aa0_s),.o_exp(aa0_e),.o_sig(aa0_m));
    fp_mul24_w mw_aa1(.a(da2),.b(Y3Y1),.o_sgn(aa1_s),.o_exp(aa1_e),.o_sig(aa1_m));
    fp_add32_w aw_aa(.a_sgn(aa0_s),.a_exp(aa0_e),.a_sig(aa0_m),
                     .b_sgn(aa1_s),.b_exp(aa1_e),.b_sig(aa1_m),.sub(1'b1),
                     .o_sgn(Aa_s),.o_exp(Aa_e),.o_sig(Aa_m));
    fp_from_wide pw_aa(.sgn(Aa_s),.exp(Aa_e),.sig(Aa_m),.y(Aa));
    fp_mul24_w mw_ba0(.a(X3X1),.b(da2),.o_sgn(ba0_s),.o_exp(ba0_e),.o_sig(ba0_m));
    fp_mul24_w mw_ba1(.a(X2X1),.b(da3),.o_sgn(ba1_s),.o_exp(ba1_e),.o_sig(ba1_m));
    fp_add32_w aw_ba(.a_sgn(ba0_s),.a_exp(ba0_e),.a_sig(ba0_m),
                     .b_sgn(ba1_s),.b_exp(ba1_e),.b_sig(ba1_m),.sub(1'b1),
                     .o_sgn(Ba_s),.o_exp(Ba_e),.o_sig(Ba_m));
    fp_from_wide pw_ba(.sgn(Ba_s),.exp(Ba_e),.sig(Ba_m),.y(Ba));

    // ddx = -Aa*inv ; ddy = -Ba*inv
    wire [31:0] pddx, pddy, ddx_w, ddy_w;
    fp_mul24 m_ddx(.a(Aa),.b(inv_area),.y(pddx));
    fp_mul24 m_ddy(.a(Ba),.b(inv_area),.y(pddy));
    assign ddx_w = fneg32(pddx);
    assign ddy_w = fneg32(pddy);

    // c = za1 - ddx*XL1 - ddy*YT1  (WIDE 3-input: za1 + (-p_dx) + (-p_dy))
    wire dx_s, dy_s; wire [7:0] dx_e, dy_e; wire [31:0] dx_m, dy_m;
    fp_mul24_w mw_cdx(.a(ddx_w),.b(XL1),.o_sgn(dx_s),.o_exp(dx_e),.o_sig(dx_m));
    fp_mul24_w mw_cdy(.a(ddy_w),.b(YT1),.o_sgn(dy_s),.o_exp(dy_e),.o_sig(dy_m));
    wire za1_s; wire [7:0] za1_e; wire [31:0] za1_m;
    fp_to_wide w_za1(.f(za1),.sgn(za1_s),.exp(za1_e),.sig(za1_m));
    wire c_s; wire [7:0] c_e; wire [31:0] c_m; wire [31:0] c_w;
    // c = za1 - dx - dy : negate the two products' signs, 3-input add.
    fp_add3_32_w aw_c(.a_sgn(za1_s),     .a_exp(za1_e),.a_sig(za1_m),
                      .b_sgn(~dx_s),     .b_exp(dx_e), .b_sig(dx_m),
                      .c_sgn(~dy_s),     .c_exp(dy_e), .c_sig(dy_m),
                      .o_sgn(c_s),.o_exp(c_e),.o_sig(c_m));
    fp_from_wide pw_c(.sgn(c_s),.exp(c_e),.sig(c_m),.y(c_w));

    // ---------------- plane-enable ----------------
    function plane_en(input [3:0] i);
        plane_en = (i<=4'd1) ? tex_r : (i<=4'd5) ? 1'b1 : ofs_r; endfunction

    // ---------------- FSM ----------------
    // LOAD -> latch inputs, request reciprocal on the area.
    // WAIT -> hold for rc_ack, latch inv_area, start plane 0.
    // EMIT -> for each enabled plane, pulse plane_valid with its ddx/ddy/c; advance.
    localparam S_IDLE=2'd0, S_WAIT=2'd1, S_EMIT=2'd2;
    reg [1:0] st;

    always @(posedge clk) begin
        if (reset) begin
            st<=S_IDLE; done<=1'b0; plane_valid<=1'b0; rc_req<=1'b0; cur_idx<=4'd0;
        end else begin
            done        <= 1'b0;
            plane_valid <= 1'b0;
            rc_req      <= 1'b0;
            case (st)
            S_IDLE: if (start) begin
                X1<=x1;Y1<=y1;Z1<=z1; X2<=x2;Y2<=y2;Z2<=z2; X3<=x3;Y3<=y3;Z3<=z3;
                XB<=xbase; YB<=ybase;
                U1<=u1;V1<=v1;U2<=u2;V2<=v2;U3<=u3;V3<=v3;
                COL1<=col1;COL2<=col2;COL3<=col3; OFS1<=ofs1;OFS2<=ofs2;OFS3<=ofs3;
                g_r<=gouraud; tex_r<=texture; ofs_r<=offset;
                st <= S_WAIT;
                // area is combinational off the just-latched inputs; request the
                // reciprocal next cycle (S_WAIT enters with area stable).
            end
            S_WAIT: begin
                if (!rc_req && !rc_ack_seen) begin rc_in<=area; rc_req<=1'b1; end
                if (rc_ack) begin
                    inv_area <= rc_y;
                    cur_idx  <= 4'd0;
                    st       <= S_EMIT;
                end
            end
            S_EMIT: begin
                if (plane_en(cur_idx)) begin
                    // results are combinational off cur_idx + inv_area -> emit now.
                    plane_valid <= 1'b1;
                    plane_idx   <= cur_idx;
                    o_ddx       <= ddx_w;
                    o_ddy       <= ddy_w;
                    o_c         <= c_w;
                end
                if (cur_idx == 4'd9) begin done <= 1'b1; st <= S_IDLE; end
                else cur_idx <= cur_idx + 4'd1;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    // track whether we've already fired the reciprocal request this triangle
    reg rc_ack_seen;
    always @(posedge clk) begin
        if (reset) rc_ack_seen <= 1'b0;
        else if (st == S_IDLE) rc_ack_seen <= 1'b0;
        else if (rc_req)       rc_ack_seen <= 1'b1;
    end
endmodule
