//
// isp_setup_streamed - SIMPLE full-precision model (mirrors the C++ reference
// do_triangle_setup_pvr in tools/triangle_setup.cpp).
//
// This replaces the 4-way mac16-interleaved version (kept as
// isp_setup_streamed.sv.macbak). The interleave existed only to share the cheap
// 16-bit-mantissa DSPs; now every multiply is a full 24-bit fp_mul24 and every
// add/sub a fp_add24, so we just unroll the whole computation combinationally and
// retire one triangle per accepted input - no MAC scheduling, no per-slot state.
//
// The datapath is a straight transcription of the C++ model: same op order, same
// widened muls (fp_mul24), same fp_add24. Combinational cloud; a 1-deep output
// register makes it a clean streaming stage (accept -> next cycle retire). NOT
// timing-optimised - this is the "for now, I wanna test the numbers" version.
//
// Ports are UNCHANGED from the mac version so the testbench links as-is.
//
module isp_setup_streamed (
    input             clk,
    input             reset,

    input             in_valid,
    output            in_ready,
    input      [31:0] isp_word,
    input      [31:0] in_tag,
    input      [31:0] x1, input [31:0] y1, input [31:0] z1,
    input      [31:0] x2, input [31:0] y2, input [31:0] z2,
    input      [31:0] x3, input [31:0] y3, input [31:0] z3,
    input      [31:0] xbase, input [31:0] ybase,

    output            busy,
    input             out_ready,
    output reg        out_valid,
    output reg [31:0] out_tag,
    output reg [31:0] out_isp,
    output reg        sgn_neg,
    output reg        cull,
    output reg [31:0] dx12, output reg [31:0] dx23, output reg [31:0] dx31, output reg [31:0] dx41,
    output reg [31:0] dy12, output reg [31:0] dy23, output reg [31:0] dy31, output reg [31:0] dy41,
    output reg [31:0] c1,   output reg [31:0] c2,   output reg [31:0] c3,   output reg [31:0] c4,
    output reg [31:0] ddx_invw, output reg [31:0] ddy_invw, output reg [31:0] c_invw,
    output reg [4:0]  bx0, output reg [4:0] bx1, output reg [4:0] by0, output reg [4:0] by1
);
    localparam [31:0] ONE = 32'h3f800000, NEG1 = 32'hbf800000;

    // -------- helper functions (identical to the mac version) --------
    function fzero(input [31:0] f); fzero=(f[30:0]==31'd0); endfunction
    function fneg (input [31:0] f); fneg = f[31]&&(f[30:0]!=31'd0); endfunction
    function fpos (input [31:0] f); fpos = !f[31]&&(f[30:0]!=31'd0); endfunction
    function istl(input [31:0] fdx, input [31:0] fdy);
        istl=(fzero(fdy)&&fpos(fdx))||fneg(fdy); endfunction
    function [31:0] fneg32(input [31:0] f); fneg32={~f[31],f[30:0]}; endfunction

    function automatic signed [15:0] f2i_floor(input [31:0] f);
        integer e, sh; reg [31:0] mag; reg [11:0] sat;
        begin
            e = f[30:23] - 127;
            if (f[30:23] == 8'd0 || e < 0) mag = 0;
            else if (e >= 11) mag = 32'h7FFFFFFF;
            else begin sh = 23 - e; mag = {8'b0, 1'b1, f[22:0]} >> sh; end
            sat = (mag > 32'd2047) ? 12'd2047 : mag[11:0];
            f2i_floor = f[31] ? -{4'b0, sat} : {4'b0, sat};
        end
    endfunction
    function automatic [4:0] clamp5(input signed [15:0] v);
        begin
            if (v < 0) clamp5 = 5'd0; else if (v > 31) clamp5 = 5'd31; else clamp5 = v[4:0];
        end
    endfunction

    // ---------------- combinational multiply / add primitives ----------------
    // Small helper macros via module instances would need many wires; instead use
    // combinational functions? SystemVerilog functions can't instantiate modules,
    // so we lay out explicit fp_mul24 / fp_add24 instances for each op below.

    // The reciprocal (fp_rcp_fast) is the one pipelined unit. We request it on the
    // area and wait its fixed latency before finishing.
    reg        rc_req; reg [31:0] rc_in; wire rc_ack; wire [31:0] rc_y;
    fp_rcp_fast u_rcp (.clk(clk),.reset(reset),.stall(1'b0),.in_valid(rc_req),.x(rc_in),
                       .out_valid(rc_ack),.y(rc_y));

    // ---------------- latched inputs ----------------
    reg [31:0] X1,Y1,Z1,X2,Y2,Z2,X3,Y3,Z3,XB,YB,ISPW,TAG;

    // ================= combinational datapath (mirrors do_triangle_setup_pvr) ==========
    // difference terms
    wire [31:0] d_X1X3, d_Y2Y3, d_Y1Y3, d_X2X3, d_X1X2, d_Y1Y2,
                d_X2X1, d_Y2Y1, d_X3X1, d_Y3Y1, d_Z2Z1, d_Z3Z1;
    fp_add24 s_x1x3(.a(X1),.b_in(X3),.sub(1'b1),.y(d_X1X3));
    fp_add24 s_y2y3(.a(Y2),.b_in(Y3),.sub(1'b1),.y(d_Y2Y3));
    fp_add24 s_y1y3(.a(Y1),.b_in(Y3),.sub(1'b1),.y(d_Y1Y3));
    fp_add24 s_x2x3(.a(X2),.b_in(X3),.sub(1'b1),.y(d_X2X3));
    fp_add24 s_x1x2(.a(X1),.b_in(X2),.sub(1'b1),.y(d_X1X2));
    fp_add24 s_y1y2(.a(Y1),.b_in(Y2),.sub(1'b1),.y(d_Y1Y2));
    fp_add24 s_x2x1(.a(X2),.b_in(X1),.sub(1'b1),.y(d_X2X1));
    fp_add24 s_y2y1(.a(Y2),.b_in(Y1),.sub(1'b1),.y(d_Y2Y1));
    fp_add24 s_x3x1(.a(X3),.b_in(X1),.sub(1'b1),.y(d_X3X1));
    fp_add24 s_y3y1(.a(Y3),.b_in(Y1),.sub(1'b1),.y(d_Y3Y1));
    fp_add24 s_z2z1(.a(Z2),.b_in(Z1),.sub(1'b1),.y(d_Z2Z1));
    fp_add24 s_z3z1(.a(Z3),.b_in(Z1),.sub(1'b1),.y(d_Z3Z1));

    // tri_area = d_X1X3*d_Y2Y3 - d_Y1Y3*d_X2X3
    wire [31:0] pA0, pA1, tri_area;
    fp_mul24 m_pA0(.a(d_X1X3),.b(d_Y2Y3),.y(pA0));
    fp_mul24 m_pA1(.a(d_Y1Y3),.b(d_X2X3),.y(pA1));
    fp_add24 a_area(.a(pA0),.b_in(pA1),.sub(1'b1),.y(tri_area));

    // Aa = -d_Z2Z1*d_Y3Y1 + d_Z3Z1*d_Y2Y1 ; Ba = -d_X2X1*d_Z3Z1 + d_X3X1*d_Z2Z1
    // fpm<32>: 24x24->32-bit WIDE products (fp_mul24_w) summed at 32-bit (fp_add32_w),
    // then packed to float32 (Aa/Ba feed a 24-bit-input mul downstream).
    wire pAa0_s,pAa1_s,Aa_s, pBa0_s,pBa1_s,Ba_s;
    wire [7:0] pAa0_e,pAa1_e,Aa_e, pBa0_e,pBa1_e,Ba_e;
    wire [31:0] pAa0_m,pAa1_m,Aa_m, pBa0_m,pBa1_m,Ba_m;
    wire [31:0] Aa, Ba;
    fp_mul24_w m_aa0(.a(fneg32(d_Z2Z1)),.b(d_Y3Y1),.o_sgn(pAa0_s),.o_exp(pAa0_e),.o_sig(pAa0_m));
    fp_mul24_w m_aa1(.a(d_Z3Z1),.b(d_Y2Y1),.o_sgn(pAa1_s),.o_exp(pAa1_e),.o_sig(pAa1_m));
    fp_add32_w a_aa(.a_sgn(pAa0_s),.a_exp(pAa0_e),.a_sig(pAa0_m),
                    .b_sgn(pAa1_s),.b_exp(pAa1_e),.b_sig(pAa1_m),.sub(1'b0),
                    .o_sgn(Aa_s),.o_exp(Aa_e),.o_sig(Aa_m));
    fp_from_wide p_aa(.sgn(Aa_s),.exp(Aa_e),.sig(Aa_m),.y(Aa));
    fp_mul24_w m_ba0(.a(fneg32(d_X2X1)),.b(d_Z3Z1),.o_sgn(pBa0_s),.o_exp(pBa0_e),.o_sig(pBa0_m));
    fp_mul24_w m_ba1(.a(d_X3X1),.b(d_Z2Z1),.o_sgn(pBa1_s),.o_exp(pBa1_e),.o_sig(pBa1_m));
    fp_add32_w a_ba(.a_sgn(pBa0_s),.a_exp(pBa0_e),.a_sig(pBa0_m),
                    .b_sgn(pBa1_s),.b_exp(pBa1_e),.b_sig(pBa1_m),.sub(1'b0),
                    .o_sgn(Ba_s),.o_exp(Ba_e),.o_sig(Ba_m));
    fp_from_wide p_ba(.sgn(Ba_s),.exp(Ba_e),.sig(Ba_m),.y(Ba));

    // winding sign / cull, from tri_area (registered so it lines up with rc result)
    wire        r_area_pos = fpos(tri_area);
    wire        r_area_neg = fneg(tri_area);
    wire [31:0] sgn = r_area_pos ? NEG1 : ONE;
    wire [1:0]  cm  = ISPW[28:27];
    wire        wrong = (cm[0]==1'b0 && r_area_neg) || (cm[0]==1'b1 && r_area_pos);
    wire        cull_w = (cm >= 2'd2) && wrong;

    // signed edge gradients DX/DY = sgn * delta
    wire [31:0] DX12,DX23,DX31,DY12,DY23,DY31;
    fp_mul24 m_dx12(.a(sgn),.b(d_X1X2),.y(DX12));
    fp_mul24 m_dx23(.a(sgn),.b(d_X2X3),.y(DX23));
    fp_mul24 m_dx31(.a(sgn),.b(d_X3X1),.y(DX31));
    fp_mul24 m_dy12(.a(sgn),.b(d_Y1Y2),.y(DY12));
    fp_mul24 m_dy23(.a(sgn),.b(d_Y2Y3),.y(DY23));
    fp_mul24 m_dy31(.a(sgn),.b(d_Y3Y1),.y(DY31));

    // anchors XL/YT = vertex - tile origin
    wire [31:0] XL1,XL2,XL3,YT1,YT2,YT3;
    fp_add24 s_xl1(.a(X1),.b_in(XB),.sub(1'b1),.y(XL1));
    fp_add24 s_xl2(.a(X2),.b_in(XB),.sub(1'b1),.y(XL2));
    fp_add24 s_xl3(.a(X3),.b_in(XB),.sub(1'b1),.y(XL3));
    fp_add24 s_yt1(.a(Y1),.b_in(YB),.sub(1'b1),.y(YT1));
    fp_add24 s_yt2(.a(Y2),.b_in(YB),.sub(1'b1),.y(YT2));
    fp_add24 s_yt3(.a(Y3),.b_in(YB),.sub(1'b1),.y(YT3));

    // edge constants Craw = DY*XL - DX*YT  (fpm<32>: wide products + wide sub, packed)
    wire c1a_s,c1b_s,C1r_s, c2a_s,c2b_s,C2r_s, c3a_s,c3b_s,C3r_s;
    wire [7:0]  c1a_e,c1b_e,C1r_e, c2a_e,c2b_e,C2r_e, c3a_e,c3b_e,C3r_e;
    wire [31:0] c1a_m,c1b_m,C1r_m, c2a_m,c2b_m,C2r_m, c3a_m,c3b_m,C3r_m;
    wire [31:0] C1raw, C2raw, C3raw;
    fp_mul24_w m_c1a(.a(DY12),.b(XL1),.o_sgn(c1a_s),.o_exp(c1a_e),.o_sig(c1a_m));
    fp_mul24_w m_c1b(.a(DX12),.b(YT1),.o_sgn(c1b_s),.o_exp(c1b_e),.o_sig(c1b_m));
    fp_add32_w a_c1(.a_sgn(c1a_s),.a_exp(c1a_e),.a_sig(c1a_m),
                    .b_sgn(c1b_s),.b_exp(c1b_e),.b_sig(c1b_m),.sub(1'b1),
                    .o_sgn(C1r_s),.o_exp(C1r_e),.o_sig(C1r_m));
    fp_from_wide p_c1(.sgn(C1r_s),.exp(C1r_e),.sig(C1r_m),.y(C1raw));
    fp_mul24_w m_c2a(.a(DY23),.b(XL2),.o_sgn(c2a_s),.o_exp(c2a_e),.o_sig(c2a_m));
    fp_mul24_w m_c2b(.a(DX23),.b(YT2),.o_sgn(c2b_s),.o_exp(c2b_e),.o_sig(c2b_m));
    fp_add32_w a_c2(.a_sgn(c2a_s),.a_exp(c2a_e),.a_sig(c2a_m),
                    .b_sgn(c2b_s),.b_exp(c2b_e),.b_sig(c2b_m),.sub(1'b1),
                    .o_sgn(C2r_s),.o_exp(C2r_e),.o_sig(C2r_m));
    fp_from_wide p_c2(.sgn(C2r_s),.exp(C2r_e),.sig(C2r_m),.y(C2raw));
    fp_mul24_w m_c3a(.a(DY31),.b(XL3),.o_sgn(c3a_s),.o_exp(c3a_e),.o_sig(c3a_m));
    fp_mul24_w m_c3b(.a(DX31),.b(YT3),.o_sgn(c3b_s),.o_exp(c3b_e),.o_sig(c3b_m));
    fp_add32_w a_c3(.a_sgn(c3a_s),.a_exp(c3a_e),.a_sig(c3a_m),
                    .b_sgn(c3b_s),.b_exp(c3b_e),.b_sig(c3b_m),.sub(1'b1),
                    .o_sgn(C3r_s),.o_exp(C3r_e),.o_sig(C3r_m));
    fp_from_wide p_c3(.sgn(C3r_s),.exp(C3r_e),.sig(C3r_m),.y(C3raw));

    // top-left fill rule: raw -1 ULP on non-top-left edges
    wire        tl1 = istl(DX12,DY12), tl2 = istl(DX23,DY23), tl3 = istl(DX31,DY31);
    wire [31:0] C1 = tl1 ? C1raw : (C1raw - 32'd1);
    wire [31:0] C2 = tl2 ? C2raw : (C2raw - 32'd1);
    wire [31:0] C3 = tl3 ? C3raw : (C3raw - 32'd1);

    // invW gradients ddx=-Aa*inv, ddy=-Ba*inv (inv from the reciprocal, latched)
    reg  [31:0] inv_area;
    wire [31:0] pddx, pddy, ddx_w, ddy_w;
    fp_mul24 m_ddx(.a(Aa),.b(inv_area),.y(pddx));
    fp_mul24 m_ddy(.a(Ba),.b(inv_area),.y(pddy));
    assign ddx_w = fneg32(pddx);
    assign ddy_w = fneg32(pddy);

    // invW constant c = Z1 - ddx*XL1 - ddy*YT1  (fpm<32>: wide products+subs, packed)
    //   zc0    = Z1 - ddx*XL1        (Z1 promoted to wide; product wide)
    //   c_invw = zc0 - ddy*YT1
    wire z1_s; wire [7:0] z1_e; wire [31:0] z1_m;
    fp_to_wide w_z1(.f(Z1),.sgn(z1_s),.exp(z1_e),.sig(z1_m));
    wire dx1_s,zc0_s, dy1_s,civ_s;
    wire [7:0]  dx1_e,zc0_e, dy1_e,civ_e;
    wire [31:0] dx1_m,zc0_m, dy1_m,civ_m;
    wire [31:0] cinvw_w;
    fp_mul24_w m_dx1(.a(ddx_w),.b(XL1),.o_sgn(dx1_s),.o_exp(dx1_e),.o_sig(dx1_m));
    fp_add32_w a_zc0(.a_sgn(z1_s),.a_exp(z1_e),.a_sig(z1_m),
                     .b_sgn(dx1_s),.b_exp(dx1_e),.b_sig(dx1_m),.sub(1'b1),
                     .o_sgn(zc0_s),.o_exp(zc0_e),.o_sig(zc0_m));
    fp_mul24_w m_dy1(.a(ddy_w),.b(YT1),.o_sgn(dy1_s),.o_exp(dy1_e),.o_sig(dy1_m));
    fp_add32_w a_civ(.a_sgn(zc0_s),.a_exp(zc0_e),.a_sig(zc0_m),
                     .b_sgn(dy1_s),.b_exp(dy1_e),.b_sig(dy1_m),.sub(1'b1),
                     .o_sgn(civ_s),.o_exp(civ_e),.o_sig(civ_m));
    fp_from_wide p_civ(.sgn(civ_s),.exp(civ_e),.sig(civ_m),.y(cinvw_w));

    // tile-local integer bbox
    wire signed [15:0] lXa = f2i_floor(X1) - f2i_floor(XB);
    wire signed [15:0] lXb = f2i_floor(X2) - f2i_floor(XB);
    wire signed [15:0] lXc = f2i_floor(X3) - f2i_floor(XB);
    wire signed [15:0] lYa = f2i_floor(Y1) - f2i_floor(YB);
    wire signed [15:0] lYb = f2i_floor(Y2) - f2i_floor(YB);
    wire signed [15:0] lYc = f2i_floor(Y3) - f2i_floor(YB);
    wire signed [15:0] bxmin = (lXa<lXb?(lXa<lXc?lXa:lXc):(lXb<lXc?lXb:lXc));
    wire signed [15:0] bxmax = (lXa>lXb?(lXa>lXc?lXa:lXc):(lXb>lXc?lXb:lXc));
    wire signed [15:0] bymin = (lYa<lYb?(lYa<lYc?lYa:lYc):(lYb<lYc?lYb:lYc));
    wire signed [15:0] bymax = (lYa>lYb?(lYa>lYc?lYa:lYc):(lYb>lYc?lYb:lYc));

    // ---------------- control FSM ----------------
    // S_IDLE : accept a triangle, latch inputs, request reciprocal on the area.
    //          (tri_area is combinational off the just-latched inputs -> next cycle
    //           it's stable; we request the rcp the cycle after latching.)
    // S_AREA : area/rcp in flight; wait rc_ack, latch inv_area.
    // S_EMIT : results combinational off inv_area; retire when out_ready.
    localparam S_IDLE=2'd0, S_RCP=2'd1, S_AREA=2'd2, S_EMIT=2'd3;
    reg [1:0] st;
    assign in_ready = (st == S_IDLE);
    assign busy     = (st != S_IDLE);

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; out_valid <= 1'b0; rc_req <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            rc_req    <= 1'b0;
            case (st)
            S_IDLE: if (in_valid) begin
                X1<=x1;Y1<=y1;Z1<=z1; X2<=x2;Y2<=y2;Z2<=z2; X3<=x3;Y3<=y3;Z3<=z3;
                XB<=xbase; YB<=ybase; ISPW<=isp_word; TAG<=in_tag;
                st <= S_RCP;
            end
            // inputs are latched; tri_area is now stable -> request reciprocal.
            S_RCP: begin
                rc_in  <= tri_area;
                rc_req <= 1'b1;
                st     <= S_AREA;
            end
            S_AREA: if (rc_ack) begin
                inv_area <= rc_y;
                st       <= S_EMIT;
            end
            S_EMIT: if (out_ready) begin
                out_valid <= 1'b1;
                out_tag   <= TAG;
                out_isp   <= ISPW;
                sgn_neg   <= r_area_pos;
                cull      <= cull_w;
                dx12<=DX12; dx23<=DX23; dx31<=DX31; dx41<=32'd0;
                dy12<=DY12; dy23<=DY23; dy31<=DY31; dy41<=32'd0;
                c1<=C1; c2<=C2; c3<=C3; c4<=ONE;
                ddx_invw<=ddx_w; ddy_invw<=ddy_w; c_invw<=cinvw_w;
                bx0<=clamp5(bxmin);      bx1<=clamp5(bxmax+16'sd1);
                by0<=clamp5(bymin);      by1<=clamp5(bymax+16'sd1);
                st  <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
