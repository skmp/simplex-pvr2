//
// isp_raster_line - evaluate one full 32-pixel scanline of a triangle in a
// single clock (combinational), for opaque render mode.
//
// For tile-local pixel (x,y), x,y in 0..31 (pixel-center ignored, integer
// coords used directly), per refsw:
//    Xhs_n(x) = Cn + DXn*y - DYn*x        (n = 12,23,31,41)
//    inside_mask   = Xhs12>=0 && Xhs23>=0 && Xhs31>=0 && Xhs41>=0
//    invW(x)  = c_invw + ddx*x + ddy*y
//
// Numerics per spec: DX/DY/ddx/ddy and Cn/c_invw are the reduced setup format;
// the *pixel-index* products use the fast 16-bit x 5-bit multiplier (fp_mul_i5)
// since x,y are 5-bit; sums use the higher-precision fp_add24, packed to fp32.
//
// One clock: line base (Cn+DXn*y, c+ddy*y) feeds a 32-wide parallel array of
// (ebase - DYn*x) and (wbase + ddx*x). Outputs: 32 'inside_mask' bits + 32 invW.
//
module isp_raster_line (
    input      [4:0]  y,          // current line (0..31)

    input      [31:0] c1,c2,c3,c4,
    input      [31:0] dx12,dx23,dx31,dx41,
    input      [31:0] dy12,dy23,dy31,dy41,
    input      [31:0] ddx,ddy,c_invw,

    output     [31:0] inside_mask,     // bit x = pixel x is inside_mask the triangle
    output     [31:0] invw_0,  output [31:0] invw_1,  output [31:0] invw_2,  output [31:0] invw_3,
    output     [31:0] invw_4,  output [31:0] invw_5,  output [31:0] invw_6,  output [31:0] invw_7,
    output     [31:0] invw_8,  output [31:0] invw_9,  output [31:0] invw_10, output [31:0] invw_11,
    output     [31:0] invw_12, output [31:0] invw_13, output [31:0] invw_14, output [31:0] invw_15,
    output     [31:0] invw_16, output [31:0] invw_17, output [31:0] invw_18, output [31:0] invw_19,
    output     [31:0] invw_20, output [31:0] invw_21, output [31:0] invw_22, output [31:0] invw_23,
    output     [31:0] invw_24, output [31:0] invw_25, output [31:0] invw_26, output [31:0] invw_27,
    output     [31:0] invw_28, output [31:0] invw_29, output [31:0] invw_30, output [31:0] invw_31
);
    function fpos_or_zero(input [31:0] f); // f >= 0 : sign bit clear (incl +0)
        fpos_or_zero = ~f[31]; endfunction

    // ---- per-line base: ebase_n = Cn + DXn*y ; wbase = c_invw + ddy*y ----
    wire [31:0] dx12y,dx23y,dx31y,dx41y, ddyy;
    fp_mul_i5 m_dx12y(.f(dx12),.k(y),.y(dx12y));
    fp_mul_i5 m_dx23y(.f(dx23),.k(y),.y(dx23y));
    fp_mul_i5 m_dx31y(.f(dx31),.k(y),.y(dx31y));
    fp_mul_i5 m_dx41y(.f(dx41),.k(y),.y(dx41y));
    fp_mul_i5 m_ddyy (.f(ddy), .k(y),.y(ddyy));

    wire [31:0] eb1,eb2,eb3,eb4, wbase;
    fp_add24 a_eb1(.a(c1),.b_in(dx12y),.sub(1'b0),.y(eb1));
    fp_add24 a_eb2(.a(c2),.b_in(dx23y),.sub(1'b0),.y(eb2));
    fp_add24 a_eb3(.a(c3),.b_in(dx31y),.sub(1'b0),.y(eb3));
    fp_add24 a_eb4(.a(c4),.b_in(dx41y),.sub(1'b0),.y(eb4));
    fp_add24 a_wb (.a(c_invw),.b_in(ddyy),.sub(1'b0),.y(wbase));

    wire [31:0] iw [0:31];
    genvar gx;
    generate
      for (gx = 0; gx < 32; gx = gx + 1) begin : px
        // Xhs_n(x) = ebase_n - DYn*x
        wire [31:0] dy12x,dy23x,dy31x,dy41x, ddxx;
        fp_mul_i5 mdy12(.f(dy12),.k(gx[4:0]),.y(dy12x));
        fp_mul_i5 mdy23(.f(dy23),.k(gx[4:0]),.y(dy23x));
        fp_mul_i5 mdy31(.f(dy31),.k(gx[4:0]),.y(dy31x));
        fp_mul_i5 mdy41(.f(dy41),.k(gx[4:0]),.y(dy41x));
        fp_mul_i5 mddx (.f(ddx), .k(gx[4:0]),.y(ddxx));

        wire [31:0] xh1,xh2,xh3,xh4;
        fp_add24 axh1(.a(eb1),.b_in(dy12x),.sub(1'b1),.y(xh1));
        fp_add24 axh2(.a(eb2),.b_in(dy23x),.sub(1'b1),.y(xh2));
        fp_add24 axh3(.a(eb3),.b_in(dy31x),.sub(1'b1),.y(xh3));
        fp_add24 axh4(.a(eb4),.b_in(dy41x),.sub(1'b1),.y(xh4));

        assign inside_mask[gx] = fpos_or_zero(xh1) & fpos_or_zero(xh2)
                          & fpos_or_zero(xh3) & fpos_or_zero(xh4);

        // invW(x) = wbase + ddx*x
        fp_add24 aiw(.a(wbase),.b_in(ddxx),.sub(1'b0),.y(iw[gx]));
      end
    endgenerate

    assign invw_0=iw[0];   assign invw_1=iw[1];   assign invw_2=iw[2];   assign invw_3=iw[3];
    assign invw_4=iw[4];   assign invw_5=iw[5];   assign invw_6=iw[6];   assign invw_7=iw[7];
    assign invw_8=iw[8];   assign invw_9=iw[9];   assign invw_10=iw[10]; assign invw_11=iw[11];
    assign invw_12=iw[12]; assign invw_13=iw[13]; assign invw_14=iw[14]; assign invw_15=iw[15];
    assign invw_16=iw[16]; assign invw_17=iw[17]; assign invw_18=iw[18]; assign invw_19=iw[19];
    assign invw_20=iw[20]; assign invw_21=iw[21]; assign invw_22=iw[22]; assign invw_23=iw[23];
    assign invw_24=iw[24]; assign invw_25=iw[25]; assign invw_26=iw[26]; assign invw_27=iw[27];
    assign invw_28=iw[28]; assign invw_29=iw[29]; assign invw_30=iw[30]; assign invw_31=iw[31];
endmodule
