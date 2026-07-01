//
// tex_filter - refsw TextureFilter blend, combinational + testable.
// Point (filter 0) -> t11. Bilinear (filter 1) -> weighted blend of 4 texels.
// Colours packed {A,R,G,B}. Blend weights from the sub-texel fraction:
//   ublend = u8_256(ui&255) ; vblend = vi&255 ; nublend=256-ub ; nvblend=256-vb
//   out[i] = (t00*ub*vb + t01*nub*vb + t10*ub*nvb + t11*nub*nvb) / 65536
// t00=(u+1,v+1) t01=(u+0,v+1) t10=(u+1,v+0) t11=(u+0,v+0)  (per refsw offsets)
// If ignore_texa: out.a = 255.
//
module tex_filter (
    input             filter,       // 0=point, 1=bilinear
    input             ignore_texa,
    input      [7:0]  ufrac,        // ui & 255
    input      [7:0]  vfrac,        // vi & 255
    input      [31:0] t00,t01,t10,t11,
    output     [31:0] textel
);
    function [7:0] ch(input [31:0] c, input [1:0] i); ch = c[8*i +: 8]; endfunction
    wire [8:0] ub  = {1'b0,ufrac} + {8'b0,ufrac[7]};   // u8_256
    wire [8:0] vb  = {1'b0,vfrac};                       // refsw uses raw vi&255
    wire [9:0] nub = 10'd256 - ub;
    wire [9:0] nvb = 10'd256 - {1'b0,vb};

    function [7:0] blend(input [1:0] i);
        reg [31:0] s;
        begin
            s = ch(t00,i)*ub *vb
              + ch(t01,i)*nub*vb
              + ch(t10,i)*ub *nvb
              + ch(t11,i)*nub*nvb;
            blend = s[23:16];    // /65536
        end
    endfunction

    wire [7:0] bB = blend(2'd0), bG = blend(2'd1), bR = blend(2'd2), bA = blend(2'd3);
    wire [31:0] bilin = {bA,bR,bG,bB};
    wire [31:0] pre   = filter ? bilin : t11;
    assign textel = ignore_texa ? {8'hFF, pre[23:0]} : pre;
endmodule
