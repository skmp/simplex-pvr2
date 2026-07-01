//
// color_combiner - refsw ColorCombiner (texenv), combinational + testable.
// Combines base, textel, offset per ShadInstr. No bump. Offset is added
// (saturating) to rgb when pp_offset. Colours are packed {A,R,G,B} bytes.
//
//   ShadInstr 0: rv = textel
//   ShadInstr 1: rv.rgb = textel.rgb * u8_256(base.rgb)/256 ; rv.a = textel.a
//   ShadInstr 2: rv.rgb = mix(base.rgb, textel.rgb, textel.a) ; rv.a = base.a
//   ShadInstr 3: rv = textel * u8_256(base)/256  (all 4)
//   +offset: rv.rgb = min(rv.rgb + offset.rgb, 255)
//   !pp_texture: rv = base
//
module color_combiner (
    input             pp_texture,
    input             pp_offset,
    input      [1:0]  shadinstr,
    input      [31:0] base,      // {A,R,G,B}
    input      [31:0] textel,
    input      [31:0] offset,
    output     [31:0] col
);
    // channel accessors (i: 0=B[7:0] 1=G[15:8] 2=R[23:16] 3=A[31:24])
    function [7:0] ch(input [31:0] c, input [1:0] i);
        ch = c[8*i +: 8]; endfunction
    // u8_256(v) = v + (v>>7)
    function [8:0] u8_256(input [7:0] v);
        u8_256 = {1'b0,v} + {8'b0, v[7]}; endfunction
    // a*b/256 with a in 0..255, b = u8_256 (0..256)
    function [7:0] mul256(input [7:0] a, input [8:0] b);
        reg [16:0] p; begin p = a * b; mul256 = p[15:8]; end endfunction
    function [7:0] sat_add(input [7:0] a, input [7:0] b);
        reg [8:0] s; begin s = {1'b0,a}+{1'b0,b}; sat_add = s[8]?8'hFF:s[7:0]; end endfunction
    // mix(base, tex, texA): (tex*tb + base*(256-tb))/256, tb=u8_256(texA)
    function [7:0] mix(input [7:0] texc, input [7:0] basec, input [8:0] tb);
        reg [17:0] p; begin
            p = texc*tb + basec*(9'd256 - tb);
            mix = p[15:8];
        end
    endfunction

    // per-channel combine
    function [7:0] comb_ch(input [1:0] i);
        reg [7:0] t,b;
        begin
            t = ch(textel,i); b = ch(base,i);
            case (shadinstr)
              2'd0: comb_ch = t;                                    // replace
              2'd1: comb_ch = (i==2'd3) ? t : mul256(t, u8_256(b)); // modulate rgb, a=tex
              2'd2: comb_ch = (i==2'd3) ? b                         // mix, a=base
                            : mix(t, b, u8_256(ch(textel,2'd3)));
              2'd3: comb_ch = mul256(t, u8_256(b));                 // modulate all
            endcase
        end
    endfunction

    wire [7:0] cB = comb_ch(2'd0), cG = comb_ch(2'd1), cR = comb_ch(2'd2), cA = comb_ch(2'd3);
    // offset add to rgb only
    wire [7:0] oB = pp_offset ? sat_add(cB, ch(offset,2'd0)) : cB;
    wire [7:0] oG = pp_offset ? sat_add(cG, ch(offset,2'd1)) : cG;
    wire [7:0] oR = pp_offset ? sat_add(cR, ch(offset,2'd2)) : cR;

    wire [31:0] tex_col = {cA, oR, oG, oB};
    assign col = pp_texture ? tex_col : base;
endmodule
