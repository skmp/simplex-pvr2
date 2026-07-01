//
// tsp_shade - per-pixel TSP shading (1 pixel/request, stalls on DDR).
// Orchestrates verified blocks; the texel fetch is delegated to tex_fetch.
//   fp_rcp_fast          : W = 1/invW  (invW from depth buffer)
//   fp_mul_i5/add24/mul16: perspective interpolation of the 10 planes
//   tex_uv2texel         : UV -> 4 corner texel coords + fractions
//   tex_fetch            : per-corner texel -> ARGB (addr/cache/VQ/palette/decode)
//   tex_filter           : nearest / bilinear
//   color_combiner       : texenv (ShadInstr) + offset
// No fog/clamp/blend/bump/mipmap. The two texture caches are INJECTED as bundle
// pairs and passed straight through to tex_fetch.
//
module tsp_shade import tsp_pkg::*; (
    input             clk,
    input             reset,
    input             req,
    input      [4:0]  px,
    input      [4:0]  py,
    output reg        done,
    output reg [31:0] argb,

    input      [31:0] invw_in,               // depth-buffer invW for this pixel
    input      [31:0] p_ddx [0:9],
    input      [31:0] p_ddy [0:9],
    input      [31:0] p_c   [0:9],

    input      [31:0] tsp,
    input      [31:0] tcw,
    input      [4:0]  text_ctrl,
    input             pp_texture,
    input             pp_offset,

    // injected caches (passed through to tex_fetch)
    output cache_req_t   tc_req,
    input  cache_resp_t  tc_resp,
    output cache_req_t   vq_req,
    input  cache_resp_t  vq_resp
);
    wire [2:0] texv=tsp[2:0], texu=tsp[5:3];
    wire [1:0] shadinstr=tsp[7:6], filtmode=tsp[14:13];
    wire clampv=tsp[15],clampu=tsp[16],flipv=tsp[17],flipu=tsp[18],ignoretexa=tsp[19];
    wire bilinear = (filtmode != 2'd0);

    // ---- FP interp units ----
    reg  [31:0] mi_f; reg [4:0] mi_k; wire [31:0] mi_y;
    fp_mul_i5 u_mi (.f(mi_f), .k(mi_k), .y(mi_y));
    reg  [31:0] mm_a, mm_b; wire [31:0] mm_y;
    fp_mul16  u_mm (.a(mm_a), .b(mm_b), .y(mm_y));
    reg  [31:0] ad_a, ad_b; reg ad_s; wire [31:0] ad_y;
    fp_add24  u_ad (.a(ad_a), .b_in(ad_b), .sub(ad_s), .y(ad_y));
    reg        rc_req; reg [31:0] rc_in; wire rc_ack; wire [31:0] rc_y;
    fp_rcp_fast u_rc (.clk(clk),.reset(reset),.in_valid(rc_req),.x(rc_in),
                      .out_valid(rc_ack),.y(rc_y));

    function [7:0] f2u8(input [31:0] f);
        reg [8:0] iv;
        begin
            if (f[31] || f[30:23] < 8'd127) f2u8 = 8'd0;
            else if (f[30:23] >= 8'd135)     f2u8 = 8'd255;
            else begin
                iv = {1'b1, f[22:15]} >> (8'd134 - f[30:23]);
                f2u8 = (iv > 9'd255) ? 8'd255 : iv[7:0];
            end
        end
    endfunction

    reg [31:0] Wp;
    reg [31:0] attr [0:9];
    reg [3:0]  pidx; reg [2:0] istep;
    reg [31:0] t_ddx_x;

    // ---- uv -> texel corners ----
    wire [10:0] c00u,c00v,c01u,c01v,c10u,c10v,c11u,c11v; wire [7:0] ufrac,vfrac;
    tex_uv2texel u_uv (
        .u(attr[0]), .v(attr[1]), .texu(texu), .texv(texv),
        .clampu(clampu),.clampv(clampv),.flipu(flipu),.flipv(flipv),
        .c00u(c00u),.c00v(c00v),.c01u(c01u),.c01v(c01v),
        .c10u(c10u),.c10v(c10v),.c11u(c11u),.c11v(c11v),.ufrac(ufrac),.vfrac(vfrac));

    // ---- texel fetch (per corner) ----
    reg        tf_req; reg [10:0] tf_u, tf_v; wire tf_ack; wire [31:0] tf_argb;
    tex_fetch u_tf (
        .clk(clk),.reset(reset),.req(tf_req),.u(tf_u),.v(tf_v),
        .tsp(tsp),.tcw(tcw),.text_ctrl(text_ctrl),.ack(tf_ack),.argb(tf_argb),
        .tc_req(tc_req),.tc_resp(tc_resp),.vq_req(vq_req),.vq_resp(vq_resp));

    reg [31:0] corner [0:3];
    wire [31:0] t_filt;
    tex_filter u_flt (.filter(bilinear),.ignore_texa(ignoretexa),.ufrac(ufrac),.vfrac(vfrac),
                      .t00(corner[0]),.t01(corner[1]),.t10(corner[2]),.t11(corner[3]),.textel(t_filt));

    reg [31:0] base_col, ofs_col;
    wire [31:0] comb_col;
    color_combiner u_cc (.pp_texture(pp_texture),.pp_offset(pp_offset),.shadinstr(shadinstr),
                         .base(base_col),.textel(t_filt),.offset(ofs_col),.col(comb_col));

    // ---- FSM ----
    localparam S_IDLE=0,S_IW=1,S_INTERP=2,S_TEXSEL=3,S_CSEL=4,S_CFETCH=5,
               S_CNEXT=6,S_COMB=7,S_DONE=8;
    reg [3:0] st;
    reg [1:0] corner_i;

    always @(posedge clk) begin
        if (reset) begin st<=S_IDLE; done<=0; rc_req<=0; tf_req<=0; end
        else begin
            done<=0; rc_req<=0; tf_req<=0;
            case (st)
            S_IDLE: if (req) begin rc_in<=invw_in; rc_req<=1; st<=S_IW; end
            S_IW: if (rc_ack) begin Wp<=rc_y; pidx<=0; istep<=0; st<=S_INTERP; end

            S_INTERP: begin
                case (istep)
                0: begin mi_f<=p_ddx[pidx]; mi_k<=px; istep<=1; end
                1: begin t_ddx_x<=mi_y; mi_f<=p_ddy[pidx]; mi_k<=py; istep<=2; end
                2: begin ad_a<=t_ddx_x; ad_b<=mi_y; ad_s<=0; istep<=3; end
                3: begin ad_a<=ad_y; ad_b<=p_c[pidx]; ad_s<=0; istep<=4; end
                4: begin mm_a<=ad_y; mm_b<=Wp; istep<=5; end
                5: begin attr[pidx]<=mm_y;
                        if (pidx==4'd9) st<=S_TEXSEL;
                        else begin pidx<=pidx+1; istep<=0; end
                    end
                endcase
            end

            S_TEXSEL: begin
                base_col <= {f2u8(attr[5]),f2u8(attr[4]),f2u8(attr[3]),f2u8(attr[2])};
                ofs_col  <= {f2u8(attr[9]),f2u8(attr[8]),f2u8(attr[7]),f2u8(attr[6])};
                if (!pp_texture) st<=S_COMB;
                else begin corner_i<=2'd3; st<=S_CSEL; end   // t11 first (nearest)
            end

            // select corner (u,v) and issue fetch
            S_CSEL: begin
                case (corner_i)
                  2'd0: begin tf_u<=c00u; tf_v<=c00v; end
                  2'd1: begin tf_u<=c01u; tf_v<=c01v; end
                  2'd2: begin tf_u<=c10u; tf_v<=c10v; end
                  2'd3: begin tf_u<=c11u; tf_v<=c11v; end
                endcase
                tf_req<=1; st<=S_CFETCH;
            end
            S_CFETCH: if (tf_ack) begin corner[corner_i]<=tf_argb; st<=S_CNEXT; end
            S_CNEXT: begin
                if (!bilinear)            st<=S_COMB;   // nearest: only t11 needed
                else if (corner_i==2'd2)  st<=S_COMB;   // order 3,0,1,2 done
                else begin
                    corner_i <= (corner_i==2'd3)?2'd0:corner_i+2'd1;
                    st<=S_CSEL;
                end
            end

            S_COMB: begin argb<=comb_col; st<=S_DONE; end
            S_DONE: begin done<=1; st<=S_IDLE; end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
