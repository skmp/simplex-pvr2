//
// simplex_synth_seq - synthesizable top for the SEQUENCED (area-optimized)
//                     setup design. This is the one to compile in Quartus for
//                     the real resource/timing numbers of the one-MAC units.
//
// Wraps tri_setup_seq_top: VRAM comes from a MIF ROM, a 'start' pulse is
// generated once after reset, and every output is XOR-reduced to a 1-bit
// 'digest' so the fitter keeps all the logic without needing many pins.
//
// Unlike the combinational simplex_synth, this exercises the clocked datapath:
// one fmac_seq in ISP, one in TSP, one shared fp_recip.
//
module simplex_synth_seq (
    input      clk,
    input      reset,
    input      is_quad,
    output reg digest
);
    // one-shot start a few cycles after reset
    reg [3:0] boot;
    reg       start;
    always @(posedge clk) begin
        if (reset) begin boot <= 0; start <= 0; end
        else begin
            start <= 0;
            if (boot != 4'hF) boot <= boot + 1'b1;
            if (boot == 4'h4) start <= 1'b1;   // single start pulse
        end
    end

    wire done;
    wire [31:0] isp_tsp, tsp_word, tcw_word;
    wire        sgn_neg, cull, t1,t2,t3,t4;
    wire [31:0] dx12,dx23,dx31,dx41, dy12,dy23,dy31,dy41, c1,c2,c3,c4, z_ddx,z_ddy,z_c;
    wire [31:0] u_ddx,u_ddy,u_c, v_ddx,v_ddy,v_c;
    wire [31:0] col0_ddx,col0_ddy,col0_c, col1_ddx,col1_ddy,col1_c;
    wire [31:0] col2_ddx,col2_ddy,col2_c, col3_ddx,col3_ddy,col3_c;
    wire [31:0] ofs0_ddx,ofs0_ddy,ofs0_c, ofs1_ddx,ofs1_ddy,ofs1_c;
    wire [31:0] ofs2_ddx,ofs2_ddy,ofs2_c, ofs3_ddx,ofs3_ddy,ofs3_c;

    tri_setup_seq_top #(.VRAM_INIT("build/vram.hex")) u_top (
        .clk(clk), .reset(reset), .start(start), .is_quad(is_quad),
        .rect_left(32'h0), .rect_top(32'h0), .done(done),
        .isp_tsp(isp_tsp), .tsp_word(tsp_word), .tcw_word(tcw_word),
        .sgn_neg(sgn_neg), .cull(cull),
        .dx12(dx12),.dx23(dx23),.dx31(dx31),.dx41(dx41),
        .dy12(dy12),.dy23(dy23),.dy31(dy31),.dy41(dy41),
        .c1(c1),.c2(c2),.c3(c3),.c4(c4),.t1(t1),.t2(t2),.t3(t3),.t4(t4),
        .z_ddx(z_ddx),.z_ddy(z_ddy),.z_c(z_c),
        .u_ddx(u_ddx),.u_ddy(u_ddy),.u_c(u_c),.v_ddx(v_ddx),.v_ddy(v_ddy),.v_c(v_c),
        .col0_ddx(col0_ddx),.col0_ddy(col0_ddy),.col0_c(col0_c),
        .col1_ddx(col1_ddx),.col1_ddy(col1_ddy),.col1_c(col1_c),
        .col2_ddx(col2_ddx),.col2_ddy(col2_ddy),.col2_c(col2_c),
        .col3_ddx(col3_ddx),.col3_ddy(col3_ddy),.col3_c(col3_c),
        .ofs0_ddx(ofs0_ddx),.ofs0_ddy(ofs0_ddy),.ofs0_c(ofs0_c),
        .ofs1_ddx(ofs1_ddx),.ofs1_ddy(ofs1_ddy),.ofs1_c(ofs1_c),
        .ofs2_ddx(ofs2_ddx),.ofs2_ddy(ofs2_ddy),.ofs2_c(ofs2_c),
        .ofs3_ddx(ofs3_ddx),.ofs3_ddy(ofs3_ddy),.ofs3_c(ofs3_c)
    );

    wire [31:0] acc =
        dx12^dx23^dx31^dx41^dy12^dy23^dy31^dy41^c1^c2^c3^c4^z_ddx^z_ddy^z_c^
        u_ddx^u_ddy^u_c^v_ddx^v_ddy^v_c^
        col0_ddx^col0_ddy^col0_c^col1_ddx^col1_ddy^col1_c^
        col2_ddx^col2_ddy^col2_c^col3_ddx^col3_ddy^col3_c^
        ofs0_ddx^ofs0_ddy^ofs0_c^ofs1_ddx^ofs1_ddy^ofs1_c^
        ofs2_ddx^ofs2_ddy^ofs2_c^ofs3_ddx^ofs3_ddy^ofs3_c^
        isp_tsp^tsp_word^tcw_word^
        {26'd0,sgn_neg,cull,t1,t2,t3,t4};

    always @(posedge clk) begin
        if (reset) digest <= 1'b0;
        else if (done) digest <= ^acc;
    end
endmodule
