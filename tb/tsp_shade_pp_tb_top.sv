// tsp_shade_pp_tb_top - drive both the serial tsp_shade and the pipelined
// tsp_shade_pp with the SAME per-pixel inputs + the SAME behavioral VRAM, and
// expose both ARGB outputs so the C++ TB can check bit-equivalence.
//
// Serial ref: one pixel at a time (req/done), 2 tex_caches over VRAM.
// Pipelined DUT: stream pixels in (in_valid/stall), 4 corner tex_cache pairs.
// All caches read the same behavioral VRAM (64-bit view).
//
module tsp_shade_pp_tb_top import tsp_pkg::*; (
    input             clk,
    input             reset,

    // shared per-pixel inputs
    input      [4:0]  px,
    input      [4:0]  py,
    input      [31:0] invw_in,
    input      [31:0] ddx0,ddx1,ddx2,ddx3,ddx4,ddx5,ddx6,ddx7,ddx8,ddx9,
    input      [31:0] ddy0,ddy1,ddy2,ddy3,ddy4,ddy5,ddy6,ddy7,ddy8,ddy9,
    input      [31:0] c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,
    input      [31:0] tsp,
    input      [31:0] tcw,
    input      [4:0]  text_ctrl,
    input             pp_texture,
    input             pp_offset,

    // serial reference control
    input             ref_req,
    output            ref_done,
    output     [31:0] ref_argb,

    // pipelined dut control
    input             pp_in_valid,
    input      [9:0]  pp_in_id,
    output            pp_stall,
    output            pp_out_valid,
    output     [9:0]  pp_out_id,
    output     [31:0] pp_out_argb
);
    // pack the flat plane inputs into arrays
    wire [31:0] a_ddx [0:9]; wire [31:0] a_ddy [0:9]; wire [31:0] a_c [0:9];
    assign a_ddx[0]=ddx0;assign a_ddx[1]=ddx1;assign a_ddx[2]=ddx2;assign a_ddx[3]=ddx3;assign a_ddx[4]=ddx4;
    assign a_ddx[5]=ddx5;assign a_ddx[6]=ddx6;assign a_ddx[7]=ddx7;assign a_ddx[8]=ddx8;assign a_ddx[9]=ddx9;
    assign a_ddy[0]=ddy0;assign a_ddy[1]=ddy1;assign a_ddy[2]=ddy2;assign a_ddy[3]=ddy3;assign a_ddy[4]=ddy4;
    assign a_ddy[5]=ddy5;assign a_ddy[6]=ddy6;assign a_ddy[7]=ddy7;assign a_ddy[8]=ddy8;assign a_ddy[9]=ddy9;
    assign a_c[0]=c0;assign a_c[1]=c1;assign a_c[2]=c2;assign a_c[3]=c3;assign a_c[4]=c4;
    assign a_c[5]=c5;assign a_c[6]=c6;assign a_c[7]=c7;assign a_c[8]=c8;assign a_c[9]=c9;

    // ---- behavioral VRAM shared by all caches (64-bit physical view) ----
    (* verilator public_flat_rw *) reg [63:0] vram [0:1048575];

    // a DDR read port stub factory (1-cycle, never busy)
    `define DDRPORT(NM) \
        ddr_rd_req_t NM``_dreq; ddr_rd_resp_t NM``_dresp; \
        reg [63:0] NM``_do; reg NM``_dv; \
        assign NM``_dresp.busy=1'b0; assign NM``_dresp.dout=NM``_do; assign NM``_dresp.dready=NM``_dv; \
        always @(posedge clk) begin NM``_dv<=0; if(NM``_dreq.rd) begin NM``_do<=vram[NM``_dreq.addr[19:0]]; NM``_dv<=1; end end

    // ---- serial reference: tsp_shade + 2 caches ----
    `DDRPORT(rd) `DDRPORT(rq)
    cache_req_t rd_creq, rq_creq; cache_resp_t rd_cresp, rq_cresp;
    tex_cache u_rdc (.clk(clk),.reset(reset),.creq(rd_creq),.cresp(rd_cresp),.dreq(rd_dreq),.dresp(rd_dresp));
    tex_cache u_rqc (.clk(clk),.reset(reset),.creq(rq_creq),.cresp(rq_cresp),.dreq(rq_dreq),.dresp(rq_dresp));
    tsp_shade u_ref (
        .clk(clk),.reset(reset),.req(ref_req),.px(px),.py(py),
        .done(ref_done),.argb(ref_argb),
        .invw_in(invw_in),.p_ddx(a_ddx),.p_ddy(a_ddy),.p_c(a_c),
        .tsp(tsp),.tcw(tcw),.text_ctrl(text_ctrl),
        .pp_texture(pp_texture),.pp_offset(pp_offset),
        .tc_req(rd_creq),.tc_resp(rd_cresp),.vq_req(rq_creq),.vq_resp(rq_cresp));

    // ---- pipelined dut: tsp_shade_pp + 4 corner pairs (8 caches) ----
    `DDRPORT(p0d) `DDRPORT(p0q) `DDRPORT(p1d) `DDRPORT(p1q)
    `DDRPORT(p2d) `DDRPORT(p2q) `DDRPORT(p3d) `DDRPORT(p3q)
    cache_req_t  pp_tc_req [0:3], pp_vq_req [0:3];
    cache_resp_t pp_tc_resp[0:3], pp_vq_resp[0:3];
    tex_cache u_p0d (.clk(clk),.reset(reset),.creq(pp_tc_req[0]),.cresp(pp_tc_resp[0]),.dreq(p0d_dreq),.dresp(p0d_dresp));
    tex_cache u_p0q (.clk(clk),.reset(reset),.creq(pp_vq_req[0]),.cresp(pp_vq_resp[0]),.dreq(p0q_dreq),.dresp(p0q_dresp));
    tex_cache u_p1d (.clk(clk),.reset(reset),.creq(pp_tc_req[1]),.cresp(pp_tc_resp[1]),.dreq(p1d_dreq),.dresp(p1d_dresp));
    tex_cache u_p1q (.clk(clk),.reset(reset),.creq(pp_vq_req[1]),.cresp(pp_vq_resp[1]),.dreq(p1q_dreq),.dresp(p1q_dresp));
    tex_cache u_p2d (.clk(clk),.reset(reset),.creq(pp_tc_req[2]),.cresp(pp_tc_resp[2]),.dreq(p2d_dreq),.dresp(p2d_dresp));
    tex_cache u_p2q (.clk(clk),.reset(reset),.creq(pp_vq_req[2]),.cresp(pp_vq_resp[2]),.dreq(p2q_dreq),.dresp(p2q_dresp));
    tex_cache u_p3d (.clk(clk),.reset(reset),.creq(pp_tc_req[3]),.cresp(pp_tc_resp[3]),.dreq(p3d_dreq),.dresp(p3d_dresp));
    tex_cache u_p3q (.clk(clk),.reset(reset),.creq(pp_vq_req[3]),.cresp(pp_vq_resp[3]),.dreq(p3q_dreq),.dresp(p3q_dresp));

    tsp_shade_pp #(.IDW(10)) u_pp (
        .clk(clk),.reset(reset),
        .in_valid(pp_in_valid),.in_id(pp_in_id),.px(px),.py(py),.invw_in(invw_in),
        .in_ddx(a_ddx),.in_ddy(a_ddy),.in_c(a_c),
        .tsp(tsp),.tcw(tcw),.text_ctrl(text_ctrl),
        .pp_texture(pp_texture),.pp_offset(pp_offset),
        .out_valid(pp_out_valid),.out_id(pp_out_id),.out_argb(pp_out_argb),
        .stall(pp_stall),
        .tc_req(pp_tc_req),.tc_resp(pp_tc_resp),.vq_req(pp_vq_req),.vq_resp(pp_vq_resp));
endmodule
