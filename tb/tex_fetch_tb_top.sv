// TB top: tex_fetch + 2 injected tex_caches + a behavioral 64-bit DDR (VRAM).
// Uses the tsp_pkg bundle types for the cache/DDR ports.
module texfetch_tb_top import tsp_pkg::*; (
    input             clk,
    input             reset,
    input             req,
    input      [10:0] u,
    input      [10:0] v,
    input      [31:0] tsp,
    input      [31:0] tcw,
    input      [4:0]  text_ctrl,
    output            ack,
    output     [31:0] argb
);
    // tex_fetch <-> caches (bundle pairs)
    cache_req_t  tc_req, vq_req;
    cache_resp_t tc_resp, vq_resp;

    tex_fetch u_tf (
        .clk(clk),.reset(reset),.req(req),.u(u),.v(v),
        .tsp(tsp),.tcw(tcw),.text_ctrl(text_ctrl),.ack(ack),.argb(argb),
        .tc_req(tc_req),.tc_resp(tc_resp),.vq_req(vq_req),.vq_resp(vq_resp));

    // two caches, each with an injected DDR read port
    ddr_rd_req_t  d_dreq, q_dreq;
    ddr_rd_resp_t d_dresp, q_dresp;
    tex_cache u_dc (.clk(clk),.reset(reset),.creq(tc_req),.cresp(tc_resp),.dreq(d_dreq),.dresp(d_dresp));
    tex_cache u_qc (.clk(clk),.reset(reset),.creq(vq_req),.cresp(vq_resp),.dreq(q_dreq),.dresp(q_dresp));

    // behavioral DDR: 64-bit words. addr = {4'b0011, wordidx[24:0]}.
    (* verilator public_flat_rw *) reg [63:0] vram [0:65535];
    reg [63:0] d_dout_r, q_dout_r; reg d_dready_r, q_dready_r;
    assign d_dresp.busy = 1'b0; assign q_dresp.busy = 1'b0;
    assign d_dresp.dout = d_dout_r; assign d_dresp.dready = d_dready_r;
    assign q_dresp.dout = q_dout_r; assign q_dresp.dready = q_dready_r;
    always @(posedge clk) begin
        d_dready_r<=0; q_dready_r<=0;
        if (d_dreq.rd) begin d_dout_r <= vram[d_dreq.addr[15:0]]; d_dready_r<=1; end
        if (q_dreq.rd) begin q_dout_r <= vram[q_dreq.addr[15:0]]; q_dready_r<=1; end
    end
endmodule
