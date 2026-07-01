// TB top: data_cache256 + a behavioral 64-bit DDR (VRAM). Exercises the 32-byte
// (256-bit) line cache: 4 x 64-bit beats per fill. Uses tsp_pkg bundle types.
module data_cache256_tb_top import tsp_pkg::*; #(
    parameter integer NLINE = 256
) (
    input             clk,
    input             reset,
    input             req,
    input      [26:0] laddr,
    output            ack,
    output     [255:0] rdata
);
    cache_req256_t  creq;
    cache_resp256_t cresp;
    assign creq.req   = req;
    assign creq.laddr = laddr;
    assign ack   = cresp.ack;
    assign rdata = cresp.rdata;

    ddr_rd_req_t  dreq;
    ddr_rd_resp_t dresp;
    data_cache256 #(.NLINE(NLINE)) u_dc (
        .clk(clk),.reset(reset),.creq(creq),.cresp(cresp),.dreq(dreq),.dresp(dresp));

    // behavioral DDR: 64-bit words, addr = {4'b0011, wordidx[24:0]}. Model a
    // 1-cycle-latency ready with no busy (matches tex_fetch tb).
    (* verilator public_flat_rw *) reg [63:0] vram [0:65535];
    reg [63:0] dout_r; reg dready_r;
    assign dresp.busy   = 1'b0;
    assign dresp.dout   = dout_r;
    assign dresp.dready = dready_r;
    always @(posedge clk) begin
        dready_r <= 0;
        if (dreq.rd) begin dout_r <= vram[dreq.addr[15:0]]; dready_r <= 1; end
    end
endmodule
