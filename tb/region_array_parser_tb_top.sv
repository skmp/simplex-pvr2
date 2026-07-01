// TB top: region_array_parser + injected data_cache256 + behavioral 64-bit DDR.
module region_array_parser_tb_top import tsp_pkg::*; (
    input             clk,
    input             reset,
    input             start,
    input      [26:0] region_base,
    input             region_v1,
    input             consume,          // pulse: ack.list_done
    output            busy,
    output            tiles_parsed,
    output            list_ready,
    output     [5:0]  tile_x,
    output     [5:0]  tile_y,
    output     [4:0]  state,
    output     [26:0] list_ptr
);
    region_out_t rout;
    region_ack_t ack;
    assign ack.list_done = consume;
    assign list_ready = rout.list_ready;
    assign tile_x     = rout.tile_x;
    assign tile_y     = rout.tile_y;
    assign state      = rout.state;
    assign list_ptr   = rout.list_ptr;

    cache_req256_t  creq;
    cache_resp256_t cresp;
    region_array_parser u_rap (
        .clk(clk),.reset(reset),.start(start),.region_base(region_base),
        .region_v1(region_v1),.busy(busy),.tiles_parsed(tiles_parsed),
        .rout(rout),.ack(ack),.creq(creq),.cresp(cresp));

    ddr_rd_req_t  dreq;
    ddr_rd_resp_t dresp;
    data_cache256 u_dc (.clk(clk),.reset(reset),.creq(creq),.cresp(cresp),
        .dreq(dreq),.dresp(dresp));

    (* verilator public_flat_rw *) reg [63:0] vram [0:65535];
    reg [63:0] dout_r; reg dready_r;
    assign dresp.busy=1'b0; assign dresp.dout=dout_r; assign dresp.dready=dready_r;
    always @(posedge clk) begin
        dready_r<=0;
        if (dreq.rd) begin dout_r<=vram[dreq.addr[15:0]]; dready_r<=1; end
    end
endmodule
