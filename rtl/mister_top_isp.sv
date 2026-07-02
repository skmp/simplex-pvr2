//
// mister_top_isp - synthesizable MiSTer top for the ISP-only render core.
//
// Same HPS Avalon backend as mister_top (sysmem_lite f2sdram bridge, no DDR3
// pins), but wraps isp_core instead of peel_core: NO TSP, NO texture reads. The
// framebuffer is written with the per-pixel 32-bit CoreTag left by the ISP
// depth/tag pass (deferred-tile "tag visualisation").
//
//   * ram1 : isp_core's DDR READS (region/objlist/param) - one 64-bit burst read
//            in flight, driven by the injected ddr_req/ddr_resp arbiter output.
//   * ram2 : isp_core's framebuffer WRITES (CoreTags packed 2/word).
//   * vbuf : unused.
//
// The HPS ARM side preloads VRAM into DDR and writes the PVR registers
// (wr_en/wr_addr/wr_data), then pulses `go`; `done` marks completion. The
// framebuffer is written to DDR at FB_BASE (64-bit words, 2 tags/word) for the
// HPS/scanout to read.
//
module mister_top_isp import tsp_pkg::*; #(
    parameter [28:0] FB_BASE_WORD = 29'h0080000   // 64-bit-word address
) (
    input             reset_req,
    input             cold_req,
    output            core_clk,
    output            core_reset,
    input             wr_en,
    input      [12:0] wr_addr,
    input      [31:0] wr_data,
    input             go,
    output            done
);
    // ------------------------------------------------------------------
    // HPS DDR3 bridge (sysmem_lite): core clock/reset + Avalon RAM ports.
    // ------------------------------------------------------------------
    wire clk_100m, reset_100m;
    assign core_clk   = clk_100m;
    assign core_reset = reset_100m;

    // ram1 : DDR read channel (Avalon)
    wire        r1_clk = clk_100m;
    wire [28:0] r1_addr;
    wire  [7:0] r1_burstcnt;
    wire        r1_waitrequest;
    wire [63:0] r1_readdata;
    wire        r1_readdatavalid;
    wire        r1_read;
    // ram2 : DDR framebuffer write channel (Avalon)
    wire        r2_clk = clk_100m;
    wire [28:0] r2_addr;
    wire  [7:0] r2_burstcnt;
    wire        r2_waitrequest;
    wire        r2_write;
    wire [63:0] r2_writedata;
    wire  [7:0] r2_byteenable;

    sysmem_lite u_sysmem (
        .reset_core_req    (reset_req),
        .reset_out         (reset_100m),
        .clock             (clk_100m),
        .reset_hps_cold_req(cold_req),
        .reset_hps_warm_req(1'b0),

        // ram1 : reads
        .ram1_clk          (r1_clk),
        .ram1_address      (r1_addr),
        .ram1_burstcount   (r1_burstcnt),
        .ram1_waitrequest  (r1_waitrequest),
        .ram1_readdata     (r1_readdata),
        .ram1_readdatavalid(r1_readdatavalid),
        .ram1_read         (r1_read),
        .ram1_writedata    (64'd0),
        .ram1_byteenable   (8'hFF),
        .ram1_write        (1'b0),

        // ram2 : framebuffer writes
        .ram2_clk          (r2_clk),
        .ram2_address      (r2_addr),
        .ram2_burstcount   (r2_burstcnt),
        .ram2_waitrequest  (r2_waitrequest),
        .ram2_readdata     (),
        .ram2_readdatavalid(),
        .ram2_read         (1'b0),
        .ram2_writedata    (r2_writedata),
        .ram2_byteenable   (r2_byteenable),
        .ram2_write        (r2_write),

        // vbuf : unused
        .vbuf_clk          (clk_100m),
        .vbuf_address      (28'd0),
        .vbuf_burstcount   (8'd0),
        .vbuf_waitrequest  (),
        .vbuf_readdata     (),
        .vbuf_readdatavalid(),
        .vbuf_read         (1'b0),
        .vbuf_writedata    (128'd0),
        .vbuf_byteenable   (16'd0),
        .vbuf_write        (1'b0)
    );

    // ------------------------------------------------------------------
    // isp_core with the DDR read + framebuffer write injected.
    // ------------------------------------------------------------------
    ddr_rd_req_t  ddr_req;  ddr_rd_resp_t ddr_resp;
    fb_wr_req_t   fbw_req;  fb_wr_resp_t  fbw_resp;

    isp_core u_core (
        .clk(clk_100m), .reset(reset_100m),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .go(go), .done(done),
        .ddr_req(ddr_req), .ddr_resp(ddr_resp),
        .fbw_req(fbw_req), .fbw_resp(fbw_resp)
    );

    // ==================================================================
    // DDR READ master: isp_core ddr_req/ddr_resp  <->  Avalon ram1.
    // ==================================================================
    reg        rd_inflight;
    reg [7:0]  rd_left;
    wire       rd_issue = ddr_req.rd && !rd_inflight;
    assign r1_read     = rd_issue;
    assign r1_addr     = ddr_req.addr;
    assign r1_burstcnt = ddr_req.burst;

    assign ddr_resp.busy   = rd_inflight || (rd_issue && r1_waitrequest);
    assign ddr_resp.dout   = r1_readdata;
    assign ddr_resp.dready = r1_readdatavalid;

    always @(posedge clk_100m) begin
        if (reset_100m) begin
            rd_inflight <= 1'b0; rd_left <= 8'd0;
        end else begin
            if (!rd_inflight) begin
                if (rd_issue && !r1_waitrequest) begin
                    rd_inflight <= 1'b1;
                    rd_left     <= ddr_req.burst;
                end
            end else if (r1_readdatavalid) begin
                if (rd_left <= 8'd1) rd_inflight <= 1'b0;
                rd_left <= rd_left - 8'd1;
            end
        end
    end

    // ==================================================================
    // FRAMEBUFFER WRITE master: isp_core fbw_req (one 32-bit tag/cycle, linear
    // index)  ->  Avalon ram2 (64-bit words, 2 tags/word). Pairs even/odd indices.
    // ==================================================================
    reg        have_lo;
    reg [31:0] lo_px;
    reg [19:0] lo_idx;
    reg        wr_pending;
    reg [63:0] wr_data64;
    reg [28:0] wr_addr64;

    assign fbw_resp.busy = wr_pending;

    assign r2_write      = wr_pending;
    assign r2_addr       = wr_addr64;
    assign r2_writedata  = wr_data64;
    assign r2_burstcnt   = 8'd1;
    assign r2_byteenable = 8'hFF;

    always @(posedge clk_100m) begin
        if (reset_100m) begin
            have_lo <= 1'b0; wr_pending <= 1'b0;
        end else begin
            if (wr_pending && !r2_waitrequest) wr_pending <= 1'b0;

            if (fbw_req.we && !fbw_resp.busy) begin
                if (!have_lo) begin
                    have_lo <= 1'b1;
                    lo_px   <= fbw_req.argb;
                    lo_idx  <= fbw_req.pix_idx;
                end else begin
                    have_lo    <= 1'b0;
                    wr_pending <= 1'b1;
                    wr_data64  <= {fbw_req.argb, lo_px};
                    wr_addr64  <= FB_BASE_WORD + {9'd0, lo_idx[19:1]};
                end
            end
        end
    end
endmodule
