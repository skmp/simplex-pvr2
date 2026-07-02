// peel_core - the layer-peeling tile render core, with the DDR controller and the
// framebuffer INJECTED as dependencies (ports). Contains: reg_file, the 6-client
// single-channel DDR arbiter, the data/texture caches, region/objlist/iterator
// parsers, ISP setup+raster, the pipelined TSP shade+blend, and the unified PT+TL
// peel FSM. The single 64-bit DDR read channel below the arbiter is exposed as
// ddr_req/ddr_resp; the shaded framebuffer is streamed out one pixel/cycle on fbw_*.
//
//   per pixel: tag -> plane-cache lookup (64-entry, keyed by the FULL 32-bit
//   CoreTag) miss: fetch param record (GetFpuEntry) + tsp_setup_min; hit: planes
//   from cache. tsp_shade_pp shades (textures via two tex_cache_4p over the shared
//   DDR channel), blend composites the layer, PT/TL peel back-to-front.
//
// Wrappers provide the DDR/fb backend:
//   frontend_tsp_lp_tb_top : faux DDR controller (behavioral vram[]) + fb[]  (sim)
//   mister_top             : real HPS Avalon ram1 read + framebuffer write  (synth)
//
module peel_core import tsp_pkg::*; (
    input             clk,
    input             reset,
    // register/command load (host writes the PVR reg dump before `go`)
    input             wr_en,
    input      [12:0] wr_addr,
    input      [31:0] wr_data,
    input             go,             // 1-cycle: start rendering the region array
    output reg        done,           // 1-cycle: region array fully processed

    // ---- injected DDR read controller (single 64-bit channel below the arbiter) ----
    output ddr_rd_req_t  ddr_req,
    input  ddr_rd_resp_t ddr_resp,

    // ---- injected framebuffer write port (one 32-bit pixel per accepted cycle) ----
    output fb_wr_req_t   fbw_req,
    input  fb_wr_resp_t  fbw_resp
);
    // -------------------- reg_file --------------------
    pvr_regs_t  regs;
    fog_rd_req_t fog_req; fog_rd_resp_t fog_resp;
    pal_rd_req_t pal_req; pal_rd_resp_t pal_resp;
    assign fog_req = '0; assign pal_req = '0;
    reg_file u_rf (.clk(clk),.reset(reset),.wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),
        .regs(regs),.fog_req(fog_req),.fog_resp(fog_resp),.pal_req(pal_req),.pal_resp(pal_resp));

    wire [26:0] region_base = regs.region_base[26:0];
    wire [26:0] param_base  = (regs.param_base[26:0] & 27'h0F00000); // PARAM_BASE & 0xF00000
    wire        region_v1   = (regs.fpu_param_cfg.region_header_type == 1'b0);

    // ==================== SINGLE SHARED DDR CHANNEL (arbiter) ====================
    // Real hardware (MiSTer) has ONE 64-bit DDR read channel. All six clients -
    // region array (ra), object list (ol), ISP param/vertex (pr), TSP param (ts),
    // and the two 4-read-port texture caches (tc data, vq codebook) - are
    // arbitrated onto the INJECTED single-channel DDR controller (ddr_req/ddr_resp;
    // one read in flight). Priority (high->low): tc, vq, ts, pr, ol, ra - shade-
    // critical clients win, geometry fills between. Per-client PENDING latch
    // captures each 1-cycle rd pulse so a request is never lost while the channel
    // is busy elsewhere. The controller (faux or real Avalon) owns latency+burst.
    ddr_rd_req_t  ra_dreq, ol_dreq, pr_dreq, ts_dreq;
    ddr_rd_resp_t ra_dresp, ol_dresp, pr_dresp, ts_dresp;
    ddr_rd_req_t  tex_dreq [0:1];      // [0]=tc data, [1]=vq codebook
    ddr_rd_resp_t tex_dresp [0:1];

    // 0=tc 1=vq 2=ts 3=pr 4=ol 5=ra (priority high->low)
    reg  [5:0]  pend;
    reg  [28:0] pa [0:5]; reg [7:0] pb [0:5];
    wire [5:0]  rd_pulse = { ra_dreq.rd, ol_dreq.rd, pr_dreq.rd, ts_dreq.rd,
                             tex_dreq[1].rd, tex_dreq[0].rd };
    wire [28:0] ca [0:5]; wire [7:0] cbv [0:5];
    assign ca[0]=tex_dreq[0].addr; assign cbv[0]=tex_dreq[0].burst;
    assign ca[1]=tex_dreq[1].addr; assign cbv[1]=tex_dreq[1].burst;
    assign ca[2]=ts_dreq.addr;     assign cbv[2]=ts_dreq.burst;
    assign ca[3]=pr_dreq.addr;     assign cbv[3]=pr_dreq.burst;
    assign ca[4]=ol_dreq.addr;     assign cbv[4]=ol_dreq.burst;
    assign ca[5]=ra_dreq.addr;     assign cbv[5]=ra_dreq.burst;

    wire       any_pend = |pend;
    wire [2:0] d_win = pend[0] ? 3'd0 : pend[1] ? 3'd1 : pend[2] ? 3'd2 :
                       pend[3] ? 3'd3 : pend[4] ? 3'd4 : 3'd5;
    // A read is in flight (d_busy) from when we grant until the last beat arrives.
    // d_beats counts remaining beats; the injected controller streams ddr_resp.
    reg        d_busy; reg [2:0] d_owner;
    reg [7:0]  d_beats;
    reg        d_issued;   // ddr_req.rd already pulsed for the current grant
    integer di;

    // drive the injected DDR controller: issue the granted read once, then wait
    // for `d_beats` dready beats. ddr_req.rd is a 1-cycle pulse (accepted when
    // !ddr_resp.busy). We assert it on grant and hold-until-accepted.
    assign ddr_req.rd    = d_busy && !d_issued;
    assign ddr_req.addr  = pa[d_owner];
    assign ddr_req.burst = d_beats;

    always @(posedge clk) begin
        if (reset) begin d_busy <= 1'b0; pend <= 6'd0; d_issued <= 1'b0; end
        else begin
            for (di=0; di<6; di=di+1)
                if (rd_pulse[di]) begin pend[di] <= 1'b1; pa[di] <= ca[di]; pb[di] <= cbv[di]; end
            if (!d_busy) begin
                if (any_pend) begin
                    d_busy   <= 1'b1; d_owner <= d_win;
                    d_beats  <= pb[d_win];
                    d_issued <= 1'b0;
                    pend[d_win] <= (rd_pulse[d_win]);  // clear grant (unless re-pulsed same cyc)
                end
            end else begin
                // hold ddr_req.rd until the controller accepts it
                if (ddr_req.rd && !ddr_resp.busy) d_issued <= 1'b1;
                // count returned beats; release the channel after the last
                if (ddr_resp.dready) begin
                    if (d_beats <= 8'd1) begin d_busy <= 1'b0; d_issued <= 1'b0; end
                    d_beats <= d_beats - 8'd1;
                end
            end
        end
    end
    // a client cannot issue while the channel is busy or it has a pending request
    assign tex_dresp[0].busy = d_busy || pend[0];
    assign tex_dresp[1].busy = d_busy || pend[1];
    assign ts_dresp.busy     = d_busy || pend[2];
    assign pr_dresp.busy     = d_busy || pend[3];
    assign ol_dresp.busy     = d_busy || pend[4];
    assign ra_dresp.busy     = d_busy || pend[5];
    assign tex_dresp[0].dout=ddr_resp.dout; assign tex_dresp[1].dout=ddr_resp.dout;
    assign ts_dresp.dout=ddr_resp.dout; assign pr_dresp.dout=ddr_resp.dout;
    assign ol_dresp.dout=ddr_resp.dout; assign ra_dresp.dout=ddr_resp.dout;
    assign tex_dresp[0].dready = ddr_resp.dready && (d_owner==3'd0);
    assign tex_dresp[1].dready = ddr_resp.dready && (d_owner==3'd1);
    assign ts_dresp.dready     = ddr_resp.dready && (d_owner==3'd2);
    assign pr_dresp.dready     = ddr_resp.dready && (d_owner==3'd3);
    assign ol_dresp.dready     = ddr_resp.dready && (d_owner==3'd4);
    assign ra_dresp.dready     = ddr_resp.dready && (d_owner==3'd5);

    // -------------------- caches --------------------
    // Region parser, OL parser, ISP iterator AND TSP param fetch all read DDR
    // directly (each with its own 8-word sliding-window line reader). No caches.

    // The 4 corner fetchers share ONE 4-read-port data cache + ONE 4-read-port VQ
    // cache (2x dual-port M10K each, full copy), replacing the 8 per-corner
    // tex_cache instances. Simultaneous same-line misses dedupe to one DDR read;
    // distinct-line misses serialize (the pipe stalls while any corner is busy).
    cache_req_t   pp_tc_req [0:3], pp_vq_req [0:3];
    cache_resp_t  pp_tc_resp[0:3], pp_vq_resp[0:3];
    tex_cache_4p u_tc4 (.clk(clk),.reset(reset),.creq(pp_tc_req),.cresp(pp_tc_resp),
        .dreq(tex_dreq[0]),.dresp(tex_dresp[0]));
    tex_cache_4p u_vq4 (.clk(clk),.reset(reset),.creq(pp_vq_req),.cresp(pp_vq_resp),
        .dreq(tex_dreq[1]),.dresp(tex_dresp[1]));

    // -------------------- parsers --------------------
    reg          ra_start;
    wire         ra_busy, ra_tiles_parsed;
    region_out_t ra_out; region_ack_t ra_ack;
    region_array_parser u_ra (.clk(clk),.reset(reset),.start(ra_start),
        .region_base(region_base),.region_v1(region_v1),.busy(ra_busy),
        .tiles_parsed(ra_tiles_parsed),.rout(ra_out),.ack(ra_ack),
        .dreq(ra_dreq),.dresp(ra_dresp));

    reg          ol_start; reg [26:0] ol_list_ptr;
    wire         ol_busy, ol_done;
    prim_out_t   ol_prim; prim_ack_t ol_ack;
    object_list_parser u_ol (.clk(clk),.reset(reset),.start(ol_start),
        .list_ptr(ol_list_ptr),.busy(ol_busy),.done(ol_done),
        .prim(ol_prim),.ack(ol_ack),.dreq(ol_dreq),.dresp(ol_dresp));

    // Prefetching iterator: streaming entry input (entry_valid/entry_ack) driven
    // from the entry FIFO head, ping-pong record buffers so the next record's DDR
    // burst overlaps the current record's triangle emit. The barrier gates on the
    // iterator's AUTHORITATIVE level busy (it_pf_busy), NOT a pulse-cleared reg -
    // the peel loop re-runs the OL list per pass; the iterator restarts simply by
    // having entries streamed into it again (idle -> busy as new entries arrive).
    wire             it_entry_valid, it_entry_ack, it_pf_busy;
    objlist_entry_t  it_entry; entry_type_e it_etype;   // combinational eq head
    triangle_out_t   it_trio; triangle_ack_t it_ack;
    isp_primitive_iterator_pf u_it (.clk(clk),.reset(reset),
        .intensity_shadow(regs.fpu_shad_scale.intensity_shadow),
        .param_base(param_base),
        .entry_valid(it_entry_valid),.entry_type(it_etype),.entry(it_entry),
        .entry_ack(it_entry_ack),.busy(it_pf_busy),
        .trio(it_trio),.ack(it_ack),.dreq(pr_dreq),.dresp(pr_dresp));

    // -------------------- depth/tag tile + color buffer + framebuffer --------------------
    // IMPROVEMENT #3 (APPLIED) - M10K banking of the tile buffers. The five peel
    // depth/tag buffers (dt_depth/dt_depth2/dt_tag/dt_tag2/dt_valid) are packed into
    // ONE 129-bit x 8-bank simple-dual-port tile_ram (u_peel); col_buf into a single
    // 1024x32 M10K (u_col). Bank = x[2:0], addr = {y[4:0], x[4:3]} (7-bit, 128
    // entries/bank), so a whole 8-lane raster chunk is one address across 8 banks -
    // exactly the isp_core scheme. The registered read forces the peel depth compare
    // and the blend RMW into 2-stage stage-A/stage-B pipelines (see the raster
    // consumer and the CB blend stage), and CLEAR / PeelBuffers into sequential
    // 128-chunk RAM walks. Barriers already serialize the raster / shade / bulk
    // phases, so the single read+write port of each RAM is never contended across
    // phases. dt_pt stays a REGISTER array (1 bit x 1024, negligible M10K, read
    // combinationally by the blend alpha test - banking it would need a 2nd peel-RAM
    // read port for the shade producer/consumer split, for no M10K gain).
    //
    // The 4-way streamed setup + 8-deep plane FIFO (#1), the prefetch iterator +
    // it_pf_busy barrier (#2), and the 1px/cycle combinational FLUSH (#4) are also
    // applied.
    localparam integer TILE_W = 32, TILE_H = 32;
    localparam integer NB = RAS_LANES;         // 8 banks (one per raster lane)

    // ---- peel RAM: {valid, tag2[31:0], tag[31:0], depth2[31:0], depth[31:0]} / lane ----
    // packed word field offsets (per lane, LSB-first):
    localparam integer PW_DEPTH  = 0;          // [31:0]  depthBufferA (zb)
    localparam integer PW_DEPTH2 = 32;         // [31:0]  depthBufferB (zb2, reference)
    localparam integer PW_TAG    = 64;         // [31:0]  tagBufferA   (pb)
    localparam integer PW_TAG2   = 96;         // [31:0]  tagBufferB   (pb2)
    localparam integer PW_VALID  = 128;        // [0]     tagStatus.valid
    localparam integer PEEL_W = 129;           // bits per lane

    reg  [NB-1:0]         pr_we;
    reg  [7*NB-1:0]       pr_waddr;            // stage-B / bulk write address
    reg  [7*NB-1:0]       pr_raddr;            // stage-A / bulk / shade read address
    reg  [PEEL_W*NB-1:0]  pr_wdata;
    wire [PEEL_W*NB-1:0]  pr_rdata;
    tile_ram #(.WIDTH(PEEL_W), .NBANKS(NB)) u_peel (
        .clk(clk), .we(pr_we), .waddr(pr_waddr), .wdata(pr_wdata),
        .raddr(pr_raddr), .rdata(pr_rdata)
    );

    // pack a 7-bit bank address {y[4:0], x[4:3]} onto all 8 banks (same addr/bank)
    function automatic [7*NB-1:0] pr_pack_addr(input [4:0] y, input [4:0] xchunk);
        integer b;
        begin
            pr_pack_addr = '0;
            for (b = 0; b < NB; b = b + 1)
                pr_pack_addr[7*b +: 7] = {y, xchunk[4:3]};
        end
    endfunction
    // per-lane field extractors from a packed chunk word
    function automatic [31:0] pr_depth (input [PEEL_W*NB-1:0] w, input integer b);
        pr_depth  = w[PEEL_W*b + PW_DEPTH  +: 32]; endfunction
    function automatic [31:0] pr_depth2(input [PEEL_W*NB-1:0] w, input integer b);
        pr_depth2 = w[PEEL_W*b + PW_DEPTH2 +: 32]; endfunction
    function automatic [31:0] pr_tag   (input [PEEL_W*NB-1:0] w, input integer b);
        pr_tag    = w[PEEL_W*b + PW_TAG    +: 32]; endfunction
    function automatic [31:0] pr_tag2  (input [PEEL_W*NB-1:0] w, input integer b);
        pr_tag2   = w[PEEL_W*b + PW_TAG2   +: 32]; endfunction
    function automatic       pr_valid  (input [PEEL_W*NB-1:0] w, input integer b);
        pr_valid  = w[PEEL_W*b + PW_VALID]; endfunction

    // dt_pt kept as a register array (see note above)
    reg        dt_pt [0:TILE_W*TILE_H-1];      // winning peel fragment came from PT list

    // ---- color RAM: single-bank 1024x32 (low load - only TSP blend + FLUSH) ----
    reg          cr_we;
    reg  [9:0]   cr_waddr, cr_raddr;
    reg  [31:0]  cr_wdata;
    wire [31:0]  cr_rdata;
    (* ramstyle = "M10K, no_rw_check" *) reg [31:0] col_ram [0:TILE_W*TILE_H-1];
    reg  [31:0]  cr_rdata_r;
    always @(posedge clk) begin
        if (cr_we) col_ram[cr_waddr] <= cr_wdata;
        cr_rdata_r <= col_ram[cr_raddr];        // registered read (read-first)
    end
    assign cr_rdata = cr_rdata_r;

    localparam [31:0] FLT_MAX = 32'h7F7FFFFF;  // refsw PeelBuffers depth clear value

    // -------------------- int -> float (tile origin, 0..2016) --------------------
    function automatic [31:0] i2f(input [15:0] v);
        integer i, p; reg [38:0] m;
        begin
            p = -1;
            for (i = 0; i < 16; i = i + 1) if (v[i]) p = i;
            if (p < 0) i2f = 32'd0;
            else begin
                m   = {23'd0, v} << (23 - p);
                i2f = {1'b0, 8'(127 + p), m[22:0]};
            end
        end
    endfunction

    // -------------------- ISP triangle setup (4-way interleaved streamed) --------------------
    // isp_word/tri_tag are the ACTIVE raster triangle (popped from the plane FIFO
    // pq) used by the depth compare / tag write. The streamed setup accepts from the
    // tri FIFO (fq) head and retires planes into the 8-deep plane FIFO (pq).
    reg  [31:0] isp_word;                  // active (raster) triangle's isp
    reg  [31:0] t_xbase, t_ybase;
    reg  [31:0] tri_tag;                   // active (raster) triangle's CoreTag
    wire        isp_sgn_neg, isp_cull;
    wire [4:0]  w_bx0, w_bx1, w_by0, w_by1;   // tile-local bbox from setup
    wire [31:0] w_dx12,w_dx23,w_dx31,w_dx41, w_dy12,w_dy23,w_dy31,w_dy41;
    wire [31:0] w_c1,w_c2,w_c3,w_c4, w_ddx,w_ddy,w_cinvw;

    // Streaming setup interface: accept a triangle from the tri FIFO (fq) whenever
    // in_ready, retire one (out_valid) into the plane FIFO (pq) when out_ready.
    //   su_in_valid  : fq has a triangle AND pq has room (throttle at pq near-full)
    //   su_out_valid : retire; push non-culled planes into pq (out_ready = !pq_full)
    //   su_busy      : any slot in flight (barrier).
    wire        su_in_valid, su_in_ready, su_out_valid, su_busy;
    wire [31:0] su_out_tag, su_out_isp;
    assign su_in_valid = !fq_empty && (pq_count <= 5'd4);

    isp_setup_streamed u_isp (
        .clk(clk), .reset(reset),
        .in_valid(su_in_valid), .in_ready(su_in_ready),
        .isp_word(fq_isp[fq_head[2:0]]), .in_tag(fq_tag[fq_head[2:0]]),
        .x1(fq_x1[fq_head[2:0]]), .y1(fq_y1[fq_head[2:0]]), .z1(fq_z1[fq_head[2:0]]),
        .x2(fq_x2[fq_head[2:0]]), .y2(fq_y2[fq_head[2:0]]), .z2(fq_z2[fq_head[2:0]]),
        .x3(fq_x3[fq_head[2:0]]), .y3(fq_y3[fq_head[2:0]]), .z3(fq_z3[fq_head[2:0]]),
        .xbase(t_xbase), .ybase(t_ybase),
        .busy(su_busy),
        .out_ready(!pq_full),
        .out_valid(su_out_valid), .out_tag(su_out_tag), .out_isp(su_out_isp),
        .sgn_neg(isp_sgn_neg), .cull(isp_cull),
        .dx12(w_dx12), .dx23(w_dx23), .dx31(w_dx31), .dx41(w_dx41),
        .dy12(w_dy12), .dy23(w_dy23), .dy31(w_dy31), .dy41(w_dy41),
        .c1(w_c1), .c2(w_c2), .c3(w_c3), .c4(w_c4),
        .ddx_invw(w_ddx), .ddy_invw(w_ddy), .c_invw(w_cinvw),
        .bx0(w_bx0), .bx1(w_bx1), .by0(w_by0), .by1(w_by1)
    );

    // latched setup results (rasterizer consumes these)
    reg [31:0] isp_dx12,isp_dx23,isp_dx31,isp_dx41;
    reg [31:0] isp_dy12,isp_dy23,isp_dy31,isp_dy41;
    reg [31:0] isp_c1,isp_c2,isp_c3,isp_c4;
    reg [31:0] isp_ddx_invw, isp_ddy_invw, isp_c_invw;

    // -------------------- ISP rasterize (as tile_engine_top) --------------------
    // 8 depth lanes/clock, matching the real FPGA (32 lanes is DSP-heavy). Sim
    // models the same 8 lanes so cycle counts reflect hardware.
    localparam integer RAS_LANES = 8;
    reg  [4:0]  ras_y, ras_x;
    reg  [4:0]  rbx0, rbx1, rby1;   // active bbox sweep bounds (chunk-aligned x)
    // combinational: issue a chunk every raster-sweep cycle, in phase with ras_x/y
    // (a registered pulse lags one cycle -> pairs with the advanced ras_x and
    //  drops the first chunk of every tile).
    wire        ras_in_valid = (rs_st == RS_RAS);
    wire        ras_out_valid;
    wire [RAS_LANES-1:0]    ras_inside;
    wire [32*RAS_LANES-1:0] ras_invw_flat;
    function [31:0] ras_invw(input integer lane);
        ras_invw = ras_invw_flat[32*lane +: 32];
    endfunction

    wire [4:0] ras_ox, ras_oy;     // coords echoed with the result chunk
    isp_raster_line #(.LANES(RAS_LANES)) u_line (
        .clk(clk), .reset(reset),
        .in_valid(ras_in_valid), .y(ras_y), .x_base(ras_x),
        .c1(isp_c1), .c2(isp_c2), .c3(isp_c3), .c4(isp_c4),
        .dx12(isp_dx12),.dx23(isp_dx23),.dx31(isp_dx31),.dx41(isp_dx41),
        .dy12(isp_dy12),.dy23(isp_dy23),.dy31(isp_dy31),.dy41(isp_dy41),
        .ddx(isp_ddx_invw),.ddy(isp_ddy_invw),.c_invw(isp_c_invw),
        .out_valid(ras_out_valid),
        .inside_mask(ras_inside),
        .invw_flat(ras_invw_flat),
        .out_x(ras_ox), .out_y(ras_oy)
    );

    wire [2:0] depth_mode = isp_word[31:29];
    wire       zwrite_dis = isp_word[26];

    // `peeling` selects the layer-peel (RM_TRANSLUCENT_AUTOSORT) compare for the
    // active raster triangle; OP uses the plain opaque DepthMode compare. Set when
    // the active region state is PT or TR (PT treated as TR for now).
    reg        peeling;

    // ---- raster consumer stage-A -> stage-B pipeline (M10K registered read) ----
    // Stage A (on ras_out_valid): present the peel-RAM READ for the resolved chunk
    // and latch the raster result fields. Stage B (next cycle, b_valid): pr_rdata =
    // that chunk's OLD packed {depth,depth2,tag,tag2,valid}; the dcmp/dcmp_lp run
    // COMBINATIONALLY off pr_rdata (exactly the values the old combinational path
    // read from dt_*[] one cycle earlier), then the write port writes back.
    reg                    b_valid;
    reg [RAS_LANES-1:0]    b_inside;
    reg [32*RAS_LANES-1:0] b_invw;
    reg [4:0]              b_ox, b_oy;
    reg [31:0]             b_tag;
    reg [2:0]              b_mode;
    reg                    b_zwdis;
    reg                    b_peeling;   // carry the peel/opaque select into stage B
    reg                    b_which;     // peel_which snapshot (PT list => dt_pt=1)
    function [31:0] b_invw_lane(input integer lane);
        b_invw_lane = b_invw[32*lane +: 32];
    endfunction

    // per-lane compare on the STAGE-B chunk (pr_rdata), using the latched b_* fields.
    //  - opaque path: isp_depth_cmp (DepthMode) -> ras_pass_op
    //  - peel   path: isp_depth_cmp_lp          -> ras_pass_lp + ras_more_lp
    wire [RAS_LANES-1:0] ras_pass_op, ras_pass_lp, ras_more_lp;
    genvar gd;   // declared out-of-line (Quartus Standard rejects inline for-genvar)
    generate
        for (gd = 0; gd < RAS_LANES; gd = gd + 1) begin : dcmp
            isp_depth_cmp u_cmp (
                .mode(b_mode),
                .nw  (b_invw[32*gd +: 32]),
                .ob  (pr_depth(pr_rdata, gd)),
                .pass(ras_pass_op[gd]));
            isp_depth_cmp_lp u_cmp_lp (
                .nw   (b_invw[32*gd +: 32]),
                .tag  (b_tag),
                .zb   (pr_depth (pr_rdata, gd)),
                .zb2  (pr_depth2(pr_rdata, gd)),
                .pb   (pr_tag   (pr_rdata, gd)),
                .pb2  (pr_tag2  (pr_rdata, gd)),
                .valid(pr_valid (pr_rdata, gd)),
                .pass (ras_pass_lp[gd]),
                .more (ras_more_lp[gd]));
        end
    endgenerate

    // ==================================================================
    // TSP plane cache: 64 entries, keyed by the FULL CoreTag word.
    // cache_bypass is deliberately IGNORED (always cache). Miss -> param
    // fetch (GetFpuEntry) + tsp_setup_min; per entry: 10 planes {ddx,ddy,c}
    // + the record's isp/tsp/tcw words.
    // ==================================================================
    localparam integer PC_N = 64;
    reg            pc_valid [0:PC_N-1];
    reg [31:0]     pc_tag   [0:PC_N-1];
    reg [31:0]     pc_isp   [0:PC_N-1];
    reg [31:0]     pc_tsp   [0:PC_N-1];
    reg [31:0]     pc_tcw   [0:PC_N-1];
    reg [31:0]     pc_ddx   [0:PC_N-1][0:9];
    reg [31:0]     pc_ddy   [0:PC_N-1][0:9];
    reg [31:0]     pc_c     [0:PC_N-1][0:9];

    // current entry (feeds tsp_shade)
    reg [31:0] cur_isp, cur_tsp, cur_tcw;
    reg [31:0] cur_ddx [0:9];
    reg [31:0] cur_ddy [0:9];
    reg [31:0] cur_c   [0:9];

    // shade-pass state
    reg [9:0]  shp;           // current tile pixel 0..1023
    reg [9:0]  fw_i;          // framebuffer-writeout pixel 0..1023
    reg [31:0] sh_tag;        // that pixel's CoreTag
    reg [31:0] sh_invw;       // that pixel's depth-buffer invW
    // slot: param_offs low bits XOR tag_offset (strip triangles share param_offs)
    wire [5:0] sh_slot = sh_tag[8:3] ^ {3'b000, sh_tag[2:0]};
    // CoreTag fields (ISP_BACKGND_T layout)
    wire [20:0] sh_po   = sh_tag[23:3];
    wire [2:0]  sh_skip = sh_tag[26:24];
    wire [2:0]  sh_toff = sh_tag[2:0];
    wire        sh_two_vol = sh_tag[27] & ~regs.fpu_shad_scale.intensity_shadow;
    wire [4:0]  sh_stride_w = 5'd3 + sh_skip * (sh_two_vol ? 5'd2 : 5'd1);
    wire [26:0] sh_stride_b = {sh_stride_w, 2'b00};

    // ---- 32-bit word reader: DIRECT DDR, 8-word sliding-window (as object_list_parser) ----
    // f_go with a byte address (f_addr) returns the word NEXT cycle in
    // f_word/f_word_v on a resident line. Each miss fetches a whole 256-bit line
    // (burst=8) into tw0 (demand) or tw1 (prefetch of tw0+1). Drives ts_dreq/ts_dresp.
    reg  [26:0] f_addr; reg f_go; reg [31:0] f_word; reg f_word_v; reg [2:0] f_sel;
    reg  [255:0] tw0; reg [21:0] t0_tag; reg t0_v;
    reg  [255:0] tw1; reg [21:0] t1_tag; reg t1_v;
    reg          tpend; reg [21:0] tline; reg [2:0] tsel;

    localparam TF_IDLE=2'd0, TF_MISS=2'd1, TF_FILL=2'd2;
    reg [1:0]   tfst;
    reg [21:0]  tf_line; reg tf_is_pf; reg [2:0] tf_beat; reg [255:0] tf_acc;
    wire        tf_bank    = tf_line[17];
    wire [19:0] tf_wofs_b  = {tf_line[16:0], 3'b000};
    wire [28:0] tf_base_wd = {9'b0, tf_wofs_b};
    wire [31:0] tf_half    = tf_bank ? ts_dresp.dout[63:32] : ts_dresp.dout[31:0];

    reg        ts_rd_r; reg [28:0] ts_addr_r; reg [7:0] ts_burst_r;
    assign ts_dreq.rd    = ts_rd_r;
    assign ts_dreq.addr  = ts_addr_r;
    assign ts_dreq.burst = ts_burst_r;

    always @(posedge clk) begin
        f_word_v <= 1'b0;
        ts_rd_r  <= 1'b0;

        if (f_go) begin tpend <= 1'b1; tline <= f_addr[26:5]; tsel <= f_addr[4:2]; end

        if (tpend) begin
            if (t0_v && t0_tag == tline) begin
                f_word <= tw0[32*tsel +: 32]; f_word_v <= 1'b1; if (!f_go) tpend <= 1'b0;
            end else if (t1_v && t1_tag == tline) begin
                tw0 <= tw1; t0_tag <= t1_tag; t0_v <= 1'b1; t1_v <= 1'b0;
                f_word <= tw1[32*tsel +: 32]; f_word_v <= 1'b1; if (!f_go) tpend <= 1'b0;
            end
        end

        case (tfst)
        TF_IDLE: begin
            if (tpend && !(t0_v && t0_tag==tline) && !(t1_v && t1_tag==tline)) begin
                tf_line <= tline; tf_is_pf <= 1'b0; tf_beat <= 3'd0; t1_v <= 1'b0;
                tfst <= TF_MISS;
            end else if (t0_v && !(t1_v && t1_tag == t0_tag + 22'd1)) begin
                tf_line <= t0_tag + 22'd1; tf_is_pf <= 1'b1; tf_beat <= 3'd0;
                tfst <= TF_MISS;
            end
        end
        TF_MISS: if (!ts_dresp.busy) begin
            ts_rd_r    <= 1'b1;
            ts_addr_r  <= {4'b0011, tf_base_wd[24:0]};
            ts_burst_r <= 8'd8;
            tfst       <= TF_FILL;
        end
        TF_FILL: if (ts_dresp.dready) begin
            tf_acc[32*tf_beat +: 32] <= tf_half;
            if (tf_beat == 3'd7) begin
                if (tf_is_pf) begin tw1 <= { tf_half, tf_acc[223:0] }; t1_tag <= tf_line; t1_v <= 1'b1; end
                else          begin tw0 <= { tf_half, tf_acc[223:0] }; t0_tag <= tf_line; t0_v <= 1'b1; end
                tfst <= TF_IDLE;
            end else tf_beat <= tf_beat + 3'd1;
        end
        default: tfst <= TF_IDLE;
        endcase

        if (reset) begin t0_v<=0; t1_v<=0; tpend<=0; tfst<=TF_IDLE; ts_rd_r<=0; end
    end

    // fetched vertices (3) - decode_pvr_vertex fields
    reg [31:0] fv_x[0:2], fv_y[0:2], fv_z[0:2];
    reg [31:0] fv_u[0:2], fv_v[0:2], fv_col[0:2], fv_ofs[0:2];
    reg [1:0]  fv_i;           // vertex being fetched 0..2
    reg [2:0]  fv_fld;         // field sequencer (see FV_* below)
    reg [26:0] f_rec, f_vtx;   // record base / current vertex base byte addr
    reg [26:0] f_ptr;          // running word pointer

    // decoded isp flags of the record being fetched
    wire f_texture = cur_isp[ISP_TEXTURE_BIT];
    wire f_offset  = cur_isp[ISP_OFFSET_BIT];
    wire f_gouraud = cur_isp[ISP_GOURAUD_BIT];
    wire f_uv16    = cur_isp[ISP_UV16_BIT];

    // -------------------- TSP setup (plane producer) --------------------
    reg         tsp_start;
    wire        tsp_done, tsp_pvalid;
    wire [3:0]  tsp_pidx;
    wire [31:0] tsp_pddx, tsp_pddy, tsp_pc;
    tsp_setup_min u_tsp (
        .clk(clk), .reset(reset), .start(tsp_start), .done(tsp_done),
        .gouraud(f_gouraud), .texture(f_texture), .offset(f_offset),
        .x1(fv_x[0]),.y1(fv_y[0]),.z1(fv_z[0]),
        .x2(fv_x[1]),.y2(fv_y[1]),.z2(fv_z[1]),
        .x3(fv_x[2]),.y3(fv_y[2]),.z3(fv_z[2]),
        .xbase(t_xbase), .ybase(t_ybase),
        .u1(fv_u[0]),.v1(fv_v[0]),.u2(fv_u[1]),.v2(fv_v[1]),.u3(fv_u[2]),.v3(fv_v[2]),
        .col1(fv_col[0]),.col2(fv_col[1]),.col3(fv_col[2]),
        .ofs1(fv_ofs[0]),.ofs2(fv_ofs[1]),.ofs3(fv_ofs[2]),
        .plane_valid(tsp_pvalid), .plane_idx(tsp_pidx),
        .o_ddx(tsp_pddx), .o_ddy(tsp_pddy), .o_c(tsp_pc)
    );

    // -------------------- TSP shade (FULLY PIPELINED, 1 pixel/clock) --------------------
    // The producer FSM presents a resolved pixel (planes from the plane cache +
    // its tsp/tcw/isp flags) on pp_in_valid; tsp_shade_pp streams results out on
    // pp_out_valid, carrying the pixel index (0..1023) as the id so the consumer
    // can write col_buf[out_id]. pp_stall (any texel fetcher busy) freezes the
    // pipe; the producer holds while stalled.
    reg          pp_in_valid;
    reg  [9:0]   pp_in_id;       // = pixel index shp
    reg  [4:0]   pp_px, pp_py;
    reg  [31:0]  pp_invw;
    reg  [31:0]  pp_tsp, pp_tcw; reg pp_ptex, pp_pofs;
    reg  [31:0]  pp_ddx [0:9];
    reg  [31:0]  pp_ddy [0:9];
    reg  [31:0]  pp_c   [0:9];
    wire         pp_stall;
    wire         pp_out_valid;
    wire [9:0]   pp_out_id;
    wire [31:0]  pp_out_argb;
    wire [31:0]  pp_out_tsp;

    tsp_shade_pp #(.IDW(10)) u_shade (
        .clk(clk),.reset(reset),
        .in_valid(pp_in_valid),.in_id(pp_in_id),.px(pp_px),.py(pp_py),.invw_in(pp_invw),
        .in_ddx(pp_ddx),.in_ddy(pp_ddy),.in_c(pp_c),
        .tsp(pp_tsp),.tcw(pp_tcw),.text_ctrl(regs.text_control[4:0]),
        .pp_texture(pp_ptex),.pp_offset(pp_pofs),
        .out_valid(pp_out_valid),.out_id(pp_out_id),.out_argb(pp_out_argb),
        .out_tsp(pp_out_tsp),
        .stall(pp_stall),
        .tc_req(pp_tc_req),.tc_resp(pp_tc_resp),.vq_req(pp_vq_req),.vq_resp(pp_vq_resp));

    // -------- blend unit: the very end of the TSP pipeline (refsw BlendingUnit) --------
    // The color buffer is now an M10K (col_ram, registered read), so the blend RMW
    // is a 2-stage pipeline (like the raster depth compare):
    //   stage CA (on pp_out_valid && !pp_stall): latch cb_* and present the col-RAM
    //     READ of cb_id; cb_valid<=1.
    //   stage CB (next cycle, cb_valid): cr_rdata = OLD col_buf[cb_id] = the dst;
    //     tsp_blend runs COMBINATIONALLY off it, then col-RAM[cb_id] <- blend_out.
    // Because the shade sub-phase presents pixels in ASCENDING shp order and the pipe
    // is in-order, out ids never repeat within a sub-phase -> CA id N and CB id N-1
    // are always distinct, so no same-address RMW hazard. SH_DRAIN additionally
    // waits !cb_valid before returning so the trailing blend lands before the next
    // phase (peel pass / FLUSH) touches col_ram.
    reg          cb_valid;
    reg  [9:0]   cb_id;
    reg  [31:0]  cb_argb, cb_tsp;
    reg          cb_at_en;    // alpha-test enable snapshot (peeling && dt_pt[id])
    wire [2:0]   cb_src_instr = cb_tsp[31:29];
    wire [2:0]   cb_dst_instr = cb_tsp[28:26];
    wire [31:0]  pp_blend_out;
    wire         pp_blend_at;
    tsp_blend u_blend (
        .src       (cb_argb),
        .dst       (cr_rdata),          // registered read: OLD col_buf[cb_id]
        .src_instr (cb_src_instr),
        .dst_instr (cb_dst_instr),
        .alpha_test(cb_at_en),
        .alpha_ref (regs.pt_alpha_ref[7:0]),
        .out       (pp_blend_out),
        .at_pass   (pp_blend_at));

    // -------------------- orchestration FSM --------------------
    // Decoupled producer / consumer (as frontend_isp_tb_top): the region->objlist
    // path pushes OL entries into an 8-deep entry FIFO (eq); a concurrent iterator-
    // consumer (it_cst) drains eq, runs the iterator, and pushes triangles into an
    // 8-deep triangle FIFO (fq); a concurrent setup||raster consumer pops fq and
    // rasterizes into the tile buffer. BARRIER: at every region-state boundary the
    // producer waits for both FIFOs empty AND the consumer idle so CLEAR/OP/PT/TR/
    // FLUSH stay strictly ordered on the shared tile depth/tag buffer.
    localparam S_IDLE=0, S_RA=1, S_STATE=2,
               S_OL_RUN=4,                 // producer: OL entries -> entry FIFO
               S_RA_ACK=9, S_DONE=10,
               S_DRAIN=11,                 // barrier: wait consumer idle + FIFOs empty
               // shade pass (FLUSH): producer walks pixels, feeds tsp_shade_pp
               SH_PIX=16, SH_LOOK=17,
               FH_ISP=18, FH_ISPW=19, FH_TSPW=20, FH_TCWW=21,
               FV_RD=22, FV_W=23,
               TSP_RUN=24, SH_PRESENT=25, SH_DRAIN=26, SH_OUT=27,
               // unified PT+TL peel loop: init pb2, PeelBuffers, run, check
               S_PEEL_INIT=28, S_PEEL_BUF=29, S_PEEL_CHK=30,
               // shared shade sub-phase entry + OP-done return + FLUSH writeout
               S_SHADE_START=31, S_OP_DONE=32, S_FLUSH_WR=33,
               // M10K bulk-op walks over the peel RAM ports (128 chunk addrs each):
               S_CLEAR_WR=34,              // CLEAR: write {bg_depth, bg_tag} chunks
               S_PEEL_BUF_RUN=35,          // PeelBuffers RMW walk (read A -> write B)
               // shade producer: 1-cycle peel-RAM read latency for pixel shp
               SH_RD=36,
               S_FLUSH_RD=37;              // FLUSH: prime col-RAM read of pixel fw_i
    reg [5:0] st;

    // consumer sub-FSM (setup is now the streamed pipeline u_isp -> pq FIFO)
    localparam RS_IDLE=0, RS_RAS=1, RS_DRAIN=2;  reg [1:0] rs_st;

    // ---- unified PT+TL layer-peel pass loop (TB-FSM; refsw do..while(MoreToDraw)) ----
    reg        more_to_draw;      // set by the raster consumer during a peel pass
    reg        op_shaded;         // OP shade (background/opaque -> col_buf) done this tile
    reg [31:0] pt_ptr_l, tr_ptr_l;// latched PT / TL list pointers for this tile
    reg        has_pt,  has_tr;   // this tile has a PT / TL list
    reg        peel_which;        // 0 = rasterizing PT list, 1 = TL list (this pass)
    reg [7:0]  peel_pass;         // pass counter (safety bound)
    localparam integer PEEL_MAX_PASS = 64;

    // ---- M10K bulk-op walk counters (128 chunk addresses = whole 32x32 tile) ----
    reg [6:0]  cl_i;              // CLEAR chunk-address counter 0..127
    reg [6:0]  pb_i;             // PeelBuffers chunk-address counter 0..127
    reg [6:0]  pb_rd;            // PeelBuffers read-ahead chunk (1 ahead of pb_i)
    reg        pb_pipe;         // PeelBuffers RMW pipe primed (stage-B has valid rdata)
    reg        first_peel;      // this tile's FIRST PeelBuffers (folds refsw SetTagToMax:
                                //   pb2 <- 0xFFFFFFFF instead of copying tagA)

    // ---- single shared shade sub-phase (ONE tsp_shade_pp pipeline) ----
    // Invoked as a subroutine after OP and after each peel pass. The TSP pipeline
    // ALWAYS blends (refsw PixelFlush_tsp always runs BlendingUnit); the tag's
    // SrcInstr/DstInstr decide the effect (opaque tags use ONE/ZERO = overwrite).
    // shade_mode only selects WHICH pixels are shaded, not whether we blend:
    //   0 = OP  : shade every pixel of the tile
    //   1 = PEEL: shade only pixels staged this pass (dt_valid)
    // shade_ret : FSM state to return to once the shade sub-phase drains.
    reg        shade_mode;
    reg [5:0]  shade_ret;
    integer    sh_pending;   // pixels presented this shade sub-phase (PEEL skips some)

    // ---- entry FIFO (object_list_parser -> iterator), depth 8 ----
    localparam integer EQ_N = 8;
    reg [1:0]       eq_etype [0:EQ_N-1];
    objlist_entry_t eq_entry [0:EQ_N-1];
    reg [3:0] eq_head, eq_tail; reg [4:0] eq_count;
    reg       eq_push, eq_pop;
    wire eq_full  = (eq_count == EQ_N);
    wire eq_empty = (eq_count == 0);

    // entry FIFO head -> prefetching iterator's streaming input. The iterator pulls
    // entries via entry_valid/entry_ack; the barrier observes list-done via
    // eq_empty && !it_pf_busy (no pull FSM, no it_cst / prim_seen tracking).
    always @(*) begin
        it_entry = eq_entry[eq_head[2:0]];
        it_etype = entry_type_e'(eq_etype[eq_head[2:0]]);
    end
    assign it_entry_valid = !eq_empty;

    // ---- triangle FIFO (producer -> consumer), depth 8 ----
    localparam integer FIFO_N = 8;
    reg [31:0] fq_isp [0:FIFO_N-1];
    reg [31:0] fq_tag [0:FIFO_N-1];
    reg [31:0] fq_x1[0:FIFO_N-1], fq_y1[0:FIFO_N-1], fq_z1[0:FIFO_N-1];
    reg [31:0] fq_x2[0:FIFO_N-1], fq_y2[0:FIFO_N-1], fq_z2[0:FIFO_N-1];
    reg [31:0] fq_x3[0:FIFO_N-1], fq_y3[0:FIFO_N-1], fq_z3[0:FIFO_N-1];
    reg [3:0]  fq_head, fq_tail;   // ring indices 0..FIFO_N-1 (0..7)
    reg [4:0]  fq_count;
    reg        fifo_push, fifo_pop; // 1-cycle intents (reconciled into fq_count)
    wire fq_full  = (fq_count == FIFO_N);
    wire fq_empty = (fq_count == 0);

    // ---- 8-deep PLANE FIFO (pq): streamed setup -> rasterizer ----
    // Decouples the (interleaved) setup from the rasterizer so setup runs ahead and
    // fills the FIFO instead of lock-stepping through the old 1-deep pend handoff.
    localparam integer PQ_N = 8;
    reg [31:0] pq_dx12[0:PQ_N-1],pq_dx23[0:PQ_N-1],pq_dx31[0:PQ_N-1],pq_dx41[0:PQ_N-1];
    reg [31:0] pq_dy12[0:PQ_N-1],pq_dy23[0:PQ_N-1],pq_dy31[0:PQ_N-1],pq_dy41[0:PQ_N-1];
    reg [31:0] pq_c1[0:PQ_N-1],pq_c2[0:PQ_N-1],pq_c3[0:PQ_N-1],pq_c4[0:PQ_N-1];
    reg [31:0] pq_ddx[0:PQ_N-1],pq_ddy[0:PQ_N-1],pq_cinvw[0:PQ_N-1];
    reg [31:0] pq_isp[0:PQ_N-1],pq_tag[0:PQ_N-1];
    reg [4:0]  pq_bx0[0:PQ_N-1],pq_bx1[0:PQ_N-1],pq_by0[0:PQ_N-1],pq_by1[0:PQ_N-1];
    reg [3:0]  pq_head, pq_tail;
    reg [4:0]  pq_count;
    reg        pq_push, pq_pop;
    wire pq_full  = (pq_count == PQ_N);
    wire pq_empty = (pq_count == 0);

    // consumer fully idle: entry FIFO empty, iterator authoritative-idle (it_pf_busy
    // clear: no record buffered/being read/emitted/outstanding), streamed setup
    // idle (!su_busy), raster idle, and the plane FIFO empty. Using it_pf_busy (not
    // a pulse-cleared reg) is the isp_core fix: a drained pulse racing a refilled eq
    // could open the barrier while triangles were still pending -> next state's
    // CLEAR/PeelBuffers/FLUSH would corrupt live tile data. The peel loop re-runs
    // the OL list per pass; the barrier waits for it_pf_busy to clear each pass.
    // include !b_valid: the depth-cmp write-back (stage B) is one cycle behind
    // ras_out_valid, so the last chunk's write to the peel RAM may still be in flight
    // when rs_st returns to RS_IDLE. CLEAR/PeelBuffers must not walk the RAM until it
    // lands (isp_core fix).
    wire consumer_idle = eq_empty && !it_pf_busy
                       && !su_busy && (rs_st==RS_IDLE) && pq_empty
                       && !b_valid;

    // shade pass pixel accounting: producer index shp, consumer count sh_out_n
    integer sh_out_n;
    // streamed rasterizer: chunks in flight (issued but not yet consumed)
    localparam integer NCHUNK = (TILE_W/RAS_LANES) * TILE_H;
    integer ras_inflight;

    // vertex field ids for FV_RD/FV_W
    localparam [2:0] FLD_X=0, FLD_Y=1, FLD_Z=2, FLD_UV16=3, FLD_U=4, FLD_V=5,
                     FLD_COL=6, FLD_OFS=7;

    integer tri_count, cull_count, miss_count, hit_count, tri_seen;
    reg [5:0] cur_tx, cur_ty;      // latched tile coords (stable during lists)
    integer i, l, j;

    // ---- COMBINATIONAL framebuffer-write (FLUSH), valid/ready handshake ----
    // fbw_req is presented + accepted the SAME cycle so a blocking controller works
    // (busy holds; we advance fw_i only when the pixel is consumed). The screen
    // pixel for the linear tile index fw_i (0..1023): x=fw_i[4:0], y=fw_i[9:5].
    // fbw_req.argb comes from the col RAM's REGISTERED read (cr_rdata): the combi
    // read-port block below presents cr_raddr=fw_i whenever we HOLD the pixel, so
    // cr_rdata == col_ram[fw_i]. On entry S_FLUSH_RD primes the read for pixel 0.
    wire [10:0] fw_px = {5'd0, cur_tx}*11'd32 + {6'd0, fw_i[4:0]};
    wire [10:0] fw_py = {5'd0, cur_ty}*11'd32 + {6'd0, fw_i[9:5]};
    wire        fw_onscreen = (fw_px < 11'd640) && (fw_py < 11'd480);
    always @(*) begin
        fbw_req.we      = (st==S_FLUSH_WR) && fw_onscreen;
        fbw_req.pix_idx = fw_py*20'd640 + {9'd0, fw_px};
        fbw_req.argb    = cr_rdata;
    end
    // a pixel is consumed this cycle when: on-screen and the sink accepted it, OR
    // off-screen (nothing to write -> skip immediately).
    wire fw_pix_consumed = (st==S_FLUSH_WR) &&
                           ( (fw_onscreen && !fbw_resp.busy) || !fw_onscreen );

    // ============ COMBINATIONAL peel-RAM control (address valid THIS cycle) ============
    // Presenting addresses combinationally makes the RAM's registered read give
    // exactly 1-cycle latency, so the stage-B consumer (next cycle) sees THIS
    // address's old data. Ports are shared across phases but the region barriers
    // serialize raster / shade / bulk, so only one driver is ever active.
    //
    //   READ  port: raster stage A (ras_out_valid) | shade producer (SH_PIX) |
    //               PeelBuffers read-ahead (pb_rd).
    //   WRITE port: raster stage B (b_valid) | CLEAR (cl_i) | PeelBuffers (pb_i).
    integer cw;
    always @(*) begin
        pr_we    = '0;
        pr_waddr = '0;
        pr_raddr = '0;
        pr_wdata = '0;

        // ---- READ port ----
        if (ras_out_valid)                       // stage A: chunk being resolved
            pr_raddr = pr_pack_addr(ras_oy, ras_ox);
        else if (st == SH_PIX)                   // shade producer: pixel shp
            pr_raddr = {NB{ {shp[9:5], shp[4:3]} }};
        else if (st == S_PEEL_BUF_RUN)           // PeelBuffers: read-ahead chunk
            pr_raddr = {NB{pb_rd}};

        // ---- WRITE port ----
        if (st == S_CLEAR_WR) begin              // CLEAR: {bg depth, bg tag} all banks
            pr_we    = {NB{1'b1}};
            pr_waddr = {NB{cl_i}};
            for (cw = 0; cw < NB; cw = cw + 1) begin
                pr_wdata[PEEL_W*cw + PW_DEPTH  +: 32] = regs.isp_backgnd_d;
                pr_wdata[PEEL_W*cw + PW_TAG    +: 32] = regs.isp_backgnd_t;
                // depth2/tag2/valid are don't-care for OP; PeelBuffers sets them.
            end
        end else if (st == S_PEEL_BUF_RUN && pb_pipe) begin
            // PeelBuffers(FLT_MAX,0) transform of the read-back chunk (pr_rdata):
            //   depth2 <- depth ; tag2 <- tag (or 0xFFFFFFFF on the tile's FIRST
            //   PeelBuffers, folding refsw SetTagToMax) ; depth <- FLT_MAX ;
            //   valid <- 0. (dt_pt is a reg, cleared in the FSM.)
            pr_we    = {NB{1'b1}};
            pr_waddr = {NB{pb_i}};
            for (cw = 0; cw < NB; cw = cw + 1) begin
                pr_wdata[PEEL_W*cw + PW_DEPTH  +: 32] = FLT_MAX;
                pr_wdata[PEEL_W*cw + PW_DEPTH2 +: 32] = pr_depth(pr_rdata, cw);
                pr_wdata[PEEL_W*cw + PW_TAG    +: 32] = pr_tag  (pr_rdata, cw);
                pr_wdata[PEEL_W*cw + PW_TAG2   +: 32] =
                    first_peel ? 32'hFFFFFFFF : pr_tag(pr_rdata, cw);
                pr_wdata[PEEL_W*cw + PW_VALID]        = 1'b0;
            end
        end else if (b_valid) begin              // stage B: depth-cmp write-back
            pr_waddr = pr_pack_addr(b_oy, b_ox);
            for (cw = 0; cw < NB; cw = cw + 1) begin
                if (b_inside[cw]) begin
                    if (b_peeling) begin
                        // layer-peel accept: zb<-invW, pb<-tag, valid<-1.
                        if (ras_pass_lp[cw]) begin
                            pr_we[cw] = 1'b1;
                            pr_wdata[PEEL_W*cw + PW_DEPTH  +: 32] = b_invw_lane(cw);
                            pr_wdata[PEEL_W*cw + PW_TAG    +: 32] = b_tag;
                            pr_wdata[PEEL_W*cw + PW_VALID]        = 1'b1;
                            // preserve depth2/tag2 (reference for this pass)
                            pr_wdata[PEEL_W*cw + PW_DEPTH2 +: 32] = pr_depth2(pr_rdata, cw);
                            pr_wdata[PEEL_W*cw + PW_TAG2   +: 32] = pr_tag2  (pr_rdata, cw);
                        end
                    end else begin
                        // opaque accept: DepthMode pass -> tag<-tag, depth<-invW
                        // (unless ZWriteDis). preserve the rest.
                        if (ras_pass_op[cw]) begin
                            pr_we[cw] = 1'b1;
                            pr_wdata[PEEL_W*cw + PW_DEPTH  +: 32] =
                                b_zwdis ? pr_depth(pr_rdata, cw) : b_invw_lane(cw);
                            pr_wdata[PEEL_W*cw + PW_TAG    +: 32] = b_tag;
                            pr_wdata[PEEL_W*cw + PW_DEPTH2 +: 32] = pr_depth2(pr_rdata, cw);
                            pr_wdata[PEEL_W*cw + PW_TAG2   +: 32] = pr_tag2  (pr_rdata, cw);
                            pr_wdata[PEEL_W*cw + PW_VALID]        = pr_valid (pr_rdata, cw);
                        end
                    end
                end
            end
        end
    end

    // ============ COMBINATIONAL color-RAM control ============
    //   READ  port: blend stage CA (cb read of out_id) | FLUSH read (fw_i).
    //   WRITE port: blend stage CB (cb_valid -> col_ram[cb_id] <- blend_out).
    // Barriers serialize shade vs FLUSH, so the read port is uncontended.
    always @(*) begin
        cr_we    = 1'b0;
        cr_waddr = 10'd0;
        cr_wdata = 32'd0;
        cr_raddr = 10'd0;

        // ---- READ port ----
        if (pp_out_valid && !pp_stall)           // stage CA: pre-read dst for out_id
            cr_raddr = pp_out_id;
        else if (st == S_FLUSH_RD || st == S_FLUSH_WR)
            cr_raddr = fw_i;                     // FLUSH: hold pixel address

        // ---- WRITE port ----
        if (cb_valid) begin                      // stage CB: blended pixel write-back
            cr_we    = 1'b1;
            cr_waddr = cb_id;
            cr_wdata = pp_blend_out;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            st<=S_IDLE; done<=0; ra_start<=0; ol_start<=0;
            tsp_start<=0; pp_in_valid<=0; f_go<=0;
            ra_ack.list_done<=0; ol_ack.entry_done<=0; it_ack.triangle_done<=0;
            tri_count<=0; cull_count<=0; miss_count<=0; hit_count<=0; tri_seen<=0;
            sh_out_n<=0; ras_inflight<=0;
            rs_st<=RS_IDLE;
            pq_head<=0; pq_tail<=0; pq_count<=0;
            fq_head<=0; fq_tail<=0; fq_count<=0;
            eq_head<=0; eq_tail<=0; eq_count<=0;
            peeling<=1'b0; more_to_draw<=1'b0; peel_pass<=8'd0; op_shaded<=1'b0;
            has_pt<=1'b0; has_tr<=1'b0; peel_which<=1'b0;
            b_valid<=1'b0; cb_valid<=1'b0;
            cl_i<=7'd0; pb_i<=7'd0; pb_rd<=7'd0; pb_pipe<=1'b0; first_peel<=1'b0;
            for (i = 0; i < PC_N; i = i + 1) pc_valid[i] = 1'b0;
        end else begin
            done<=0; ra_start<=0; ol_start<=0;
            tsp_start<=0; pp_in_valid<=0; f_go<=0;
            ra_ack.list_done<=0; ol_ack.entry_done<=0; it_ack.triangle_done<=0;
            eq_push = 1'b0;
            // fbw_req is driven COMBINATIONALLY (see the FLUSH write block below).

            // ============ streamed rasterizer CONSUMER: stage A -> stage B ============
            // Stage A (ras_out_valid): the combinational peel-RAM read of chunk
            // {ras_oy,ras_ox} is already presented; latch the result fields for the
            // compare next cycle. Stage B (b_valid): the combinational RAM block runs
            // the depth compare off pr_rdata and writes the passing lanes back to the
            // peel RAM. Here in stage B we also write the dt_pt REG (kept out of the
            // RAM) and accumulate more_to_draw (moved off the old combinational path).
            b_valid <= 1'b0;
            if (ras_out_valid) begin
                b_valid  <= 1'b1;
                b_inside <= ras_inside;
                b_invw   <= ras_invw_flat;
                b_ox     <= ras_ox;
                b_oy     <= ras_oy;
                b_tag    <= tri_tag;
                b_mode   <= depth_mode;
                b_zwdis  <= zwrite_dis;
                b_peeling<= peeling;
                b_which  <= peel_which;
            end
            if (b_valid && b_peeling) begin
                for (l = 0; l < RAS_LANES; l = l + 1) begin
                    /* verilator lint_off WIDTH */
                    if (b_inside[l]) begin
                        // dt_pt: winning fragment came from PT list (b_which==0).
                        if (ras_pass_lp[l])
                            dt_pt[{27'd0,b_oy}*TILE_W + {27'd0,b_ox} + l] = (b_which==1'b0);
                        // MoreToDraw: another peel pass needed (refsw do..while).
                        if (ras_more_lp[l]) more_to_draw <= 1'b1;
                    end
                    /* verilator lint_on WIDTH */
                end
            end
            ras_inflight <= ras_inflight + (ras_in_valid ? 1 : 0) - (ras_out_valid ? 1 : 0);

            // ============ shade pipeline CONSUMER: blend stage CA -> CB ============
            // A fresh result is present when out_valid && !stall (out_valid holds
            // through a stall since the whole pipe is frozen). Stage CA latches the
            // pixel + snapshots the alpha-test enable (peeling && dt_pt[id], dt_pt is
            // a reg), presents the col-RAM read of out_id (combi block), and counts it
            // as drained. Stage CB (cb_valid): cr_rdata = OLD col_buf[id] = dst, the
            // blend runs combinationally, and the combi block writes col_ram[id].
            cb_valid <= 1'b0;
            if (pp_out_valid && !pp_stall) begin
                cb_valid <= 1'b1;
                cb_id    <= pp_out_id;
                cb_argb  <= pp_out_argb;
                cb_tsp   <= pp_out_tsp;
                cb_at_en <= peeling && dt_pt[pp_out_id];   // PT alpha-test enable
                sh_out_n <= sh_out_n + 1;
            end

            case (st)
            S_IDLE: if (go) begin ra_start<=1; st<=S_RA; end

            S_RA: begin
                if (ra_tiles_parsed) st<=S_DONE;
                else if (ra_out.list_ready) begin
                    cur_tx <= ra_out.tile_x; cur_ty <= ra_out.tile_y;
                    st<=S_STATE;
                end
            end

            S_STATE: begin
                // tile origin (floats) for both isp_setup and tsp_setup
                t_xbase <= i2f({10'd0, cur_tx} * 16'd32);
                t_ybase <= i2f({10'd0, cur_ty} * 16'd32);
                case (ra_out.state)
                // CLEAR touches the whole tile buffer: BARRIER first (consumer of
                // the previous state must be fully done). The M10K CLEAR is a 128-
                // chunk write walk (S_CLEAR_WR) instead of a single-cycle for-loop.
                RSTATE_CLEAR: if (consumer_idle && fq_empty) begin
                    // as tile_engine_top TILE_CLEAR: {bg depth, bg CoreTag}. Every
                    // pixel's tag = background tag and is "valid" for OP shading
                    // (refsw ClearBuffers sets tagStatus.valid=true), so the OP
                    // shade fills col_buf with the background color.
                    op_shaded <= 1'b0;   // OP shade not yet run for this tile
                    has_pt <= 1'b0; has_tr <= 1'b0;   // PT/TL lists for this tile: none yet
                    cl_i <= 7'd0; st <= S_CLEAR_WR;
                end
                // OPAQUE: single pass, plain DepthMode compare (no peeling).
                RSTATE_OP: begin
                    peeling  <= 1'b0;
                    ol_list_ptr <= ra_out.list_ptr;
                    ol_start <= 1'b1;
                    st <= S_OL_RUN;
                end
                // PT / TR: just LATCH the list pointer + present flag and ack. The
                // UNIFIED peel (both lists together, back-to-front) runs at FLUSH so
                // PT fragments (opaque-or-hole via alpha test) correctly occlude TL
                // fragments behind them within the same peel passes.
                RSTATE_PT: begin
                    pt_ptr_l  <= ra_out.list_ptr; has_pt <= 1'b1;
                    ra_ack.list_done <= 1'b1; st <= S_RA_ACK;
                end
                RSTATE_TR: begin
                    tr_ptr_l  <= ra_out.list_ptr; has_tr <= 1'b1;
                    ra_ack.list_done <= 1'b1; st <= S_RA_ACK;
                end
                // FLUSH: (1) if any PT/TL geometry, run the unified peel now (blends
                // into col_buf); (2) then copy col_buf -> fb.
                RSTATE_FLUSH: if (consumer_idle && fq_empty) begin
                    if (has_pt || has_tr) begin
                        // ensure the OP/background shade ran so col_buf is the base
                        // image the peel layers blend over.
                        if (!op_shaded) begin
                            peeling    <= 1'b0;
                            shade_mode <= 1'b0;
                            op_shaded  <= 1'b1;
                            shade_ret  <= S_PEEL_INIT;
                            st <= S_SHADE_START;
                        end else begin
                            peeling <= 1'b1;
                            st <= S_PEEL_INIT;
                        end
                    end else begin
                        fw_i <= 10'd0; st <= S_FLUSH_RD;   // prime col-RAM read of px 0
                    end
                end
                default: begin ra_ack.list_done <= 1'b1; st <= S_RA_ACK; end
                endcase
            end

            // CLEAR write walk: the combi block writes background {depth,tag} to all
            // 8 banks at address cl_i each cycle; here we just walk cl_i 0..127.
            S_CLEAR_WR: begin
                if (cl_i == 7'd127) begin ra_ack.list_done <= 1'b1; st <= S_RA_ACK; end
                else cl_i <= cl_i + 7'd1;
            end

            // S_OL_RUN: PRODUCER - push each OL entry into the entry FIFO (eq) and
            // ack the OL parser so it decodes the next entry ahead. STRIP/TRI are
            // queued; QUAD is skipped. On list end (ol_done) -> BARRIER (S_DRAIN).
            // The iterator CONSUMER (it_cst) runs concurrently, popping eq into the
            // triangle FIFO independent of `st`.
            S_OL_RUN: begin
                if (ol_done) st <= S_DRAIN;
                else if (ol_prim.entry_ready && !ol_ack.entry_done) begin
                    if (ol_prim.entry_type == ENT_STRIP ||
                        ol_prim.entry_type == ENT_TRI) begin
                        if (!eq_full) begin
                            eq_etype[eq_tail[2:0]] <= ol_prim.entry_type;
                            eq_entry[eq_tail[2:0]] <= ol_prim.entry;
                            eq_tail <= (eq_tail==EQ_N-1) ? 4'd0 : eq_tail+4'd1;
                            eq_push = 1'b1;
                            ol_ack.entry_done <= 1'b1;
                        end
                    end else begin
                        ol_ack.entry_done <= 1'b1;   // quad: skip (ack, don't queue)
                    end
                end
            end

            // BARRIER at list end: wait for the entry FIFO + iterator + triangle
            // FIFO + setup/raster to all drain before letting region advance.
            //  - OP  : run the OP shade sub-phase, then ack the region.
            //  - peel: run the peel shade sub-phase, then decide whether to peel again.
            S_DRAIN: if (fq_empty && consumer_idle) begin
                if (peeling) begin
                    // Unified peel: after the PT list, rasterize the TL list into the
                    // same buffers (if present) BEFORE shading, so both sort together.
                    if (peel_which==1'b0 && has_tr) begin
                        peel_which <= 1'b1;
                        ol_list_ptr <= tr_ptr_l; ol_start <= 1'b1;
                        st <= S_OL_RUN;
                    end else begin
                        shade_mode <= 1'b1;      // PEEL: shade only valid pixels
                        shade_ret  <= S_PEEL_CHK;
                        st <= S_SHADE_START;
                    end
                end else begin
                    shade_mode <= 1'b0;          // OP: shade every pixel
                    shade_ret  <= S_OP_DONE;
                    st <= S_SHADE_START;
                end
            end

            // OP shade drained -> mark this tile OP-shaded, ack the OP region.
            S_OP_DONE: begin op_shaded <= 1'b1; ra_ack.list_done <= 1'b1; st <= S_RA_ACK; end

            // -------- layer-peel pass loop (refsw2 RM_TRANSLUCENT_AUTOSORT) --------
            // refsw SetTagToMax(): set tagBufferA (dt_tag) = 0xFFFFFFFF for the whole
            // tile. The FIRST PeelBuffers then copies A->B so pb2 = 0xFFFFFFFF on
            // pass 1 (NOT the OP tags). depthA (dt_depth) still holds the OP depth,
            // which PeelBuffers copies to the reference zb2. Note: this discards the
            // OP tags in dt_tag, but the OP shade already ran (col_buf holds it), so
            // dt_tag is only used as the peel pending tag from here on.
            // refsw SetTagToMax() is FOLDED into the first PeelBuffers of this tile
            // (see S_PEEL_BUF_RUN + the combi block): instead of a separate walk
            // writing dt_tag=0xFFFFFFFF then copying A->B, the first PeelBuffers
            // writes tag2 <- 0xFFFFFFFF directly (first_peel), while still copying
            // depth2 <- depth (the OP depth reference). Net pass-1 state is identical:
            // pb2 = 0xFFFFFFFF, zb2 = OP depth.
            S_PEEL_INIT: begin
                peeling    <= 1'b1;  // (pre-peel OP shade may have cleared it)
                peel_pass  <= 8'd0;  // reset the pass counter for THIS tile's peel
                first_peel <= 1'b1;  // fold SetTagToMax into the first PeelBuffers
                st <= S_PEEL_BUF;
            end

            // PeelBuffers(FLT_MAX, 0): per pixel depth2<-depth, tag2<-tag (or
            // 0xFFFFFFFF on pass 1 via first_peel), depth<-FLT_MAX, valid<-0. As an
            // M10K RMW this is a read(A chunk N)->write(B transformed chunk N-1) walk
            // over 128 chunk addresses (S_PEEL_BUF_RUN). dt_pt is a reg: cleared here
            // in one cycle (else a stale PT flag fires the alpha test on TR pixels).
            // On pass 1 dt_depth holds the OP result (nothing clobbers it between OP
            // and the peel); later passes it holds the previous pass's closest layer;
            // copying live dt_depth every pass is correct - no snapshot needed.
            S_PEEL_BUF: begin
                for (i = 0; i < TILE_W*TILE_H; i = i + 1)
                    dt_pt[i] = 1'b0;
                more_to_draw <= 1'b0;
                pb_rd   <= 7'd0;     // read-ahead chunk
                pb_i    <= 7'd0;     // write chunk (1 behind read once primed)
                pb_pipe <= 1'b0;     // stage-B not yet primed
                st <= S_PEEL_BUF_RUN;
            end

            // PeelBuffers RMW walk: the combi block presents the READ of chunk pb_rd
            // and (when pb_pipe) WRITES the transformed chunk pb_i (= previous pb_rd,
            // whose data is now in pr_rdata). Advance read, delay write by one, finish
            // after the last chunk (127) has been written back.
            S_PEEL_BUF_RUN: begin
                pb_pipe <= 1'b1;
                pb_i    <= pb_rd;
                if (pb_pipe && pb_i == 7'd127) begin
                    // whole tile transformed -> issue the object list for this pass.
                    first_peel <= 1'b0;
                    peel_pass  <= peel_pass + 8'd1;
                    // start with the PT list if present, else the TL list.
                    if (has_pt) begin
                        peel_which <= 1'b0;                 // rasterizing PT list
                        ol_list_ptr <= pt_ptr_l; ol_start <= 1'b1;
                    end else begin
                        peel_which <= 1'b1;                 // TL list
                        ol_list_ptr <= tr_ptr_l; ol_start <= 1'b1;
                    end
                    st <= S_OL_RUN;
                end else if (pb_rd != 7'd127) begin
                    pb_rd <= pb_rd + 7'd1;
                end
            end

            // After a pass's peel-shade drained: another peel pass if a fragment was
            // deferred (more_to_draw) and under the bound; else write out.
            S_PEEL_CHK: begin
                if (more_to_draw && peel_pass < PEEL_MAX_PASS[7:0]) begin
                    st <= S_PEEL_BUF;
                end else begin
                    peeling <= 1'b0;
                    fw_i <= 10'd0; st <= S_FLUSH_RD;   // prime col-RAM read of px 0
                end
            end

            // FLUSH read: prime the col-RAM read of pixel fw_i (combi block presents
            // cr_raddr=fw_i); cr_rdata is valid next cycle in S_FLUSH_WR.
            S_FLUSH_RD: st <= S_FLUSH_WR;

            // FLUSH writeout: STREAM the accumulation buffer (col_ram) to the
            // framebuffer at ONE PIXEL PER CYCLE. fbw_req.argb comes from cr_rdata
            // (the registered read of fw_i, held stable while fw_i holds); here we
            // only ADVANCE fw_i, and only when the pixel is consumed (fw_pix_consumed):
            // on-screen pixel accepted (we && !busy) OR an off-screen pixel (skipped).
            // A busy on-screen pixel HOLDS fw_i so the combinational fbw_req keeps
            // presenting it -> works with a real controller that blocks. When fw_i
            // advances, the combi read of the NEW fw_i is presented THIS cycle, so
            // cr_rdata is ready next cycle: stay in S_FLUSH_WR (the just-consumed
            // pixel is done; the next pixel's data lands before we re-present it).
            S_FLUSH_WR: begin
                if (fw_pix_consumed) begin
                    if (fw_i == 10'd1023) begin ra_ack.list_done <= 1'b1; st <= S_RA_ACK; end
                    else begin fw_i <= fw_i + 10'd1; st <= S_FLUSH_RD; end
                end
            end

            // ---------------- shade sub-phase (shared, ONE tsp pipeline) ----------
            // Invalidate the tile-local plane cache, reset the pixel walk, and
            // start streaming pixels. shade_mode selects which pixels; shade_ret is
            // where we go when the pipe drains.
            S_SHADE_START: begin
                for (i = 0; i < PC_N; i = i + 1) pc_valid[i] = 1'b0;
                shp <= 10'd0; sh_out_n <= 0; sh_pending <= 0;
                st <= SH_PIX;
            end

            // PRODUCER: resolve pixel shp's planes (plane cache; miss = fetch +
            // tsp_setup) then present it to the pipelined shader. The CONSUMER
            // (blend stages CA/CB) writes results into col_ram by id. In PEEL mode,
            // pixels not staged this pass (dt_valid==0) are skipped.
            //
            // The peel buffers are now M10K (registered read): SH_PIX only PRESENTS
            // the read of pixel shp's chunk (combi block, pr_raddr for st==SH_PIX);
            // SH_RD (next cycle) samples pr_rdata lane shp[2:0] for valid/tag/depth.
            // shp does not change between SH_PIX and SH_RD, so shp[2:0] is stable.
            SH_PIX: st <= SH_RD;

            SH_RD: begin
                if (shade_mode && !pr_valid(pr_rdata, shp[2:0])) begin
                    // skip: nothing staged for this pixel this pass
                    if (shp == 10'd1023) st <= SH_DRAIN;
                    else begin shp <= shp + 10'd1; st <= SH_PIX; end
                end else begin
                    sh_tag     <= pr_tag  (pr_rdata, shp[2:0]);
                    sh_invw    <= pr_depth(pr_rdata, shp[2:0]);
                    sh_pending <= sh_pending + 1;
                    st <= SH_LOOK;
                end
            end

            // plane-cache lookup by the full tag (cache_bypass ignored)
            SH_LOOK: begin
                if (pc_valid[sh_slot] && pc_tag[sh_slot] == sh_tag) begin
                    hit_count <= hit_count + 1;
                    cur_isp = pc_isp[sh_slot];
                    cur_tsp = pc_tsp[sh_slot];
                    cur_tcw = pc_tcw[sh_slot];
                    for (j = 0; j < 10; j = j + 1) begin
                        cur_ddx[j] = pc_ddx[sh_slot][j];
                        cur_ddy[j] = pc_ddy[sh_slot][j];
                        cur_c[j]   = pc_c[sh_slot][j];
                    end
                    st <= SH_PRESENT;
                end else begin
                    miss_count <= miss_count + 1;
                    f_rec <= param_base + {4'd0, sh_po, 2'b00};
                    st <= FH_ISP;
                end
            end

            // ---- param record fetch (refsw GetFpuEntry / decode_pvr_vertices) ----
            FH_ISP:  begin f_addr<=f_rec; f_go<=1'b1; st<=FH_ISPW; end
            FH_ISPW: if (f_word_v) begin
                        cur_isp = f_word;
                        f_addr<=f_rec+27'd4; f_go<=1'b1; st<=FH_TSPW;
                    end
            FH_TSPW: if (f_word_v) begin
                        cur_tsp = f_word;
                        f_addr<=f_rec+27'd8; f_go<=1'b1; st<=FH_TCWW;
                    end
            FH_TCWW: if (f_word_v) begin
                        cur_tcw = f_word;
                        // first vertex = tag_offset (in-order, refsw GetFpuEntry)
                        f_vtx <= f_rec + (sh_two_vol ? 27'd20 : 27'd12)
                                       + {22'd0, sh_toff} * sh_stride_b;
                        fv_i  <= 2'd0;
                        fv_fld<= FLD_X;
                        // zero all fields (disabled planes/attrs read as 0)
                        for (j = 0; j < 3; j = j + 1) begin
                            fv_u[j]=32'd0; fv_v[j]=32'd0; fv_col[j]=32'd0; fv_ofs[j]=32'd0;
                        end
                        st <= FV_RD;
                    end

            // issue the read for the current field of the current vertex
            FV_RD: begin
                case (fv_fld)
                FLD_X:    f_ptr = f_vtx;
                FLD_Y:    f_ptr = f_vtx + 27'd4;
                FLD_Z:    f_ptr = f_vtx + 27'd8;
                FLD_UV16,
                FLD_U:    f_ptr = f_vtx + 27'd12;
                FLD_V:    f_ptr = f_vtx + 27'd16;
                // col follows xyz+uv; ofs follows col
                FLD_COL:  f_ptr = f_vtx + 27'd12
                                + (f_texture ? (f_uv16 ? 27'd4 : 27'd8) : 27'd0);
                default:  f_ptr = f_vtx + 27'd16
                                + (f_texture ? (f_uv16 ? 27'd4 : 27'd8) : 27'd0); // FLD_OFS
                endcase
                f_addr <= f_ptr; f_go <= 1'b1; st <= FV_W;
            end

            FV_W: if (f_word_v) begin
                case (fv_fld)
                FLD_X: begin fv_x[fv_i] = f_word; fv_fld <= FLD_Y;  st <= FV_RD; end
                FLD_Y: begin fv_y[fv_i] = f_word; fv_fld <= FLD_Z;  st <= FV_RD; end
                FLD_Z: begin
                    fv_z[fv_i] = f_word;
                    if (f_texture) begin
                        fv_fld <= f_uv16 ? FLD_UV16 : FLD_U; st <= FV_RD;
                    end else begin fv_fld <= FLD_COL; st <= FV_RD; end
                end
                FLD_UV16: begin
                    // DC 16-bit UV: each half is the top 16 bits of an f32
                    fv_u[fv_i] = {f_word[31:16], 16'd0};
                    fv_v[fv_i] = {f_word[15:0],  16'd0};
                    fv_fld <= FLD_COL; st <= FV_RD;
                end
                FLD_U: begin fv_u[fv_i] = f_word; fv_fld <= FLD_V;   st <= FV_RD; end
                FLD_V: begin fv_v[fv_i] = f_word; fv_fld <= FLD_COL; st <= FV_RD; end
                FLD_COL: begin
                    fv_col[fv_i] = f_word;
                    if (f_offset) begin fv_fld <= FLD_OFS; st <= FV_RD; end
                    else if (fv_i == 2'd2) begin
                        // all 3 vertices in: zero the planes (disabled attrs
                        // stay 0) and kick tsp_setup_min
                        for (j = 0; j < 10; j = j + 1) begin
                            cur_ddx[j]=32'd0; cur_ddy[j]=32'd0; cur_c[j]=32'd0;
                            pc_ddx[sh_slot][j]=32'd0; pc_ddy[sh_slot][j]=32'd0;
                            pc_c[sh_slot][j]=32'd0;
                        end
                        tsp_start <= 1'b1; st <= TSP_RUN;
                    end
                    else begin fv_i <= fv_i + 2'd1; f_vtx <= f_vtx + sh_stride_b;
                               fv_fld <= FLD_X; st <= FV_RD; end
                end
                default: begin // FLD_OFS
                    fv_ofs[fv_i] = f_word;
                    if (fv_i == 2'd2) begin
                        for (j = 0; j < 10; j = j + 1) begin
                            cur_ddx[j]=32'd0; cur_ddy[j]=32'd0; cur_c[j]=32'd0;
                            pc_ddx[sh_slot][j]=32'd0; pc_ddy[sh_slot][j]=32'd0;
                            pc_c[sh_slot][j]=32'd0;
                        end
                        tsp_start <= 1'b1; st <= TSP_RUN;
                    end
                    else begin fv_i <= fv_i + 2'd1; f_vtx <= f_vtx + sh_stride_b;
                               fv_fld <= FLD_X; st <= FV_RD; end
                end
                endcase
            end

            // wait for tsp_setup_min; planes stream into cur_* AND the cache
            // via the tsp_pvalid capture below; on done, commit the cache meta.
            TSP_RUN: if (tsp_done) begin
                pc_valid[sh_slot] = 1'b1;
                pc_tag[sh_slot]   = sh_tag;
                pc_isp[sh_slot]   = cur_isp;
                pc_tsp[sh_slot]   = cur_tsp;
                pc_tcw[sh_slot]   = cur_tcw;
                st <= SH_PRESENT;
            end

            // present pixel shp to the pipelined shader, HOLDING the inputs stable
            // and pp_in_valid asserted until the pipe actually accepts it
            // (pp_in_valid && !pp_stall). Advancing on the mere absence of stall in
            // the *previous* cycle races the pipe: a stall rising the cycle
            // pp_in_valid goes high would drop the pixel. So drive & hold here, and
            // only advance once accepted.
            SH_PRESENT: begin
                pp_in_valid <= 1'b1;   // override the top-of-block default (hold)
                pp_in_id    <= shp;
                pp_px       <= shp[4:0];
                pp_py       <= shp[9:5];
                pp_invw     <= sh_invw;
                pp_tsp      <= cur_tsp;
                pp_tcw      <= cur_tcw;
                pp_ptex     <= cur_isp[ISP_TEXTURE_BIT];
                pp_pofs     <= cur_isp[ISP_OFFSET_BIT];
                for (j = 0; j < 10; j = j + 1) begin
                    pp_ddx[j] <= cur_ddx[j];
                    pp_ddy[j] <= cur_ddy[j];
                    pp_c[j]   <= cur_c[j];
                end
                // accepted this cycle iff the pixel we're already driving is taken.
                // pp_in_valid reflects the PREVIOUS cycle's assertion; accept when
                // it was high and the pipe isn't stalled now.
                if (pp_in_valid && !pp_stall) begin
                    pp_in_valid <= 1'b0;      // consumed; stop driving this pixel
                    if (shp == 10'd1023) st <= SH_DRAIN;
                    else begin shp <= shp + 10'd1; st <= SH_PIX; end
                end
            end

            // all presented pixels drained: return to the caller (shade_ret).
            // sh_out_n counts blend stage-CA events; also wait !cb_valid so the
            // trailing stage-CB col_ram write lands before the next phase (peel pass
            // reads col via the blend dst, or FLUSH reads col_ram). The framebuffer
            // writeout happens later at RSTATE_FLUSH.
            SH_DRAIN: if (sh_out_n >= sh_pending && !cb_valid) begin
                st <= shade_ret;
            end

            S_RA_ACK: st <= S_RA;

            S_DONE: begin
                $display("=== done: %0d triangles rasterized, %0d culled, tsp$ %0d hits / %0d misses ===",
                         tri_count, cull_count, hit_count, miss_count);
                done<=1'b1; st<=S_IDLE;
            end
            default: st<=S_IDLE;
            endcase

            // ======== ENTRY FIFO -> prefetching iterator -> tri FIFO ========
            // The iterator pulls entries itself (entry_valid/entry_ack); we pop the
            // eq FIFO on ack and push its emitted triangles into fq. No it_cst pull
            // FSM / prim_seen: the barrier observes list-done via !it_pf_busy.
            eq_pop    = 1'b0;
            fifo_push = 1'b0;

            // entry FIFO pop when the iterator consumes the head entry
            if (it_entry_ack && !eq_empty) begin
                eq_head <= (eq_head==EQ_N-1) ? 4'd0 : eq_head+4'd1;
                eq_pop  = 1'b1;
            end

            // push emitted triangles into the tri FIFO (hold-until-space handshake)
            if (it_trio.triangle_ready && !fq_full && !it_ack.triangle_done) begin
                fq_isp[fq_tail[2:0]] <= it_trio.isp;
                fq_tag[fq_tail[2:0]] <= it_trio.tag;
                fq_x1[fq_tail[2:0]]<=it_trio.v0.x; fq_y1[fq_tail[2:0]]<=it_trio.v0.y; fq_z1[fq_tail[2:0]]<=it_trio.v0.z;
                fq_x2[fq_tail[2:0]]<=it_trio.v1.x; fq_y2[fq_tail[2:0]]<=it_trio.v1.y; fq_z2[fq_tail[2:0]]<=it_trio.v1.z;
                fq_x3[fq_tail[2:0]]<=it_trio.v2.x; fq_y3[fq_tail[2:0]]<=it_trio.v2.y; fq_z3[fq_tail[2:0]]<=it_trio.v2.z;
                it_ack.triangle_done <= 1'b1;   // advance iterator to next tri
                fq_tail  <= (fq_tail==FIFO_N-1) ? 4'd0 : fq_tail+4'd1;
                fifo_push = 1'b1;
                tri_seen <= tri_seen + 1;
                if (tri_seen % 100 == 0)
                    $display("[TILE %0d,%0d] TRI %0d tag=%08h isp=%08h",
                        cur_tx, cur_ty, tri_seen, it_trio.tag, it_trio.isp);
            end

            // ============ CONSUMER: tri FIFO -> streamed setup -> plane FIFO ============
            fifo_pop = 1'b0;
            pq_push  = 1'b0;

            // present a triangle to the streaming setup; pop fq when accepted.
            // su_in_valid is assigned combinationally outside the always block.
            if (su_in_valid && su_in_ready) begin
                fq_head <= (fq_head==FIFO_N-1) ? 4'd0 : fq_head+4'd1;
                fifo_pop = 1'b1;
            end

            // retire: on out_valid, push non-culled triangles into the plane FIFO.
            if (su_out_valid) begin
                if (isp_cull) begin
                    cull_count <= cull_count + 1;
                end else begin
                    pq_dx12[pq_tail[2:0]]<=w_dx12; pq_dx23[pq_tail[2:0]]<=w_dx23;
                    pq_dx31[pq_tail[2:0]]<=w_dx31; pq_dx41[pq_tail[2:0]]<=w_dx41;
                    pq_dy12[pq_tail[2:0]]<=w_dy12; pq_dy23[pq_tail[2:0]]<=w_dy23;
                    pq_dy31[pq_tail[2:0]]<=w_dy31; pq_dy41[pq_tail[2:0]]<=w_dy41;
                    pq_c1[pq_tail[2:0]]<=w_c1; pq_c2[pq_tail[2:0]]<=w_c2;
                    pq_c3[pq_tail[2:0]]<=w_c3; pq_c4[pq_tail[2:0]]<=w_c4;
                    pq_ddx[pq_tail[2:0]]<=w_ddx; pq_ddy[pq_tail[2:0]]<=w_ddy;
                    pq_cinvw[pq_tail[2:0]]<=w_cinvw;
                    pq_isp[pq_tail[2:0]]<=su_out_isp; pq_tag[pq_tail[2:0]]<=su_out_tag;
                    pq_bx0[pq_tail[2:0]]<=w_bx0; pq_bx1[pq_tail[2:0]]<=w_bx1;
                    pq_by0[pq_tail[2:0]]<=w_by0; pq_by1[pq_tail[2:0]]<=w_by1;
                    pq_tail <= (pq_tail==PQ_N-1) ? 4'd0 : pq_tail+4'd1;
                    pq_push  = 1'b1;
                end
            end

            // ---- RASTER: pop plane FIFO -> active planes -> BOUNDING-BOX sweep ----
            // Only sweep the chunks/rows the triangle's tile-local bbox covers.
            // x bounds are chunk-aligned (down to a RAS_LANES-wide chunk); rows go
            // by0..by1 inclusive. The rasterizer's inside-test still gates writes.
            pq_pop = 1'b0;
            case (rs_st)
            RS_IDLE: if (!pq_empty) begin
                isp_dx12<=pq_dx12[pq_head[2:0]]; isp_dx23<=pq_dx23[pq_head[2:0]];
                isp_dx31<=pq_dx31[pq_head[2:0]]; isp_dx41<=pq_dx41[pq_head[2:0]];
                isp_dy12<=pq_dy12[pq_head[2:0]]; isp_dy23<=pq_dy23[pq_head[2:0]];
                isp_dy31<=pq_dy31[pq_head[2:0]]; isp_dy41<=pq_dy41[pq_head[2:0]];
                isp_c1<=pq_c1[pq_head[2:0]]; isp_c2<=pq_c2[pq_head[2:0]];
                isp_c3<=pq_c3[pq_head[2:0]]; isp_c4<=pq_c4[pq_head[2:0]];
                isp_ddx_invw<=pq_ddx[pq_head[2:0]]; isp_ddy_invw<=pq_ddy[pq_head[2:0]];
                isp_c_invw<=pq_cinvw[pq_head[2:0]];
                isp_word<=pq_isp[pq_head[2:0]]; tri_tag<=pq_tag[pq_head[2:0]];
                pq_head <= (pq_head==PQ_N-1) ? 4'd0 : pq_head+4'd1;
                pq_pop  = 1'b1;
                tri_count<=tri_count+1;
                // chunk-aligned x range + row range from the bbox
                rbx0 <= pq_bx0[pq_head[2:0]] & 5'(~(RAS_LANES-1));
                rbx1 <= pq_bx1[pq_head[2:0]] & 5'(~(RAS_LANES-1));
                rby1 <= pq_by1[pq_head[2:0]];
                ras_y <= pq_by0[pq_head[2:0]];
                ras_x <= pq_bx0[pq_head[2:0]] & 5'(~(RAS_LANES-1));
                rs_st <= RS_RAS;
            end
            RS_RAS: begin
                if (ras_x == rbx1) begin
                    ras_x <= rbx0;
                    if (ras_y == rby1) rs_st <= RS_DRAIN;
                    else ras_y <= ras_y + 5'd1;
                end else begin
                    ras_x <= ras_x + 5'(RAS_LANES);
                end
            end
            // also wait for the depth-cmp write-back (stage B, b_valid) to land, else
            // the NEXT triangle's stage-A read races this triangle's last stage-B
            // write to the same peel-RAM word (RAW -> stale depth -> corruption).
            RS_DRAIN: if (ras_inflight==0 && !ras_in_valid && !ras_out_valid
                          && !b_valid) rs_st<=RS_IDLE;
            endcase

            // ---- FIFO count maintenance (single update; push/pop may coincide) ----
            fq_count <= fq_count + (fifo_push ? 5'd1 : 5'd0) - (fifo_pop ? 5'd1 : 5'd0);
            eq_count <= eq_count + (eq_push  ? 5'd1 : 5'd0) - (eq_pop   ? 5'd1 : 5'd0);
            pq_count <= pq_count + (pq_push  ? 5'd1 : 5'd0) - (pq_pop   ? 5'd1 : 5'd0);

            // plane stream capture (runs regardless of FSM state)
            if (tsp_pvalid) begin
                cur_ddx[tsp_pidx] = tsp_pddx;
                cur_ddy[tsp_pidx] = tsp_pddy;
                cur_c[tsp_pidx]   = tsp_pc;
                pc_ddx[sh_slot][tsp_pidx] = tsp_pddx;
                pc_ddy[sh_slot][tsp_pidx] = tsp_pddy;
                pc_c[sh_slot][tsp_pidx]   = tsp_pc;
            end
        end
    end
endmodule
