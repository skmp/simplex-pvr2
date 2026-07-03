//
// tex_fetch_pp - STREAMING single-texel fetch/decode over the pipelined tex_cache_4p
// port protocol. Accepts a NEW texel request EVERY cycle (valid/ready) and emits
// results IN ISSUE ORDER; it stalls (deasserts in_ready) ONLY when the underlying
// cache stalls on a genuine miss-fill. On an all-hit stream it sustains 1 texel/clk.
//
// Same texel math as before (tex_addr -> data cache -> optional VQ codebook read ->
// palette/decode). ORDERING IS FREE: the cache returns acks strictly in issue order
// per port, so a small side-data FIFO popped on ack reconstructs each result. There
// is exactly ONE path:
//
//   ISSUE : every accepted pixel issues a data-cache read (tc_req). NON-textured
//           pixels also issue one (harmless read; their decoded colour is forced to 0)
//           so they stay in the same in-order stream as textured pixels - no bypass
//           reorder logic. Side data (pixfmt/scan/offset/vq/palsel/vq_bytesel/tcw_addr
//           + a "textured" bit) is pushed into fifo_tc.
//   TC-ACK: tc_resp.ack pops fifo_tc (in order). VQ pixel -> issue vq_req from memtel,
//           push side into fifo_vq. Non-VQ (or non-textured) -> straight to DECODE.
//   VQ-ACK: vq_resp.ack pops fifo_vq -> memtel = vq codebook word -> DECODE.
//   DECODE: tex_decode(memtel, side) -> argb (forced 0 if !textured); out_valid.
//
// Because tcw (hence VQ-ness) is uniform across the four bilinear corners and, within
// a shade sub-phase, constant per triangle, the tc ack stream and the vq ack stream
// are each individually in-order and never collide (a non-VQ tc-ack and a vq-ack in
// the same cycle cannot both correspond to real texels of one uniform texture). The
// vq path is prioritised on the (workload-impossible) collision.
//
// PROTOCOL:
//   in_valid    : a new texel request is presented this clock
//   in_textured : 0 = non-textured pixel (flows through, argb forced to 0)
//   in_ready    : the fetcher can accept it this clock (== cache can accept a tc req)
//   out_valid   : argb corresponds to a completed fetch this clock (in issue order)
//
module tex_fetch_pp import tsp_pkg::*; (
    input             clk,
    input             reset,
    input             in_valid,
    input             in_textured,  // 0 = non-textured pixel: flow through, argb=0
    output            in_ready,
    input      [10:0] u,
    input      [10:0] v,
    input      [3:0]  miplevel,     // selected mip level (0 = base)
    input      [31:0] tsp,
    input      [31:0] tcw,
    input      [4:0]  text_ctrl,
    output reg        out_valid,
    output reg [31:0] argb,

    // injected caches: data (texel/index) + VQ codebook (tex_cache_4p port protocol)
    output cache_req_t   tc_req,
    input  cache_resp_t  tc_resp,
    output cache_req_t   vq_req,
    input  cache_resp_t  vq_resp
);
    // ---- decode of the CURRENT (incoming) request; tex_addr is combinational ----
    wire [2:0]  in_texu = tsp[5:3], in_texv = tsp[2:0];
    wire [20:0] in_tcw_addr = tcw[20:0];
    wire        in_strdsel = tcw[25], in_scan = tcw[26];
    wire [2:0]  in_pixfmt = tcw[29:27];
    wire        in_vq = tcw[30];
    wire        in_mipmapped = tcw[31];
    wire [5:0]  in_palsel = tcw[26:21];

    wire [28:0] ta_byte; wire [2:0] ta_fbpp_shr; wire [19:0] ta_off;
    tex_addr u_ta (
        .tcw_addr(in_tcw_addr),.vq(in_vq),.scan(in_scan),.stride_sel(in_strdsel),
        .mipmapped(in_mipmapped),.pixfmt(in_pixfmt),
        .texu(in_texu),.texv(in_texv),.miplevel(miplevel),.text_ctrl(text_ctrl),
        .u(u),.v(v),
        .byte_addr(ta_byte),.fbpp_shr(ta_fbpp_shr),.offset(ta_off));

    // side-data bundle carried with each request. Field layout (MSB..LSB):
    //   [42] textured  [41:39] pixfmt  [38] scan  [37:34] off_lo  [33:31] off_bytesel
    //   [30] vq  [29:27] vq_bytesel  [26:6] tcw_addr  [5:0] palsel
    localparam integer SW = 1+3+1+4+3+1+3+21+6;   // = 43
    wire [SW-1:0] in_side = { in_textured, in_pixfmt, in_scan, ta_off[3:0], ta_byte[2:0],
                              in_vq, ta_byte[2:0], in_tcw_addr, in_palsel };

    // fifo depths. Must have headroom for the whole in-flight window; a VQ pixel
    // occupies tc, then vp, then vq before completing, so gate issue on ALL having room.
    localparam integer FD = 16, FAW = 4;

    // fifo_tc: side data for tc requests in flight (issue order).
    reg  [SW-1:0] tcf [0:FD-1];
    reg  [FAW-1:0] tc_h, tc_t; reg [FAW:0] tc_cnt;

    // ============ ISSUE ============
    // Accept only when the data cache is ready AND every downstream FIFO has room, so
    // no in-flight entry is ever dropped (which would deadlock the corner-join upstream).
    wire tc_room;   // forward refs, assigned once all fifo counts exist
    wire vp_room;
    wire vq_room;
    assign in_ready   = tc_resp.ready && tc_room && vp_room && vq_room;
    wire   accept     = in_valid && in_ready;
    assign tc_req.req   = accept;                    // every accepted pixel reads tc
    assign tc_req.waddr = { 3'b0, ta_byte[28:3] };   // 64-bit word addr = byte>>3
    assign tc_room = (tc_cnt < FD-2);
    wire tc_push = accept;
    wire tc_pop  = tc_resp.ack;

    // ============ TC-ACK -> VQ or DECODE ============
    wire [SW-1:0] tc_side   = tcf[tc_h];
    wire          tc_txd    = tc_side[42];
    wire          tc_is_vq  = tc_side[30] && tc_txd;   // non-textured never chains vq
    wire [2:0]    tc_vqbsel = tc_side[29:27];
    wire [20:0]   tc_taddr  = tc_side[26:6];
    wire [63:0]   tc_memtel = tc_resp.rdata;
    wire [7:0]    vq_byte   = tc_memtel[8*tc_vqbsel +: 8];
    wire [28:0]   vq_addr   = {8'd0, tc_taddr} + {21'd0, vq_byte};

    // ---- VQ-PENDING FIFO: a VQ tc-ack derives a vq codebook address, but the vq cache
    // may not be ready THIS cycle (it could be filling). We cannot defer the tc ack, so
    // we buffer {vq_addr, side} here and issue the vq_req later, gated on vq_resp.ready.
    // This decouples the two streaming caches. ----
    reg  [28:0]   vpf_addr [0:FD-1];
    reg  [SW-1:0] vpf_side [0:FD-1];
    reg  [FAW-1:0] vp_h, vp_t; reg [FAW:0] vp_cnt;
    assign vp_room = (vp_cnt < FD-2);
    wire vp_ne   = (vp_cnt != 0);
    wire vp_push = tc_pop && tc_is_vq;            // a VQ tc-ack enqueues a pending vq req
    // issue the head pending vq req when the vq cache can accept it
    assign vq_req.req   = vp_ne && vq_resp.ready;
    assign vq_req.waddr = vpf_addr[vp_h];
    wire vp_pop  = vq_req.req;                     // pending entry consumed on issue

    // fifo_vq: side data for vq requests ISSUED (awaiting vq ack), in issue order
    reg  [SW-1:0] vqf [0:FD-1];
    reg  [FAW-1:0] vq_h, vq_t; reg [FAW:0] vq_cnt;
    assign vq_room = (vq_cnt < FD-2);
    wire vq_push = vq_req.req;                      // push side when the vq req is issued
    wire vq_pop  = vq_resp.ack;

    wire nonvq_to_decode = tc_pop && !tc_is_vq;   // non-VQ textured OR non-textured
    wire vq_to_decode    = vq_pop;

    // ============ DECODE stage register ============
    reg        d_v;
    reg [63:0] d_memtel;
    reg [SW-1:0] d_side;

    // palette ROM placeholder (ARGB8888) - matches tex_fetch
    (* rom_style = "block" *) reg [31:0] pal_rom [0:255];
    integer pri;
    initial for (pri=0; pri<256; pri=pri+1)
        pal_rom[pri] = {8'hFF, pri[7:0], pri[7:0], pri[7:0]};
    wire        d_txd    = d_side[42];
    wire [2:0]  d_pixfmt = d_side[41:39];
    wire        d_scan   = d_side[38];
    wire [3:0]  d_off_lo = d_side[37:34];
    wire [2:0]  d_off_b  = d_side[33:31];
    wire [5:0]  d_palsel = d_side[5:0];
    wire [7:0]  pal8_local = d_memtel[8*d_off_b +: 8];
    wire [3:0]  pal4_nib   = d_memtel[4*d_off_lo +: 4];
    wire [7:0]  pal8_idx   = {d_palsel[5:4], pal8_local};
    wire [7:0]  pal4_idx   = {d_palsel[3:0], pal4_nib};
    wire [7:0]  d_pal_idx  = (d_pixfmt==3'd6) ? pal8_idx :
                             (d_pixfmt==3'd5) ? pal4_idx : 8'd0;
    wire [31:0] d_pal_argb = pal_rom[d_pal_idx];

    wire [31:0] dec_argb;
    tex_decode u_dec (.pixfmt(d_pixfmt),.scan(d_scan),.memtel(d_memtel),
                      .offset_lo(d_off_lo),.pal_argb(d_pal_argb),.argb(dec_argb));

    always @(posedge clk) begin
        if (reset) begin
            tc_h<=0; tc_t<=0; tc_cnt<=0;
            vp_h<=0; vp_t<=0; vp_cnt<=0;
            vq_h<=0; vq_t<=0; vq_cnt<=0;
            d_v<=0; out_valid<=0;
        end else begin
            out_valid <= 1'b0;

            // fifo_tc
            if (tc_push) begin tcf[tc_t] <= in_side; tc_t <= tc_t + 1'b1; end
            if (tc_pop)  tc_h <= tc_h + 1'b1;
            tc_cnt <= tc_cnt + (tc_push?1:0) - (tc_pop?1:0);

            // vq-pending FIFO (buffers derived vq address until the vq cache is ready)
            if (vp_push) begin vpf_addr[vp_t] <= vq_addr; vpf_side[vp_t] <= tc_side; vp_t <= vp_t + 1'b1; end
            if (vp_pop)  vp_h <= vp_h + 1'b1;
            vp_cnt <= vp_cnt + (vp_push?1:0) - (vp_pop?1:0);

            // fifo_vq: side data captured when the vq req is actually ISSUED
            if (vq_push) begin vqf[vq_t] <= vpf_side[vp_h]; vq_t <= vq_t + 1'b1; end
            if (vq_pop)  vq_h <= vq_h + 1'b1;
            vq_cnt <= vq_cnt + (vq_push?1:0) - (vq_pop?1:0);

            // feed decode (vq ack wins the workload-impossible collision)
            d_v <= 1'b0;
            if (vq_to_decode) begin
                d_memtel <= vq_resp.rdata;
                d_side   <= vqf[vq_h];
                d_v      <= 1'b1;
            end else if (nonvq_to_decode) begin
                d_memtel <= tc_memtel;
                d_side   <= tc_side;
                d_v      <= 1'b1;
            end

            // decode output (non-textured -> argb 0)
            if (d_v) begin
                argb      <= d_txd ? dec_argb : 32'h00000000;
                out_valid <= 1'b1;
            end
        end
    end
endmodule
