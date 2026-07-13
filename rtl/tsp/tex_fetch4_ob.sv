//
// tex_fetch4_ob - 4-corner raw texel fetch (output-buffered). 4 byte-offsets in -> 4 raw
// 64-bit memory words out. Owns the two shared 4-read-port caches (data + VQ) as its ONLY
// submodules; the whole fetch pipeline is inline.
//
// Addressing. tex_addr / vq_addr are 64-BIT-WORD base addresses (21b, DC VRAM). The 4
// corner offsets are in BYTES (22b). So per corner:
//   data word addr = tex_addr + offset[21:3]      (offset byte, its 64-bit-word part)
//   byte lane      = offset[2:0]                   (byte-within-word; also the VQ lane)
//
// Behaviour per corner:
//   !TEX          : issue NO request; output undefined (this fetcher is unused).
//   TEX & !VQ     : one data-cache read -> output = that raw 64-bit word. VQ idle.
//   TEX &  VQ     : data-cache read -> memtel ; index = memtel[8*offset[2:0] +: 8] ;
//                   VQ-cache read at (vq_addr + index) -> output = that 64-bit word.
//
// =============================================================================
// MULTIPLE-IN-FLIGHT, BACK-PRESSURED HANDSHAKE. -----------------------------
// tex_cache_4p_1c is a multi-stage M10K pipeline (ACCEPT -> MATCH -> DATA) whose DATA stage
// HOLDS ack+rdata until the consumer pulses resp_take (and freezes .ready while un-taken).
// That lets the fetch keep MANY requests in flight for 1-pixel/clock throughput WITHOUT the
// "dropped ack pulse" hazard: the fetch drains each cache strictly at its ack rate (take ==
// ack), and the cache stalls issue if the fetch can't keep up.
//
// THREE ordered queues, all draining in cache-issue order:
//   df (DATA in-flight): pushed on ACCEPT (a TC read issued, or a !tex bypass). The cache
//        holds TC results in issue order; the df LAND pointer captures tc_resp.rdata into the
//        df head on every tc_ack (tc_take pulsed) and marks it landed. !tex heads land with
//        no cache trip. Fully decouples ISSUE from LAND.
//   vf (VQ in-flight):   pushed when a landed VQ pixel issues its codebook read. Drained on
//        vq_ack (vq_take pulsed). Carries only the ride-through payload; the word comes from
//        vq_resp at the rf head.
//   rf (RESULT, accept order): the single ORDERED output. Every processed df head pushes one
//        rf entry. A non-VQ / !tex entry carries its resolved word and drains immediately; a
//        VQ entry drains when its codebook word acks. HEAD-OF-LINE: a VQ entry awaiting its
//        ack stalls the entries behind it - the strict in-order discipline the downstream
//        (no-stall) decode requires (exactly one out_valid pulse per pixel).
// Because both caches ack in issue order and VQ reads issue in that same order, the rf VQ
// head's ack is exactly the current vq_ack (an earlier VQ pixel's ack always precedes and
// drains it first) - no reorder buffer needed.
//
// PAYLOAD (in_pl -> out_pl) rides the queues so it can NEVER desync from the texels.
//
// Exposes TWO DDR read ports (tc, vq) to the parent arbiter.
//
module tex_fetch4_ob import tsp_pkg::*; #(
    parameter integer PLW = 1              // decode payload bus width (rides with the pixel)
) (
    input             clk,
    input             reset,

    input             in_valid,
    input             tex,               // TEX: textured pixel
    input             vq,               // VQ:  VQ-compressed texture
    input      [20:0] tex_addr,          // data base (64-bit-word units)
    input      [20:0] vq_addr,           // VQ codebook base (64-bit-word units)
    input      [21:0] tex_offset [0:3],  // per-corner byte offsets (22b: up to 16bpp+mip)
    input      [PLW-1:0] in_pl,           // decode payload latched WITH the accepted pixel
    output            in_ready,          // 0 = stall (cache filling / queues full); hold

    output            out_valid,
    output     [63:0] texel [0:3],       // raw 64-bit memory words (undefined if !tex)
    output     [PLW-1:0] out_pl,          // in_pl carried to align with out_valid/texel

    // two DDR read ports to the parent arbiter ([0]=tc data, [1]=vq codebook)
    output ddr_rd_req_t  ddr_req  [0:1],
    input  ddr_rd_resp_t ddr_resp [0:1]
);
    localparam integer QW = 3;             // queue addr width -> depth 8
    localparam integer QD = 1 << QW;

    // shared 4-read-port caches (data + VQ). resp_take back-pressures each cache's held
    // DATA-stage result until the fetch drains it.
    cache_req_t   tc_req [0:3], vq_req [0:3];
    cache_resp_t  tc_resp[0:3], vq_resp[0:3];
    wire          tc_take, vq_take;
    tex_cache_4p_1c u_tc4 (.clk(clk),.reset(reset),
        .creq(tc_req),.cresp(tc_resp),.resp_take(tc_take),
        .dreq(ddr_req[0]),.dresp(ddr_resp[0]));
    tex_cache_4p_1c u_vq4 (.clk(clk),.reset(reset),
        .creq(vq_req),.cresp(vq_resp),.resp_take(vq_take),
        .dreq(ddr_req[1]),.dresp(ddr_resp[1]));

    wire tc_ready = tc_resp[0].ready;      // all 4 ports gate together
    wire vq_ready = vq_resp[0].ready;
    wire tc_ack   = tc_resp[0].ack;        // HELD until tc_take
    wire vq_ack   = vq_resp[0].ack;        // HELD until vq_take

    // per-corner data-cache word address + VQ byte lane (combinational off inputs)
    wire [28:0] tc_waddr [0:3];
    wire [2:0]  vqlane   [0:3];
    genvar gi;
    generate for (gi=0; gi<4; gi=gi+1) begin : addr
        assign tc_waddr[gi] = {8'd0, tex_addr} + {10'd0, tex_offset[gi][21:3]};
        assign vqlane[gi]   = tex_offset[gi][2:0];
    end endgenerate

    integer i;

    // ============================================================================
    // DATA in-flight FIFO (df). Two pointers:
    //   df_wr : push on ACCEPT (issue a TC read, or a !tex bypass entry).
    //   df_ld : LAND pointer - the oldest not-yet-landed TC entry. Advances (captures
    //           tc_resp.rdata) on each tc_ack. Decouples the cache-drain rate from the head
    //           processor's rate.
    //   df_rd : HEAD pointer - the oldest landed entry, processed into rf/vf.
    // Invariant: df_rd <= df_ld <= df_wr.
    // ============================================================================
    reg              df_vq   [0:QD-1];
    reg              df_tex  [0:QD-1];
    reg [20:0]       df_vqbase[0:QD-1];
    reg [2:0]        df_lane [0:QD-1][0:3];
    reg [PLW-1:0]    df_pl   [0:QD-1];
    reg [63:0]       df_word [0:QD-1][0:3];  // captured data word (valid once landed)
    reg [QW:0]       df_wr, df_ld, df_rd;    // extra MSB for full/empty
    wire             df_wr_full = (df_wr[QW-1:0]==df_rd[QW-1:0]) && (df_wr[QW]!=df_rd[QW]);
    wire [QW-1:0]    df_wa = df_wr[QW-1:0];
    wire [QW-1:0]    df_la = df_ld[QW-1:0];
    wire [QW-1:0]    df_ra = df_rd[QW-1:0];

    // ============================================================================
    // Result FIFO (rf): the single ORDERED output stage. Declared before the accept gate so
    // in_ready can require room. VQ entries get their word from vq_resp at the head on vq_ack.
    // ============================================================================
    reg              rf_vq  [0:QD-1];
    reg [63:0]       rf_word[0:QD-1][0:3];
    reg [PLW-1:0]    rf_pl  [0:QD-1];
    reg [QW:0]       rf_wr, rf_rd;
    wire             rf_full  = (rf_wr[QW-1:0]==rf_rd[QW-1:0]) && (rf_wr[QW]!=rf_rd[QW]);
    wire             rf_empty = (rf_wr==rf_rd);
    wire [QW-1:0]    rf_wa = rf_wr[QW-1:0];
    wire [QW-1:0]    rf_ra = rf_rd[QW-1:0];

    // ============================================================================
    // ACCEPT: a new pixel enters when the data cache is ready (for textured pixels) and both
    // the df and rf queues have room. A !tex bypass pixel needs no cache but still rides.
    // ============================================================================
    assign in_ready = !df_wr_full && !rf_full && (!tex || tc_ready);
    wire   accept   = in_valid && in_ready;

    // issue data-cache read (textured pixels only)
    generate for (gi=0; gi<4; gi=gi+1) begin : tcreq
        assign tc_req[gi].req   = accept && tex;
        assign tc_req[gi].waddr = tc_waddr[gi];
    end endgenerate

    // ============================================================================
    // LAND: capture TC results in issue order. The df LAND slot (df_la) is the oldest entry
    // whose word hasn't been captured. A textured land slot captures on tc_ack; a !tex slot
    // needs no cache trip. df_ld advances as slots land. tc_take pulses on each capture so
    // the cache releases its held result.
    // ============================================================================
    wire df_land_pending = (df_ld != df_wr);          // an issued entry awaiting land
    wire df_la_tex       = df_tex[df_la];
    wire df_do_land      = df_land_pending && (df_la_tex ? tc_ack : 1'b1);
    assign tc_take       = df_land_pending && df_la_tex && tc_ack;   // take exactly on capture

    // ============================================================================
    // HEAD PROCESS: the oldest LANDED df entry (df_rd, valid when df_rd != df_ld) is turned
    // into an rf entry. Non-VQ/!tex -> resolved word, ready now. VQ -> issue codebook read
    // (needs vq_ready) and push a VQ rf entry (word resolved later at the rf head).
    // ============================================================================
    wire df_head_valid = (df_rd != df_ld);            // a landed entry at the head
    wire df_head_tex   = df_tex[df_ra];
    wire df_head_vq    = df_vq [df_ra];

    // VQ codebook address per corner (from the CAPTURED head data word + head lanes/base)
    wire [7:0]  vqidx  [0:3];
    wire [28:0] vqaddr [0:3];
    generate for (gi=0; gi<4; gi=gi+1) begin : vqa
        assign vqidx[gi]  = df_word[df_ra][gi][ {df_lane[df_ra][gi], 3'd0} +: 8 ];
        assign vqaddr[gi] = {8'd0, df_vqbase[df_ra]} + {21'd0, vqidx[gi]};
    end endgenerate

    // A VQ head issues its codebook read when vq is ready and rf has room. A non-VQ/!tex head
    // just needs rf room. Head advances (df_rd++) and pushes rf when processed.
    wire head_is_vq   = df_head_valid && df_head_tex && df_head_vq;
    wire vq_issue     = head_is_vq && vq_ready && !rf_full;
    wire head_nonvq   = df_head_valid && !(df_head_tex && df_head_vq);  // non-VQ tex or !tex
    wire head_proc    = (head_nonvq && !rf_full) || vq_issue;

    generate for (gi=0; gi<4; gi=gi+1) begin : vqreq
        assign vq_req[gi].req   = vq_issue;
        assign vq_req[gi].waddr = vqaddr[gi];
    end endgenerate

    // ============================================================================
    // rf OUTPUT: head drains in order. Non-VQ ready now; VQ ready on its vq_ack (in order).
    // ============================================================================
    wire rf_head_vq   = rf_vq[rf_ra];
    wire rf_out_ready = !rf_empty && (!rf_head_vq || vq_ack);
    assign vq_take    = !rf_empty && rf_head_vq && vq_ack;   // consume the codebook word
    wire   rf_pop     = rf_out_ready;

    wire [63:0] rf_out_word [0:3];
    generate for (gi=0; gi<4; gi=gi+1) begin : ow
        assign rf_out_word[gi] = rf_head_vq ? vq_resp[gi].rdata : rf_word[rf_ra][gi];
    end endgenerate

    generate for (gi=0; gi<4; gi=gi+1) begin : out
        assign texel[gi] = rf_out_word[gi];
    end endgenerate
    assign out_pl    = rf_pl[rf_ra];
    assign out_valid = rf_pop;

    // ============================================================================
    // SEQUENTIAL
    // ============================================================================
    always @(posedge clk) begin
        if (reset) begin
            df_wr <= 0; df_ld <= 0; df_rd <= 0;
            rf_wr <= 0; rf_rd <= 0;
        end else begin
            // ---- df PUSH (accept) ----
            if (accept) begin
                df_vq   [df_wa]  <= vq;
                df_tex  [df_wa]  <= tex;
                df_vqbase[df_wa] <= vq_addr;
                for (i=0;i<4;i=i+1) df_lane[df_wa][i] <= vqlane[i];
                df_pl   [df_wa]  <= in_pl;
                df_wr <= df_wr + 1'b1;
            end
            // ---- df LAND (capture TC word / bypass) ----
            if (df_do_land) begin
                if (df_la_tex)
                    for (i=0;i<4;i=i+1) df_word[df_la][i] <= tc_resp[i].rdata;
                // !tex: word unused; leave as-is.
                df_ld <= df_ld + 1'b1;
            end
            // ---- HEAD PROCESS -> rf PUSH ----
            if (head_proc) begin
                rf_vq[rf_wa] <= head_is_vq;
                if (!head_is_vq)
                    for (i=0;i<4;i=i+1) rf_word[rf_wa][i] <= df_word[df_ra][i];
                rf_pl[rf_wa] <= df_pl[df_ra];
                rf_wr <= rf_wr + 1'b1;
                df_rd <= df_rd + 1'b1;
            end
            // ---- rf POP (emit) ----
            if (rf_pop) rf_rd <= rf_rd + 1'b1;
        end
    end

`ifndef SYNTHESIS
    // queues must never overflow: assert on push-into-full.
    always @(posedge clk) if (!reset) begin
        if (accept    && df_wr_full) $display("$$$ tex_fetch4_ob df OVERFLOW %m @%0t", $time);
        if (head_proc && rf_full)    $display("$$$ tex_fetch4_ob rf OVERFLOW %m @%0t", $time);
    end
`endif
endmodule
