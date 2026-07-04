//
// spanner_v2 - the DECOUPLED span-generate + TSP-setup stage of the v2 TSP pipeline.
//
// Replaces the monolithic peel_core `spn` FSM's resolve+setup+write with two engines
// that run CONCURRENTLY, joined by a small setup FIFO, so the per-triangle setup cost
// (~54cyc/tri) is hidden behind the shade stage (1px/clk, downstream, not in here):
//
//   FSM 1  SPANGEN  - walks the tile's tag buffer 4 ALIGNED pixels/clk, coalesces the
//                     leading same-tag run within each aligned group into a SPAN, and
//                     writes {id, repeat, invW[0:3]} to the OUT span buffer at the
//                     run-start pixel index. A NEW (not-yet-seen) tag BUMP-ALLOCATES a
//                     ring setup id (id = top_tag++) and pushes {id,tag} to the setup FIFO.
//                     NEVER stalls on setup (that is downstream).
//   FSM 2  SETUP    - drains the setup FIFO: tag -> record_fetcher (GetFpuEntry) ->
//                     tsp_setup (10 planes) -> WRITE triangle_setups[id]. Runs in
//                     parallel with SPANGEN and the (external) shade stage.
//
// DEDUP / ID: a direct-mapped M10K dedup map (indexed by pc_slot(tag)) holds {gen,tag,id};
// the setup id is BUMP-ALLOCATED (top_tag), so triangle_setups ids are DENSE (0..distinct-1
// per tile) not hash-scattered. triangle_setups is a RING shared with TSP (no ping-pong):
// tsp_go signals a tile's setups done, tsp_rd_done frees that tile's ring range (tail).
// A single 32x32 tile has <=1024 distinct tags, so ring_size=NSLOT never intra-tile
// overflows; cross-tile overlap can overflow -> SPANGEN stalls for tsp_rd_done. The dedup
// map's `gen` (bumped per start) invalidates prior-tile buckets for free ("don't reuse when
// x/y change"); on gen wrap a clear-walk writes gen=0.
//
// The triangle_setups RING + span buffer are EXTERNAL (module write ports); the ring
// tail/reclaim handshake (tsp_go/tsp_rd_done) lets peel_core share ONE buffer with TSP.
//
// IN (tag buffer): a 4-wide ALIGNED read port. The module presents a group base
// (rd_group = x & ~3) and receives the 4 lanes' {valid,tag,invW,pt} the NEXT cycle
// (1-cyc registered read, same timing as taginvw_tile_buffer's single-pixel port; glue
// widens that buffer to serve 4 aligned lanes). Lane l is pixel (group|l).
//
module spanner_v2 import tsp_pkg::*; #(
    parameter integer NSLOT = 1024,          // triangle_setups depth (== tile pixels)
    parameter integer SLOTW = 10             // clog2(NSLOT)
) (
    input                       clk,
    input                       reset,

    // ---- control ----
    input                       start,       // 1-cyc: begin resolving one tile pass
    output reg                  busy,         // start .. SPANGEN done && setup drained
    input                       shade_mode,  // 1=OP (shade all px); 0=PEEL (gate on valid)
    input      [31:0]           xbase, ybase,// this tile's origin (for tsp_setup_min)
    input      [26:0]           param_base,
    input                       intensity_shadow, // regs.fpu_shad_scale.intensity_shadow

    // ---- shared-ring handshake with the TSP consumer ----
    // triangle_setups is a RING shared with TSP (no ping-pong): ids are bump-allocated
    // (top_tag), TSP reads them, and a whole tile's ring range frees when TSP finishes it.
    output reg                  tsp_go,      // 1-cyc: this tile's setups are all done -> TSP may read
    input                       tsp_rd_done, // 1-cyc: TSP finished the oldest handed tile -> free its range

    // ---- IN: 4-wide ALIGNED tag-buffer read (present addr -> data NEXT cycle) ----
    output reg                  rd_valid,    // present a group read this cycle
    output reg [SLOTW-1:0]      rd_group,    // group base pixel index (x & ~3)
    input      [3:0]            ti_valid,    // per-lane staged-this-pass bit
    input      [31:0]           ti_tag  [0:3],
    input      [31:0]           ti_invw [0:3],
    input      [3:0]            ti_pt,       // per-lane PT alpha-test bit

    // ---- OUT: triangle_setups WRITE (SETUP engine) ----
    output reg                  ts_we,
    output reg [SLOTW-1:0]      ts_id,
    output reg [31:0]           ts_isp, ts_tsp, ts_tcw,
    output reg [319:0]          ts_ddx, ts_ddy, ts_c,   // 10 x 32, lane j at [32*j+:32]

    // ---- OUT: span buffer WRITE (SPANGEN engine) ----
    output reg                  sp_we,
    output reg [SLOTW-1:0]      sp_idx,      // run-start pixel index (0..1023)
    output reg [SLOTW-1:0]      sp_id,       // setup id (== triangle_setups slot)
    output reg [2:0]            sp_rep,      // run length 1..4
    output reg [31:0]           sp_invw [0:3], // per-covered-pixel invW (lanes 0..rep-1)
    output reg [3:0]            sp_shmask,   // per-covered-lane shade-valid
    output reg                  sp_at,       // PT alpha-test enable (run-start lane)
    input                       sp_ready,    // span consumer (expander) can accept this cycle

    // ---- DDR client for the internal record_fetcher ----
    output ddr_rd_req_t         dreq,
    input  ddr_rd_resp_t        dresp
);
    // ============================ dedup map + setup-id ring ============================
    // id = BUMP-ALLOCATED (top_tag), not the hash. The dedup MAP is a direct-mapped M10K
    // (indexed by pc_slot(tag)) that remembers, per hash bucket, {gen, tag, id} = which
    // ring id a tag was assigned. triangle_setups[id] is a RING (top_tag..tail) shared
    // with TSP; ids are dense (0..distinct-1 per tile) instead of scattered by the hash,
    // so the ring can be far smaller than 1024 and shared with TSP without ping-pong.
    //   lookup:  h = pc_slot(tag); if map[h].gen==cur_gen && map[h].tag==tag -> HIT, id=map[h].id
    //            else MISS -> id = top_tag++, map[h] = {cur_gen, tag, id}, push a setup.
    // pc_slot spreads tags across the 1024 map buckets (10-bit hash). tag =
    // {skip[26:24], param_offs[23:3], tag_offset[2:0]}; strip triangles share param_offs.
    function automatic [SLOTW-1:0] pc_slot(input [31:0] tag);
        pc_slot = tag[12:3] ^ tag[22:13] ^ { {(SLOTW-3){1'b0}}, tag[2:0] };
    endfunction

    // dedup MAP: ONE M10K holding {gen, tag, id} per bucket, so tag+validity+id all live
    // in block RAM (no flop valid-vector, no bulk clear). A bucket is VALID this pass iff
    // gen==cur_gen; `start` bumps cur_gen so prior-pass buckets go stale for free ("don't
    // reuse when x/y change"). cur_gen never 0; on wrap a clear-walk writes gen=0 to all.
    // REGISTERED read (1-cyc); the dedup test is PIPELINED COAL(read)->EMIT(compare+write).
    // COHERENCY: EMIT can WRITE a bucket the same cycle COAL presents the next read (M10K
    // returns stale on same-addr r+w) -> the two most-recent allocations are FORWARDED.
    localparam integer GEN_W = 8;
    localparam integer DD_W  = GEN_W + 32 + SLOTW;   // {gen, tag, id}
    localparam [GEN_W-1:0] GEN_MAX = {GEN_W{1'b1}};
    (* ramstyle = "M10K, no_rw_check" *) reg [DD_W-1:0] dedup_ram [0:NSLOT-1];
    reg [DD_W-1:0]     dd_rd_q;                   // registered read {gen,tag,id} of coal bucket
    reg [GEN_W-1:0]    cur_gen;                   // current pass generation (never 0)
    reg                dd_clearing;               // clear-walk in progress (gen wrap)
    reg [SLOTW-1:0]    dd_clr_addr;
    wire [GEN_W-1:0]   dd_rd_gen  = dd_rd_q[DD_W-1 -: GEN_W];
    wire [31:0]        slot_tag_q = dd_rd_q[SLOTW +: 32];
    wire [SLOTW-1:0]   slot_id_q  = dd_rd_q[0 +: SLOTW];
    wire               slot_valid_q = (dd_rd_gen == cur_gen);

    // ---- setup-id RING (triangle_setups slots), shared with TSP ----
    // top_tag = next id to allocate; tail = oldest id still owned by TSP. Extra MSB (like a
    // FIFO) disambiguates full vs empty so all NSLOT ids are usable. On tsp_done the oldest
    // handed tile's range frees (tail <- that tile's end). If the ring would overflow (a
    // single tile with >NSLOT distinct, or two dense tiles overlapping), SPANGEN stalls
    // until TSP frees a tile. A go-FIFO records each handed tile's end pointer.
    reg [SLOTW:0]      top_tag, tail;            // ring head / tail (SLOTW+1 bits)
    wire ring_empty = (top_tag == tail);
    wire ring_full  = (top_tag[SLOTW] != tail[SLOTW]) && (top_tag[SLOTW-1:0] == tail[SLOTW-1:0]);
    localparam integer GF_AW = 2;                 // up to 4 tiles handed-but-not-done
    reg [SLOTW:0]      gf_mem [0:(1<<GF_AW)-1];   // tile END pointers handed to TSP
    reg [GF_AW:0]      gf_wp, gf_rp;
    wire gf_empty = (gf_wp == gf_rp);

    // ============================ setup FIFO (SPANGEN -> SETUP) ============================
    // {id, tag} pushed when a span allocates a NEW slot; drained by the SETUP engine.
    localparam integer SF_AW = 3;                 // 8-deep
    localparam integer SF_N  = (1 << SF_AW);
    localparam integer SF_W  = SLOTW + 32;        // {id, tag}
    reg  [SF_W-1:0] sf_mem [0:SF_N-1];
    reg  [SF_AW:0]  sf_wp, sf_rp;                 // extra MSB for full/empty disambig
    wire            sf_empty = (sf_wp == sf_rp);
    wire            sf_full  = (sf_wp[SF_AW] != sf_rp[SF_AW]) &&
                               (sf_wp[SF_AW-1:0] == sf_rp[SF_AW-1:0]);
    reg             sf_push;  reg [SF_W-1:0] sf_pdata;
    wire [SF_W-1:0] sf_head  = sf_mem[sf_rp[SF_AW-1:0]];

    // ============================ FSM 1: SPANGEN (pipelined, 1 span/cycle) ============================
    // A continuously-advancing 2-stage pipeline (M10K-friendly: slot read is registered):
    //   COAL (this cycle): a group's 4 lanes are held in g_*; coalesce the leading same-tag
    //        run at the current intra-group position sg_x -> a span descriptor run_*.
    //        PRESENT the registered slot_tag read of run_id. Advance sg_x by run_rep. If the
    //        group is exhausted, the group PREFETCH (ahead) supplies the next group's lanes
    //        so COAL keeps producing one descriptor EVERY cycle with no reissue bubble.
    //   EMIT (next cycle): the descriptor produced by COAL last cycle is in t_*; its slot
    //        read has resolved (slot_*_q, forwarded) -> dedup compare, WRITE the span,
    //        allocate on a miss. Retires one span/cycle.
    // Uniform tile (one tag, 4px runs) -> 256 groups x 1 span x 1 cyc + fill/drain ~= 258.
    // Multi-span group (A B C D) -> COAL produces 4 descriptors on 4 consecutive cycles
    // (sg_x steps within the held group, no reissue) -> 4 cyc, still 1 span/cyc.
    //
    // GROUP PREFETCH: the tag buffer read has 1-cyc latency, so we must present the NEXT
    // group's read the cycle BEFORE COAL needs it. gp_* holds the prefetched group; when
    // COAL exhausts the current group it swaps gp_* -> g_* and the prefetch reads group+1.
    reg              sg_active;     // SPANGEN still walking (COAL may produce)
    reg [SLOTW-1:0]  sg_x;          // next pixel to coalesce a span at (0..NSLOT-1)
    reg              g_ready;       // ti_* holds sg_x's group this cycle (read landed)
    // The tag buffer is a registered-read RAM: its output ti_* reflects the address
    // presented LAST cycle, and updates EVERY cycle. So we present rd_group = the group of
    // the pixel COAL coalesces NEXT cycle, continuously; then ti_* is ALWAYS the correct
    // group source for the current sg_x -> COAL coalesces directly off ti_* (no held-group
    // register, no source mux). g_ready gates the 1-cycle fill latency after start/stall.
    reg [SLOTW-1:0]  rd_next_x;     // combinational: pixel whose group we address

    // ---- span descriptor latched at COAL, consumed (emitted) at EMIT ----
    reg              t_valid;       // EMIT stage occupied
    reg [SLOTW-1:0]  t_x;           // run-start pixel index
    reg [SLOTW-1:0]  t_h;           // pc_slot(run_tag) = dedup map bucket
    reg [31:0]       t_tag;
    reg [2:0]        t_rep;
    reg [3:0]        t_shmask;
    reg [31:0]       t_invw [0:3];
    reg              t_at;

    wire [1:0] sg_lane = sg_x[1:0];              // intra-group position of sg_x

    // ---- leading-run coalesce (combinational off ti_*, the current group source) ----
    // Start lane = sg_lane. Extend while lane in-group, tag equal. shade_ok(l) =
    // shade_mode | ti_valid[l]. A run of not-shade-eligible tags is still a span (advances
    // the walk) but with shmask=0; we coalesce equal tags to match the aligned model.
    reg  [2:0] run_rep;                          // 1..4
    reg  [3:0] run_shmask;                       // per-lane shade-valid over the run
    reg  [31:0] run_tag;
    integer rl;
    always @(*) begin
        run_tag    = ti_tag[sg_lane];
        run_rep    = 3'd1;
        run_shmask = 4'd0;
        run_shmask[sg_lane] = shade_mode | ti_valid[sg_lane];
        for (rl = 0; rl < 4; rl = rl + 1) begin
            if (rl > sg_lane && rl == sg_lane + run_rep && ti_tag[rl] == run_tag) begin
                run_rep = run_rep + 3'd1;
                run_shmask[rl] = shade_mode | ti_valid[rl];
            end
        end
    end

    wire [SLOTW-1:0] run_id = pc_slot(run_tag);

    // dedup test in EMIT, using the REGISTERED slot read (slot_tag_q/slot_valid_q) of
    // t_id, WITH forwarding. In the 1-span/cycle pipeline, EMIT retires a span every cycle
    // and may write the slot store the SAME cycle COAL presents the next span's read (M10K
    // read-during-write returns stale data). So we forward the LAST TWO emitted allocations:
    //   fwd0 = the allocation done LAST cycle (its write is in flight, read returns stale)
    //   fwd1 = the allocation done TWO cycles ago (covers the M10K's write-settle window)
    // Both are checked against t_id; the most recent wins. Uniform tile (same tag every
    // group -> same id) is covered: after the first alloc, every later EMIT sees fwd and
    // dedups (no re-push, no re-setup).
    // forwarding is keyed by the map BUCKET (t_h = pc_slot) and carries {tag, id} of the
    // two most-recent allocations, so a back-to-back reuse of a just-allocated bucket sees
    // the fresh {tag,id} despite the M10K read-during-write staleness.
    reg             fwd0_valid, fwd1_valid;
    reg [SLOTW-1:0] fwd0_h,     fwd1_h;
    reg [31:0]      fwd0_tag,   fwd1_tag;
    reg [SLOTW-1:0] fwd0_id,    fwd1_id;
    wire            fwd0_hit = fwd0_valid && (fwd0_h == t_h);
    wire            fwd1_hit = fwd1_valid && (fwd1_h == t_h);
    wire            eff_valid = fwd0_hit | fwd1_hit | slot_valid_q;
    wire [31:0]     eff_tag   = fwd0_hit ? fwd0_tag : (fwd1_hit ? fwd1_tag : slot_tag_q);
    wire [SLOTW-1:0] eff_id   = fwd0_hit ? fwd0_id  : (fwd1_hit ? fwd1_id  : slot_id_q);
    wire is_dedup_hit = t_valid && eff_valid && (eff_tag == t_tag);
    wire needs_alloc  = t_valid && !is_dedup_hit;
    // the ring id this emit uses: reuse the cached/forwarded id on a hit, else the next
    // bump-allocated id (top_tag).
    wire [SLOTW-1:0] emit_id = is_dedup_hit ? eff_id : top_tag[SLOTW-1:0];

    // emit can commit when: an ALLOCATING emit needs setup-FIFO room AND a free ring slot.
    // A same-tag reuse needs neither. When blocked, the WHOLE pipeline freezes (COAL+EMIT).
    wire emit_stall_fifo = needs_alloc && sf_full;
    wire emit_stall_ring = needs_alloc && ring_full;   // no free ring id -> wait for TSP
    // also freeze if the span consumer (expander) can't accept the span this emit produces
    wire emit_stall_span = t_valid && !sp_ready;
    wire pipe_stall      = emit_stall_fifo | emit_stall_ring | emit_stall_span;

    // ---- COAL advance: sg_x steps by run_rep; group exhausted when it crosses the group ----
    wire [SLOTW-1:0] sg_x_next   = sg_x + { {(SLOTW-3){1'b0}}, run_rep };
    wire walk_last              = (sg_x_next == '0);          // wrapped past pixel 1023
    // COAL produces a descriptor this cycle iff walking, the group read has landed
    // (g_ready), and the pipeline isn't frozen.
    wire coal_fires = sg_active && g_ready && !pipe_stall;

    // ============================ SETUP path (streaming, no serial FSM) ============================
    // Three independent stages, each fed continuously, so the record fetch of triangle
    // N+1 OVERLAPS the plane setup of triangle N (they were serial before: ~144 fetch +
    // ~257 setup = ~400 cyc/triangle back-to-back):
    //   FETCH  : pop the setup FIFO into record_fetcher whenever it is free.
    //   pend_* : 1-deep skid holding the decoded record until tsp_setup_min is free
    //            (frees the fetcher to start the next fetch).
    //   SETUP  : latch pend_* into cur_*/fv_* (held stable for the whole run) + start
    //            tsp_setup_min; on tsp_done pulse ts_pend -> ts_we write.
    reg              fetch_busy;    // a fetch is in flight (fx_start..fx_done)
    reg [SLOTW-1:0]  fx_id;         // the in-flight fetch's setup id
    reg              pend_v;        // pend_* holds a decoded record awaiting setup
    reg [SLOTW-1:0]  pend_id;
    reg [31:0]       pend_isp, pend_tsp, pend_tcw;
    reg [31:0]       pend_x[0:2], pend_y[0:2], pend_z[0:2];
    reg [31:0]       pend_u[0:2], pend_v3[0:2], pend_col[0:2], pend_ofs[0:2];
    reg              su_run;        // tsp_setup_min busy (tsp_start..tsp_done)
    reg              ts_pend;       // write triangle_setups this cycle (cycle after tsp_done)
    reg [SLOTW-1:0]  su_id;         // the active setup's id (write target)
`ifndef SYNTHESIS
    integer          su_dbg_cyc;   // +sutrace cycle counter
`endif

    // record_fetcher (demand only; FIFOs front & back hide its latency, no prefetch)
    reg              fx_start;
    reg  [31:0]      fx_tag;
    wire             fx_busy, fx_done;
    wire [31:0]      fx_isp, fx_tsp, fx_tcw;
    wire [31:0]      fx_x[0:2], fx_y[0:2], fx_z[0:2];
    wire [31:0]      fx_u[0:2], fx_v[0:2], fx_col[0:2], fx_ofs[0:2];
    record_fetcher u_fetch (
        .clk(clk), .reset(reset),
        .start(fx_start), .tag(fx_tag), .param_base(param_base),
        .intensity_shadow(intensity_shadow),
        .busy(fx_busy), .done(fx_done),
        .o_isp(fx_isp), .o_tsp(fx_tsp), .o_tcw(fx_tcw),
        .o_x(fx_x), .o_y(fx_y), .o_z(fx_z),
        .o_u(fx_u), .o_v(fx_v), .o_col(fx_col), .o_ofs(fx_ofs),
        .dreq(dreq), .dresp(dresp)
    );

    // latched decoded record of the tri being set up (feed tsp_setup_min)
    reg [31:0] cur_isp, cur_tsp, cur_tcw;
    reg [31:0] fv_x[0:2], fv_y[0:2], fv_z[0:2];
    reg [31:0] fv_u[0:2], fv_v[0:2], fv_col[0:2], fv_ofs[0:2];
    wire f_texture = cur_isp[ISP_TEXTURE_BIT];
    wire f_offset  = cur_isp[ISP_OFFSET_BIT];
    wire f_gouraud = cur_isp[ISP_GOURAUD_BIT];

    // tsp_setup_min: 10-plane producer
    reg              tsp_start;
    wire             tsp_done, tsp_pvalid;
    wire [3:0]       tsp_pidx;
    wire [31:0]      tsp_pddx, tsp_pddy, tsp_pc;
    tsp_setup_stream u_tsp (
        .clk(clk), .reset(reset), .start(tsp_start), .done(tsp_done),
        .gouraud(f_gouraud), .texture(f_texture), .offset(f_offset),
        .x1(fv_x[0]),.y1(fv_y[0]),.z1(fv_z[0]),
        .x2(fv_x[1]),.y2(fv_y[1]),.z2(fv_z[1]),
        .x3(fv_x[2]),.y3(fv_y[2]),.z3(fv_z[2]),
        .xbase(xbase), .ybase(ybase),
        .u1(fv_u[0]),.v1(fv_v[0]),.u2(fv_u[1]),.v2(fv_v[1]),.u3(fv_u[2]),.v3(fv_v[2]),
        .col1(fv_col[0]),.col2(fv_col[1]),.col3(fv_col[2]),
        .ofs1(fv_ofs[0]),.ofs2(fv_ofs[1]),.ofs3(fv_ofs[2]),
        .plane_valid(tsp_pvalid), .plane_idx(tsp_pidx),
        .o_ddx(tsp_pddx), .o_ddy(tsp_pddy), .o_c(tsp_pc)
    );

    // plane accumulators (written by tsp_setup_min's streamed plane_valid/plane_idx)
    reg [319:0] acc_ddx, acc_ddy, acc_c;

    // ============================ combinational OUT + FIFO drive ============================
    integer k;
    always @(*) begin
        // ----- EMIT: span write + FIFO push. The descriptor produced by COAL last cycle
        // is in t_* (t_valid); its slot read has resolved -> dedup here and write. Held
        // (no write, no advance) when the pipeline is frozen. -----
        sp_we      = t_valid && !pipe_stall;
        sp_idx     = t_x;
        sp_id      = emit_id;              // bump-allocated (or reused) ring id
        sp_rep     = t_rep;
        sp_shmask  = t_shmask;
        sp_at      = t_at;
        for (k = 0; k < 4; k = k + 1) sp_invw[k] = t_invw[k];

        sf_push  = sp_we && needs_alloc;   // allocate -> push a setup
        sf_pdata = { emit_id, t_tag };

        // COAL group read address. A registered-read tag buffer updates its output EVERY
        // cycle from the presented address, so we drive rd_group to the group of the pixel
        // COAL will coalesce NEXT cycle, CONTINUOUSLY (don't rely on the read output
        // persisting across cycles). Next-coalesced pixel = sg_x_next if COAL fires this
        // cycle (advancing), else sg_x (idle/fill/held). rd_valid tracks it while active.
        rd_next_x = coal_fires ? sg_x_next : sg_x;
        rd_valid  = sg_active && !pipe_stall;
        rd_group  = { rd_next_x[SLOTW-1:2], 2'b00 };

        // ----- SETUP -> triangle_setups write -----
        ts_we  = ts_pend;
        ts_id  = su_id;
        ts_isp = cur_isp;
        ts_tsp = cur_tsp;
        ts_tcw = cur_tcw;
        ts_ddx = acc_ddx;
        ts_ddy = acc_ddy;
        ts_c   = acc_c;
    end

    // ============================ sequential ============================
    integer q;
    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0; tsp_go <= 1'b0;
            sg_active <= 1'b0; sg_x <= '0; t_valid <= 1'b0;
            g_ready <= 1'b0;
            cur_gen <= GEN_MAX;           // force a clear-walk on the FIRST start (HW-safe
                                          // regardless of M10K power-up state)
            dd_clearing <= 1'b0;
            fwd0_valid <= 1'b0; fwd1_valid <= 1'b0;
            sf_wp <= '0; sf_rp <= '0;
            top_tag <= '0; tail <= '0; gf_wp <= '0; gf_rp <= '0;
            fetch_busy <= 1'b0; pend_v <= 1'b0; su_run <= 1'b0; ts_pend <= 1'b0;
            fx_start <= 1'b0; tsp_start <= 1'b0;
        end else begin
            fx_start  <= 1'b0;
            tsp_start <= 1'b0;
            tsp_go    <= 1'b0;            // 1-cyc pulse

            // ---------------- TSP freed a tile -> advance the ring tail ----------------
            if (tsp_rd_done && !gf_empty) begin
                tail  <= gf_mem[gf_rp[GF_AW-1:0]];   // free up to the oldest handed tile's end
                gf_rp <= gf_rp + 1'b1;
            end

            // ---------------- start a tile pass ----------------
            // Bump the generation so all prior-pass dedup buckets go stale for free ("don't
            // reuse when x/y change"). If gen would wrap, clear-walk first. When the ring is
            // idle (all handed tiles done - the standalone/sequential case) NORMALIZE it to 0
            // so ids restart dense at 0; when overlapped with TSP the head just continues.
            if (start) begin
                busy  <= 1'b1;
                fwd0_valid <= 1'b0; fwd1_valid <= 1'b0;
                sf_wp <= '0; sf_rp <= '0;
                if (ring_empty && !(tsp_rd_done && !gf_empty)) begin top_tag <= '0; tail <= '0; end
                if (cur_gen == GEN_MAX) begin
                    dd_clearing <= 1'b1; dd_clr_addr <= '0;   // SPANGEN idle until clear done
                end else begin
                    cur_gen <= cur_gen + 1'b1;
                    sg_x <= '0; sg_active <= 1'b1; t_valid <= 1'b0; g_ready <= 1'b0;
                end
            end

            // ---------------- dedup clear-walk (gen wrap) ----------------
            if (dd_clearing) begin
                dedup_ram[dd_clr_addr] <= '0;   // gen=0 -> permanently invalid
                if (dd_clr_addr == NSLOT-1) begin
                    dd_clearing <= 1'b0; cur_gen <= {{(GEN_W-1){1'b0}}, 1'b1};   // gen=1
                    sg_x <= '0; sg_active <= 1'b1; t_valid <= 1'b0; g_ready <= 1'b0;
                end else dd_clr_addr <= dd_clr_addr + 1'b1;
            end

            // =================== FSM 1: SPANGEN pipeline (COAL + EMIT) ===================
            // Whole pipeline freezes on pipe_stall (setup FIFO full on an allocating emit).
            if (!pipe_stall) begin
                // ---------- EMIT: retire the descriptor produced last cycle ----------
                // On a MISS, bump-allocate id=top_tag into the ring and write the map
                // bucket {cur_gen, tag, id}. span write + setup push are combinational (sp_*).
                if (t_valid && needs_alloc) begin
                    dedup_ram[t_h] <= {cur_gen, t_tag, top_tag[SLOTW-1:0]};
                    top_tag <= top_tag + 1'b1;    // advance ring head (wraps via SLOTW+1 bits)
                end
                // forwarding shift: fwd0 = alloc this cycle (bucket t_h -> {tag,id}).
                fwd1_valid <= fwd0_valid; fwd1_h <= fwd0_h; fwd1_id <= fwd0_id; fwd1_tag <= fwd0_tag;
                fwd0_valid <= t_valid && needs_alloc;
                fwd0_h     <= t_h;
                fwd0_id    <= top_tag[SLOTW-1:0];
                fwd0_tag   <= t_tag;

                // ---------- COAL: coalesce one span off ti_* (the current group) ----------
                if (coal_fires) begin
                    t_valid  <= 1'b1;
                    t_x      <= sg_x;
                    t_h      <= run_id;            // dedup map bucket = pc_slot(run_tag)
                    t_tag    <= run_tag;
                    t_rep    <= run_rep;
                    t_shmask <= run_shmask;
                    t_at     <= ti_pt[sg_lane];
                    for (q = 0; q < 4; q = q + 1)
                        t_invw[q] <= (q < run_rep) ? ti_invw[sg_lane + q[1:0]] : 32'd0;
                    // present the dedup read for THIS descriptor (resolves next cycle in EMIT)
                    dd_rd_q <= dedup_ram[run_id];

                    // advance. The read for sg_x_next's group is presented THIS cycle
                    // (rd_group tracks rd_next_x continuously) -> ti_* is correct next cycle.
                    sg_x <= sg_x_next;
                    if (walk_last) sg_active <= 1'b0;
                    // g_ready stays 1 (a valid read was presented this cycle).
                end else begin
                    // fill bubble (post start/stall): no descriptor; the read of sg_x's
                    // group was presented this cycle -> ready next cycle.
                    t_valid <= 1'b0;
                end

                // g_ready: a valid read was presented this cycle (sg_active && !pipe_stall
                // is implied by being in this block); it lands next cycle. Cleared only at
                // start (below) so the first COAL waits one fill cycle.
                if (sg_active) g_ready <= 1'b1;
            end

            // =================== FIFO push ===================
            if (sf_push && !sf_full) begin
                sf_mem[sf_wp[SF_AW-1:0]] <= sf_pdata;
                sf_wp <= sf_wp + 1'b1;
            end

            // =================== SETUP path (streaming stages) ===================
`ifndef SYNTHESIS
            su_dbg_cyc <= start ? 0 : su_dbg_cyc + 1;
            if ($test$plusargs("sutrace")) begin
                if (!fetch_busy && !pend_v && !sf_empty)
                    $display("[SU c%0d] FETCH issue tag=%08x id=%0d", su_dbg_cyc, sf_head[31:0], sf_head[SF_W-1:32]);
                if (fetch_busy && fx_done)      $display("[SU c%0d] FETCH done id=%0d", su_dbg_cyc, fx_id);
                if (!su_run && !ts_pend && (pend_v || fx_done))
                                                $display("[SU c%0d] SETUP start id=%0d", su_dbg_cyc, fx_done ? fx_id : pend_id);
                if (su_run && tsp_done)         $display("[SU c%0d] SETUP done id=%0d", su_dbg_cyc, su_id);
                if (ts_pend)                    $display("[SU c%0d] WRITE id=%0d", su_dbg_cyc, su_id);
            end
`endif
            // ---- FETCH issue: pop the FIFO into the fetcher whenever it is free and the
            // pend skid is empty (the skid frees when setup accepts, so fetch N+1 runs
            // DURING setup N). fetch_busy covers the fx_start..fx_busy visibility gap.
            if (!fetch_busy && !pend_v && !sf_empty) begin
                fx_tag     <= sf_head[31:0];
                fx_id      <= sf_head[SF_W-1:32];
                fx_start   <= 1'b1;
                sf_rp      <= sf_rp + 1'b1;    // pop
                fetch_busy <= 1'b1;
            end

            // ---- FETCH complete -> pend skid (or straight into setup if it is idle) ----
            if (fetch_busy && fx_done) begin
                fetch_busy <= 1'b0;
                pend_v     <= 1'b1;
                pend_id    <= fx_id;
                pend_isp <= fx_isp; pend_tsp <= fx_tsp; pend_tcw <= fx_tcw;
                for (q = 0; q < 3; q = q + 1) begin
                    pend_x[q]<=fx_x[q]; pend_y[q]<=fx_y[q]; pend_z[q]<=fx_z[q];
                    pend_u[q]<=fx_u[q]; pend_v3[q]<=fx_v[q];
                    pend_col[q]<=fx_col[q]; pend_ofs[q]<=fx_ofs[q];
                end
            end

            // ---- SETUP accept: whenever tsp_setup_min is idle (and the previous write
            // retired), latch the pending record into cur_*/fv_* (held stable for the whole
            // run) and start. Bypass: accept straight off fx_done the same cycle. ----
            if (!su_run && !ts_pend && (pend_v || fx_done)) begin
                if (pend_v) begin
                    cur_isp <= pend_isp; cur_tsp <= pend_tsp; cur_tcw <= pend_tcw;
                    for (q = 0; q < 3; q = q + 1) begin
                        fv_x[q]<=pend_x[q]; fv_y[q]<=pend_y[q]; fv_z[q]<=pend_z[q];
                        fv_u[q]<=pend_u[q]; fv_v[q]<=pend_v3[q];
                        fv_col[q]<=pend_col[q]; fv_ofs[q]<=pend_ofs[q];
                    end
                    su_id  <= pend_id;
                    pend_v <= 1'b0;
                end else begin  // fx_done bypass (skid empty, setup idle)
                    cur_isp <= fx_isp; cur_tsp <= fx_tsp; cur_tcw <= fx_tcw;
                    for (q = 0; q < 3; q = q + 1) begin
                        fv_x[q]<=fx_x[q]; fv_y[q]<=fx_y[q]; fv_z[q]<=fx_z[q];
                        fv_u[q]<=fx_u[q]; fv_v[q]<=fx_v[q];
                        fv_col[q]<=fx_col[q]; fv_ofs[q]<=fx_ofs[q];
                    end
                    su_id  <= fx_id;
                    pend_v <= 1'b0;   // consumed in flight, skid stays empty
                end
                acc_ddx <= '0; acc_ddy <= '0; acc_c <= '0;
                tsp_start <= 1'b1;
                su_run <= 1'b1;
            end

            // ---- SETUP run: collect streamed planes; done -> 1-cycle write pulse ----
            if (tsp_pvalid) begin
                acc_ddx[32*tsp_pidx +: 32] <= tsp_pddx;
                acc_ddy[32*tsp_pidx +: 32] <= tsp_pddy;
                acc_c  [32*tsp_pidx +: 32] <= tsp_pc;
            end
            if (su_run && tsp_done) begin
                su_run  <= 1'b0;
                ts_pend <= 1'b1;      // ts_we fires (combinational) next cycle
            end else if (ts_pend) begin
                ts_pend <= 1'b0;
            end

            // =================== busy / done -> tsp_go ===================
            // done when SPANGEN drained (not walking, EMIT stage empty) AND the whole setup
            // path is drained (FIFO, fetch, skid, setup run, write pulse). At that moment the
            // tile's setups are ALL in triangle_setups -> pulse tsp_go and record the tile's
            // END pointer (top_tag) so tsp_done can later free this tile's ring range.
            if (busy && !start && !dd_clearing && !sg_active && !t_valid && sf_empty
                && !fetch_busy && !pend_v && !su_run && !ts_pend) begin
                busy   <= 1'b0;
                tsp_go <= 1'b1;
                gf_mem[gf_wp[GF_AW-1:0]] <= top_tag;
                gf_wp <= gf_wp + 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    // one span write per cycle; ts write mutually exclusive per its FSM. Sanity: never
    // push a full FIFO, never pop an empty one.
    always @(posedge clk) if (!reset) begin
        if (sf_push && sf_full)
            $error("spanner_v2: setup FIFO overflow (push while full)");
    end
`endif
endmodule
