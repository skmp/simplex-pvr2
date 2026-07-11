//
// isp_sort_cache - per-triangle "fully processed / nothing left visible" tracker
// for the PT/TL layer-peel (RM_TRANSLUCENT_AUTOSORT) loop. VERILATOR-ONLY for now
// (combinational-read RAM; not synthesizable as written - no M10K template).
//
// A direct-mapped, 512-entry cache indexed by the low 9 bits of the triangle's
// CoreTag (== low bits of {param_offs_in_words, tag_offset}). Each entry stores the
// FULL 32-bit CoreTag it currently owns plus a single `done` bit meaning "this
// triangle has no more visible pixels this pass -> a later peel pass may SKIP it".
// A lookup HITs only when the stored tag matches the queried tag exactly (so an
// index collision from a different triangle is a MISS, not a false skip).
//
// Bulk clear uses a GENERATION counter (not a per-entry reset loop, which Verilator
// cannot do with non-blocking array writes): each entry stores the `gen` it was last
// written under; an entry is live only if e_gen==cur_gen. `clr` bumps cur_gen, so the
// whole cache goes stale in one cycle with no walk.
//
// Semantics (bit = 1 => fully occluded / skip):
//   * reset / clr : bump generation -> every entry stale (lookups MISS).
//   * set (A/B)   : install tag at its index with done=1 and gen=cur_gen. Two ports so
//                   two producers (skip-forward + before-raster) can set DIFFERENT tags
//                   in one cycle; port B wins a same-index tie.
//   * clr1 (per lane): if a lane's tag HITs a live entry, force its done bit to 0 (a
//                   Z-write displaced one of that triangle's pixels -> not done). A SET
//                   to the same index this cycle overrides (SET wins -> done stays 1).
//   * lookup(tag) : hit = live && stored tag == tag; done = that entry's done bit.
//
// Combinational reads keep the caller's single-cycle fetch-gate / raster / stage-B
// decisions timing-neutral. Writes are synchronous.
//
`ifndef SYNTHESIS
module isp_sort_cache #(
    parameter integer ENTRIES = 512,
    parameter integer IDXW    = 9,           // clog2(ENTRIES)
    parameter integer LANES   = 4,           // per-cycle CLR1 requests (one per raster lane)
    parameter integer GENW    = 16,          // generation counter width
    parameter [31:0]  INVALID = 32'hFFFFFFFF // sentinel core tag (never a real tag)
) (
    input             clk,
    input             reset,

    // ---- bulk clear (per-pass for the write copy; per-tile for both) ----
    input             clr,                   // 1: invalidate the WHOLE cache this cycle

    // ---- lookup (combinational: present tag -> hit/done same cycle) ----
    input      [31:0] lk_tag,
    output            lk_hit,
    output            lk_done,

    // ---- write: SET x2 (install tag, done=1) ----
    input             set_valid,             // port A
    input      [31:0] set_tag,
    input             set_b_valid,           // port B (wins same-index tie)
    input      [31:0] set_b_tag,

    // ---- write: CLR1 (per-lane; if a lane's tag hits, force its done=0) ----
    input      [LANES-1:0]     c1_valid,
    input      [32*LANES-1:0]  c1_tag
);
    // storage: {gen, done, tag[31:0]} per entry
    reg [31:0]     e_tag  [0:ENTRIES-1];
    reg            e_done [0:ENTRIES-1];
    reg [GENW-1:0] e_gen  [0:ENTRIES-1];
    reg [GENW-1:0] cur_gen;

    function automatic [IDXW-1:0] idx(input [31:0] t);
        idx = t[IDXW-1:0];
    endfunction
    // an entry is LIVE (belongs to the current generation) and matches `t`
    function automatic live_hit(input [IDXW-1:0] i, input [31:0] t);
        live_hit = (e_gen[i] == cur_gen) && (e_tag[i] == t) && (t != INVALID);
    endfunction

    // ---- combinational lookup ----
    wire [IDXW-1:0] lk_i = idx(lk_tag);
    assign lk_hit  = live_hit(lk_i, lk_tag);
    assign lk_done = lk_hit && e_done[lk_i];

    wire [IDXW-1:0] set_i   = idx(set_tag);
    wire [IDXW-1:0] set_b_i = idx(set_b_tag);
    function automatic set_hits(input [IDXW-1:0] i);
        set_hits = (set_valid && set_i == i) || (set_b_valid && set_b_i == i);
    endfunction

    integer l;
    reg [31:0]     cl_tag;
    reg [IDXW-1:0] cl_i;
    always @(posedge clk) begin
        if (reset)      cur_gen <= '0;
        else if (clr)   cur_gen <= cur_gen + 1'b1;   // whole cache stale in one cycle

        if (!reset) begin
            // CLR1 (per lane): clear the displaced triangle's done bit on a live hit,
            // unless a SET targets the same index this cycle (SET wins).
            for (l = 0; l < LANES; l = l + 1) begin
                cl_tag = c1_tag[32*l +: 32];
                cl_i   = idx(cl_tag);
                if (c1_valid[l] && live_hit(cl_i, cl_tag) && !set_hits(cl_i))
                    e_done[cl_i] <= 1'b0;
            end
            // SET installs tag+done=1 under the current gen. Port B wins a same-index tie.
            if (set_valid) begin
                e_tag [set_i] <= set_tag;  e_done[set_i] <= 1'b1;  e_gen[set_i] <= cur_gen;
            end
            if (set_b_valid) begin
                e_tag [set_b_i] <= set_b_tag; e_done[set_b_i] <= 1'b1; e_gen[set_b_i] <= cur_gen;
            end
        end
    end
endmodule
`endif
