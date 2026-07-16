//
// plane_cache - the 64-entry TSP plane cache for peel_core, banked into M10K with
// its RAM port owned + the access pattern enforced by typed ports.
//
// Each entry is one shaded triangle's resolved TSP parameters: {tag, isp, tsp, tcw}
// + 30 plane coefficients (10 x ddx, 10 x ddy, 10 x c). As registers that was
// 64 x (4 + 30) x 32 = ~70 kbit of flip-flops (the bulk of peel_core's registers).
// Now BOTH the wide plane payload AND the 32-bit tag live in M10K, one word per
// entry, so the only logic-resident state is the 64-bit valid vector.
//
// Split storage:
//   * pc_ram  : {tag, isp, tsp, tcw, 10xddx, 10xddy, 10xc} in a REGISTERED-read
//     block RAM (M10K; M10K has no async read) -> 1-cycle read. The tag rides in
//     the same word and the hit compare happens AFTER the read (registered), so it
//     lands aligned with rd_valid/payload -- same protocol timing as before. This
//     removes the 64x32 tag register file and the 64:1 read mux it forced (~1k ALMs).
//   * valid[] : a 64-bit register, read COMBINATIONALLY (sampled at lookup) and
//     bulk-cleared in one cycle on `inval` (a RAM can't clear all entries at once).
//     tags[] doesn't need clearing -- valid[] alone gates the hit.
//
// Keyed by slot (direct-mapped; caller computes it) + full tag. Protocol:
//   LOOKUP: pulse lu_req with lu_slot/lu_tag. rd_valid + hit + the payload appear
//           the NEXT cycle (payload + tag from the registered RAM read, valid from
//           the sampled valid bit). `hit` already means valid && tag-match; the
//           caller need not compare.
//   WRITE : pulse wr_req with wr_slot + the full payload; entry stored, valid set.
//   INVAL : pulse inval to clear all valid bits (start of a shade sub-phase).
// Payload plane arrays cross the port as FLAT 320-bit vectors (10 x 32), lane j at
// [32*j +: 32], matching cur_ddx[j] etc. in the caller.
//
module plane_cache #(
    parameter integer NENT  = 64,
    parameter integer SLOTW = 6            // clog2(NENT)
) (
    input                 clk,
    input                 reset,

    input                 inval,           // clear all valid bits (1 cyc)

    // ---- LOOKUP (registered read, 1-cycle latency) ----
    input                 lu_req,
    input  [SLOTW-1:0]    lu_slot,
    input  [31:0]         lu_tag,
    output reg            rd_valid,        // 1 cyc after lu_req: outputs ready
    output                hit,             // valid && tag match (aligned w/ rd_valid)
    output [31:0]         o_isp,
    output [31:0]         o_tsp,
    output [31:0]         o_tcw,
    output [319:0]        o_ddx,           // 10 x 32, lane j at [32*j +: 32]
    output [319:0]        o_ddy,
    output [319:0]        o_c,

    // ---- WRITE (commit a resolved entry) ----
    input                 wr_req,
    input  [SLOTW-1:0]    wr_slot,
    input  [31:0]         wr_tag,
    input  [31:0]         wr_isp,
    input  [31:0]         wr_tsp,
    input  [31:0]         wr_tcw,
    input  [319:0]        wr_ddx,
    input  [319:0]        wr_ddy,
    input  [319:0]        wr_c
);
    // wide payload layout inside pc_ram (LSB-first). The tag rides in the same word.
    localparam integer PW_TAG = 0;
    localparam integer PW_ISP = 32;
    localparam integer PW_TSP = 64;
    localparam integer PW_TCW = 96;
    localparam integer PW_DDX = 128;        // 320 bits
    localparam integer PW_DDY = 448;        // 320 bits
    localparam integer PW_C   = 768;        // 320 bits
    localparam integer EW     = 1088;       // 4*32 + 3*320

    // only logic-resident state: the valid vector (combinational read, bulk-clearable)
    reg [NENT-1:0] valid;

    // wide payload + tag store: registered-read M10K
    (* ramstyle = "M10K, no_rw_check" *) reg [EW-1:0] pc_ram [0:NENT-1];
    reg [EW-1:0] rd_word;                    // registered read data (1-cyc)

    // ---- 1-ENTRY VICTIM cache (registers) ----
    // The main store is DIRECT-MAPPED, so two tags that hash to the same slot evict
    // each other and re-miss within a pass (measured: e.g. 0x...613b and 0x...6932 both
    // map to slot 0x24 and ping-pong). Keep the LAST-EVICTED entry in a register and
    // check it in parallel with the main lookup, so an alternating pair hits instead of
    // re-missing. Captured on a WRITE that overwrites a valid entry with a DIFFERENT tag
    // (a conflict eviction): the old occupant is read back (M10K read-first, wr_slot read
    // on the write cycle -> old word next cycle) and latched here.
    reg          vic_valid;
    reg [EW-1:0] vic_word;
    reg          vic_cap;                    // 1-cyc: capture the evicted word next cycle
    reg [EW-1:0] evict_rd;                   // registered read of the overwritten slot

    // assemble the write word from the payload ports (tag now rides in pc_ram too)
    wire [EW-1:0] wr_word;
    assign wr_word[PW_TAG +: 32]  = wr_tag;
    assign wr_word[PW_ISP +: 32]  = wr_isp;
    assign wr_word[PW_TSP +: 32]  = wr_tsp;
    assign wr_word[PW_TCW +: 32]  = wr_tcw;
    assign wr_word[PW_DDX +: 320] = wr_ddx;
    assign wr_word[PW_DDY +: 320] = wr_ddy;
    assign wr_word[PW_C   +: 320] = wr_c;

    // valid[lu_slot] is sampled combinationally at lookup and registered to align
    // with the registered RAM read; lu_tag is likewise held so the tag compare can be
    // done COMBINATIONALLY off rd_word the cycle rd_valid is high -> hit stays aligned
    // with rd_valid/payload, exactly as the old register-mirror version.
    reg        lu_valid_q;                   // valid[lu_slot], aligned with rd_word
    reg [31:0] lu_tag_q;                      // lu_tag, aligned with rd_word

    reg [31:0] evict_wtag;                    // wr_tag of the write that may evict
    always @(posedge clk) begin
        if (reset) begin
            valid     <= '0;
            rd_valid  <= 1'b0;
            vic_valid <= 1'b0;
            vic_cap   <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            vic_cap  <= 1'b0;

            // WRITE: store wide payload (tag included) + set valid. If the slot was
            // already valid, its old occupant is being evicted -> read it back this
            // cycle (M10K read-first returns OLD data on same-addr r+w) and arm a
            // victim capture for next cycle (only if the old tag differs from wr_tag).
            if (wr_req) begin
                evict_rd   <= pc_ram[wr_slot];   // OLD word (read-first) before overwrite
                evict_wtag <= wr_tag;
                if (valid[wr_slot]) vic_cap <= 1'b1;
                pc_ram[wr_slot] <= wr_word;
                valid [wr_slot] <= 1'b1;
                // if the write's tag matches the current victim, the victim is now stale
                // (that entry is back in the main cache) -> drop it.
                if (vic_valid && vic_word[PW_TAG +: 32] == wr_tag) vic_valid <= 1'b0;
            end
            // capture the evicted entry into the victim (a cycle after the write), unless
            // it was just a same-tag refresh (no real eviction).
            if (vic_cap && (evict_rd[PW_TAG +: 32] != evict_wtag)) begin
                vic_valid <= 1'b1;
                vic_word  <= evict_rd;
            end

            // LOOKUP: registered payload+tag read; sample valid + hold lu_tag so the
            // combinational hit test below lands aligned with rd_valid.
            lu_valid_q <= lu_req && valid[lu_slot];
            lu_tag_q   <= lu_tag;
            if (lu_req) begin
                rd_word  <= pc_ram[lu_slot];
                rd_valid <= 1'b1;
            end

            // INVALIDATE all: single-cycle clear (main + victim).
            if (inval) begin valid <= '0; vic_valid <= 1'b0; end
        end
    end

    // hit: main slot (valid + tag match) OR the victim (valid + tag match). Both land
    // aligned with rd_valid (victim is a register, held stable).
    wire main_hit = lu_valid_q && (rd_word[PW_TAG +: 32] == lu_tag_q);
    wire vic_hit  = vic_valid  && (vic_word[PW_TAG +: 32] == lu_tag_q);
    assign hit = rd_valid && (main_hit || vic_hit);

    // payload outputs: from the victim on a victim-only hit, else the main RAM word.
    wire [EW-1:0] sel_word = (!main_hit && vic_hit) ? vic_word : rd_word;
    assign o_isp = sel_word[PW_ISP +: 32];
    assign o_tsp = sel_word[PW_TSP +: 32];
    assign o_tcw = sel_word[PW_TCW +: 32];
    assign o_ddx = sel_word[PW_DDX +: 320];
    assign o_ddy = sel_word[PW_DDY +: 320];
    assign o_c   = sel_word[PW_C   +: 320];
endmodule
