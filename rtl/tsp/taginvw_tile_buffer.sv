//
// taginvw_tile_buffer - the ISP->TSP handoff buffer: the TSP-facing SLICE of the
// peel depth/tag buffer, split out so it can be PING-PONGED independently of the
// ISP-private u_peel scratch (depth/depth2/tag2 + compare + PeelBuffers RMW).
//
// It holds only the three fields TSP shade actually reads - {valid, tag, invW} -
// and is WRITTEN as a DUPLICATE of peel_tile_buffer's stage-B accept (same write
// data, same enable) plus the CLEAR walk. It has NO depth compare, no depth2/tag2,
// and no PeelBuffers walk: those stay ISP-private in u_peel. Because ISP already
// decided which lanes pass (peel_tile_buffer computes b_pass_lp / ras_pass_op),
// this module is told directly which lanes to write and what to write - it is a
// pure banked store, not a compare.
//
// Storage: ONE simple-dual-port tile_ram, WIDTH = 65 bits/lane {valid, tag[31:0],
// invW[31:0]}, NBANKS = LANES. Same banking as peel_tile_buffer: bank = x[BW-1:0],
// addr = {y[4:0], x[4:BW]}. One read port (shade single-pixel) + one write port
// (raster stage-B duplicate | CLEAR).
//
// Read clients  (at most one/cycle): shade single-pixel read.
// Write clients (at most one/cycle): raster stage-B duplicate | CLEAR.
// The peel_core credit handshake serializes producers/consumers per ping-pong
// half, so at most one read and one write client is asserted per cycle; the module
// asserts this (sim).
//
module taginvw_tile_buffer import tsp_pkg::*; #(
    parameter integer LANES = 8
) (
    input                       clk,
    input                       reset,

    // ---- RASTER stage-B duplicate: write {valid,tag,invW} for the passing lanes ----
    // wr_we[l] is peel_tile_buffer's per-lane accept for lane l (already masked by
    // inside/pass), wr_y/wr_x the chunk (LANES-aligned), wr_invw the per-lane invW.
    input                       wr_valid,       // any lane may write this cycle
    input      [LANES-1:0]      wr_we,          // per-lane write-enable (accept)
    input      [4:0]            wr_y,
    input      [4:0]            wr_x,           // chunk base (LANES-aligned)
    input      [31:0]           wr_tag,         // fragment CoreTag (same for all lanes)
    input      [32*LANES-1:0]   wr_invw,        // per-lane invW (flat)
    input                       wr_pt,          // PT-list won (b_which==0): the blend's
                                                // PT alpha-test enable, captured HERE at
                                                // raster stage-B (dt_pt is stale by the
                                                // time the decoupled reader blends).

    // ---- CLEAR / PEELBUFFERS valid-clear: now LAZY (1-cyc register pulses, no RAM walk) ----
    // clr_all pulses at tile CLEAR: invalidate the whole tile (valid_r<='0) + latch bg tag/invW.
    // pbc_all pulses per peel pass: invalidate valid_r (mirrors u_peel's PeelBuffers valid<-0)
    // so pass P+1 doesn't re-shade pass P's staged pixels. A pixel with valid_r==0 reads
    // {valid:0, tag:bg, invW:bg} via the read defaults below (matches the old CLEAR-wrote-bg).
    input                       clr_all,        // 1-cyc: invalidate whole tile + latch bg
    input                       pbc_all,        // 1-cyc: per-pass valid clear
    input      [31:0]           clr_depth,      // background invW/depth
    input      [31:0]           clr_tag,        // background CoreTag

    // ---- SHADE: single-pixel read (id = {y[4:0], x[4:0]}) ----
    input                       sh_rd_valid,
    input      [9:0]            sh_rd_id,
    output reg                  sh_valid,       // staged-this-pass bit  (1-cyc latency)
    output reg [31:0]           sh_tag,         // pending tag           (1-cyc latency)
    output reg [31:0]           sh_depth,       // depthBufferA (invW)   (1-cyc latency)
    output reg                  sh_pt,          // PT-list-won bit       (1-cyc latency)

    // ---- SPANNER: 4-wide ALIGNED read (group = x & ~3). REQUIRES LANES==4: the 4 aligned
    // pixels {g..g+3} then map exactly to banks 0..3, so one read of all banks at
    // addr {g[9:5], g[4:2]} returns the whole group. Lane l = pixel (g|l). 1-cyc latency.
    input                       rd4_valid,
    input      [9:0]            rd4_group,
    output     [3:0]            g4_valid,
    output     [31:0]           g4_tag  [0:3],
    output     [31:0]           g4_invw [0:3],
    output     [3:0]            g4_pt
);
    localparam integer NB        = LANES;
    localparam integer BANK_BITS = $clog2(LANES);   // 3 for 8, 2 for 4
    localparam integer AW        = 10 - BANK_BITS;   // per-bank addr width (7 / 8)
    localparam integer TW_INVW   = 0;    // [31:0] depthBufferA (invW)
    localparam integer TW_TAG    = 32;   // [31:0] tagBufferA
    localparam integer TW_VALID  = 64;   // [0]    tagStatus.valid
    localparam integer TW_PT     = 65;   // [0]    PT-list-won (blend alpha-test enable)
    localparam integer TI_W      = 66;

    reg  [NB-1:0]        we;
    reg  [AW*NB-1:0]     waddr;
    reg  [AW*NB-1:0]     raddr;
    reg  [TI_W*NB-1:0]   wdata;
    wire [TI_W*NB-1:0]   rdata;
    tile_ram #(.WIDTH(TI_W), .NBANKS(NB)) u_ram (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(raddr), .rdata(rdata)
    );

    // -------------------- LAZY-CLEAR per-pixel valid register --------------------
    // `valid` (TW_VALID) is now a 1024-bit FF array cleared in ONE cycle (clr_all / pbc_all),
    // NOT walked into the RAM. A pixel with valid_r==0 reads {valid:0, tag:bg, invW:bg} via the
    // read defaults. TW_VALID in the RAM word is unused. (This module is per-half; each instance
    // owns its own valid_r.)
    reg  [1023:0] valid_r;
    reg  [31:0]   bg_tag_r, bg_invw_r;   // latched at clr_all (background for un-written px)
    // pixel index of lane b in the chunk at (y, xchunk-base): {y[4:0], x[4:0]}.
    function automatic [9:0] pix_idx(input [4:0] y, input [4:0] xchunk, input integer b);
        pix_idx = {y, xchunk[4:BANK_BITS], b[BANK_BITS-1:0]};
    endfunction

    // pack an AW-bit bank address {y[4:0], x[4:BANK_BITS]} onto all NB banks
    function automatic [AW*NB-1:0] pack_addr(input [4:0] y, input [4:0] xchunk);
        integer b;
        begin
            pack_addr = '0;
            for (b = 0; b < NB; b = b + 1)
                pack_addr[AW*b +: AW] = {y, xchunk[4:BANK_BITS]};
        end
    endfunction

    // -------------------- READ port (single-pixel OR 4-wide aligned group) --------------
    always @(*) begin
        raddr = '0;
        if (rd4_valid)        raddr = {NB{ {rd4_group[9:5], rd4_group[4:BANK_BITS]} }};
        else if (sh_rd_valid) raddr = {NB{ {sh_rd_id[9:5],  sh_rd_id[4:BANK_BITS]}  }};
    end

    // Latch the read group/id so valid_r can be indexed at the OUTPUT cycle (T+1), aligned with
    // the registered rdata. (valid_r is a register read combinationally; rdata is 1-cyc delayed.)
    reg [9:0] rd4_group_r;
    always @(posedge clk) begin
        if (reset) rd4_group_r <= '0;
        else if (rd4_valid) rd4_group_r <= rd4_group;
    end

    // 4-wide group outputs: with LANES==4, banks 0..3 ARE lanes 0..3 of the aligned group
    // (combinational off the registered read rdata, 1-cyc after rd4_group presented). A lane
    // with valid_r==0 (not written since clear) reads {valid:0, tag:bg, invW:bg, pt:0}.
    genvar gl;
    generate
      for (gl = 0; gl < 4; gl = gl + 1) begin : g4lane
        wire v = valid_r[ {rd4_group_r[9:2], gl[1:0]} ];   // pixel (group | lane), LANES==4
        assign g4_valid[gl] = v;
        assign g4_pt   [gl] = v ? rdata[TI_W*gl + TW_PT]         : 1'b0;
        assign g4_tag  [gl] = v ? rdata[TI_W*gl + TW_TAG  +: 32] : bg_tag_r;
        assign g4_invw [gl] = v ? rdata[TI_W*gl + TW_INVW +: 32] : bg_invw_r;
      end
    endgenerate

    // -------------------- WRITE port mux --------------------
    // Only the raster stage-B accept-duplicate writes the RAM now (CLEAR + PeelBuffers valid
    // walks are the lazy register clears). valid is a register, not a RAM field.
    integer cw;
    always @(*) begin
        we    = '0;
        waddr = '0;
        wdata = '0;
        if (wr_valid) begin                        // stage-B accept duplicate
            waddr = pack_addr(wr_y, wr_x);
            for (cw = 0; cw < NB; cw = cw + 1) begin
                we[cw] = wr_we[cw];
                wdata[TI_W*cw + TW_INVW +: 32] = wr_invw[32*cw +: 32];
                wdata[TI_W*cw + TW_TAG  +: 32] = wr_tag;
                wdata[TI_W*cw + TW_VALID]      = 1'b1;     // (RAM valid field unused; kept 1)
                wdata[TI_W*cw + TW_PT]         = wr_pt;    // PT-list-won (same for all lanes)
            end
        end
    end

    // -------------------- CONTROL: valid_r / bg (lazy CLEAR + per-pass clear) --------------
    // clr_all : whole-tile invalidate + latch bg tag/invW. pbc_all : per-pass valid clear.
    // stage B : each accepted lane (wr_we) marks its pixel valid_r<-1.
    always @(posedge clk) begin
        if (reset) begin
            valid_r <= '0; bg_tag_r <= '0; bg_invw_r <= '0;
        end else if (clr_all) begin
            valid_r <= '0; bg_tag_r <= clr_tag; bg_invw_r <= clr_depth;
        end else if (pbc_all) begin
            valid_r <= '0;
        end else if (wr_valid) begin
            for (cw = 0; cw < NB; cw = cw + 1)
                if (wr_we[cw]) valid_r[ pix_idx(wr_y, wr_x, cw) ] <= 1'b1;
        end
    end

    // -------------------- SHADE single-pixel read output --------------------
    // 1-cycle latency: sh_rd_valid presented this cycle -> fields next cycle.
    reg [BANK_BITS-1:0] sh_lane_r;
    always @(posedge clk) begin
        if (reset) sh_lane_r <= '0;
        else if (sh_rd_valid) sh_lane_r <= sh_rd_id[BANK_BITS-1:0];
    end
    always @(*) begin
        sh_valid = rdata[TI_W*sh_lane_r + TW_VALID];
        sh_tag   = rdata[TI_W*sh_lane_r + TW_TAG  +: 32];
        sh_depth = rdata[TI_W*sh_lane_r + TW_INVW +: 32];
        sh_pt    = rdata[TI_W*sh_lane_r + TW_PT];
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (!reset) begin
        // Only stage-B writes the RAM now; clr_all/pbc_all are register-only pulses.
        if (clr_all && pbc_all)
            $error("taginvw_tile_buffer: clr_all and pbc_all same cycle");
    end
`endif
endmodule
