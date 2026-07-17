//
// color_tile_buffer - the shaded-ARGB accumulation buffer for one 32x32 tile,
// banked into a single M10K, with its access pattern ENFORCED by typed ports.
//
// col_buf is low-load: only the TSP blend (1 px/cyc RMW) and the FLUSH readout
// (1 px/cyc) ever touch it, never LANES-wide. So it is a single-bank 1024x32 M10K
// (1R1W, registered read), addressed by the full pixel index (0..1023).
//
// The blend ALU is NOT in here: only the producer half of the ping-pong ever
// blends, so peel_core owns ONE shared tsp_blend (fed by this buffer's
// registered CA read) and registers its result (stage CW) - one set of blend
// multipliers instead of one per half, and the RAM-write tail is off the
// critical path. The RMW is therefore 3 stages:
//   stage CA (bl_ca_valid): present the col-RAM read of bl_ca_id (the dst).
//   stage CB (in peel_core): rd_argb = OLD col_buf[dst]; tsp_blend runs
//     combinationally; the result is registered (cw_*).
//   stage CW (wr_valid):    the registered blend result is written here.
// The shade pipeline is in-order and pixel ids strictly ascend within a
// sub-phase, so neither a CA read (later id) nor the delayed CW write
// (earlier id) can collide on an address; a pixel is only re-blended on a
// later pass, many cycles after its write landed.
//
// Read clients  (at most one/cycle): blend stage CA | FLUSH read.
// Write clients (at most one/cycle): blend stage CW.
// The module owns the ports and asserts the exclusion (sim).
//
module color_tile_buffer import tsp_pkg::*; #(
    parameter integer DEPTH = 1024
) (
    input                       clk,
    input                       reset,

    // ---- BLEND stage CA: present the read of the dst for out-pixel bl_ca_id ----
    input                       bl_ca_valid,
    input      [9:0]            bl_ca_id,

    // ---- BLEND stage CW: write the (externally blended, registered) pixel ----
    input                       wr_valid,
    input      [9:0]            wr_id,
    input      [31:0]           wr_argb,

    // ---- FLUSH: hold the read of pixel fl_id; fl_argb is the registered read ----
    input                       fl_rd_valid,
    input      [9:0]            fl_id,
    output     [31:0]           rd_argb      // registered read data (dst / flush pixel)
);
    reg  [9:0]   raddr;
    (* ramstyle = "M10K, no_rw_check" *) reg [31:0] col_ram [0:DEPTH-1];
    reg  [31:0]  rdata_r;
    always @(posedge clk) begin
        if (wr_valid) col_ram[wr_id] <= wr_argb;
        rdata_r <= col_ram[raddr];          // registered read (read-first)
    end
    assign rd_argb = rdata_r;

    // -------------------- READ port mux --------------------
    always @(*) begin
        raddr = 10'd0;
        if (bl_ca_valid)      raddr = bl_ca_id;
        else if (fl_rd_valid) raddr = fl_id;
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (!reset) begin
        if ((bl_ca_valid + fl_rd_valid) > 1)
            $error("color_tile_buffer: multiple READ clients (%b%b)",
                   bl_ca_valid, fl_rd_valid);
    end
`endif
endmodule
