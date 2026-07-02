//
// color_tile_buffer - the shaded-ARGB accumulation buffer for one 32x32 tile,
// banked into a single M10K, with its access pattern ENFORCED by typed ports.
//
// col_buf is low-load: only the TSP blend (1 px/cyc RMW) and the FLUSH readout
// (1 px/cyc) ever touch it, never LANES-wide. So it is a single-bank 1024x32 M10K
// (1R1W, registered read), addressed by the full pixel index (0..1023).
//
// The blend is a 2-stage RMW because the read is registered:
//   stage CA (bl_ca_valid): present the col-RAM read of bl_ca_id (the dst).
//   stage CB (bl_cb_valid): rdata = OLD col_buf[cb_id] = dst; tsp_blend runs
//     combinationally; the result is written back to col_ram[cb_id].
// The shade pipeline is in-order and pixel ids ascend within a sub-phase, so a CA
// read and a CB write never hit the same address in the same cycle (no RMW hazard).
//
// Read clients  (at most one/cycle): blend stage CA | FLUSH read.
// Write clients (at most one/cycle): blend stage CB.
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

    // ---- BLEND stage CB: blend (src over rdata) and write col_ram[cb_id] ----
    input                       bl_cb_valid,
    input      [9:0]            cb_id,
    input      [31:0]           cb_argb,     // shaded source color
    input      [31:0]           cb_tsp,      // TSP word (SrcInstr/DstInstr)
    input                       cb_at_en,    // PT alpha-test enable
    input      [7:0]            alpha_ref,   // PT_ALPHA_REF

    // ---- FLUSH: hold the read of pixel fl_id; fl_argb is the registered read ----
    input                       fl_rd_valid,
    input      [9:0]            fl_id,
    output     [31:0]           rd_argb      // registered read data (dst / flush pixel)
);
    reg          we;
    reg  [9:0]   waddr, raddr;
    reg  [31:0]  wdata;
    (* ramstyle = "M10K, no_rw_check" *) reg [31:0] col_ram [0:DEPTH-1];
    reg  [31:0]  rdata_r;
    always @(posedge clk) begin
        if (we) col_ram[waddr] <= wdata;
        rdata_r <= col_ram[raddr];          // registered read (read-first)
    end
    assign rd_argb = rdata_r;

    // -------------------- blend (stage CB, combinational off rdata_r) --------------------
    wire [2:0]  cb_src_instr = cb_tsp[31:29];
    wire [2:0]  cb_dst_instr = cb_tsp[28:26];
    wire [31:0] blend_out;
    wire        blend_at;
    tsp_blend u_blend (
        .src       (cb_argb),
        .dst       (rdata_r),               // registered read: OLD col_buf[cb_id]
        .src_instr (cb_src_instr),
        .dst_instr (cb_dst_instr),
        .alpha_test(cb_at_en),
        .alpha_ref (alpha_ref),
        .out       (blend_out),
        .at_pass   (blend_at));

    // -------------------- READ port mux --------------------
    always @(*) begin
        raddr = 10'd0;
        if (bl_ca_valid)      raddr = bl_ca_id;
        else if (fl_rd_valid) raddr = fl_id;
    end

    // -------------------- WRITE port (blend stage CB) --------------------
    always @(*) begin
        we    = bl_cb_valid;
        waddr = cb_id;
        wdata = blend_out;
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (!reset) begin
        if ((bl_ca_valid + fl_rd_valid) > 1)
            $error("color_tile_buffer: multiple READ clients (%b%b)",
                   bl_ca_valid, fl_rd_valid);
    end
`endif
endmodule
