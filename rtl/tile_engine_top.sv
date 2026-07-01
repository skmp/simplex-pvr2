//
// tile_engine_top - simple command-driven tile engine for simplex.
//
// A self-contained MiSTer-style top level. DDR3 is reached through the HPS
// SDRAM bridge (sysmem_lite, in rtl/mister/), NOT through FPGA pins - so this
// top has almost no external I/O. sysmem_lite provides the 100MHz core clock
// and the reset; the DC_MiSTer `ddram` controller (rtl/mister/ddram.sv) sits
// between it and the tile engine. A 7-bit command / 32-bit data interface
// drives the 32x32 tile pipeline.
//
//   Clock / reset:
//     - The whole design runs on `core_clk` (100MHz) from the HPS bridge.
//     - `reset_req` (async input) requests a core reset; `core_reset` is the
//       synchronous active-high reset actually used inside the design.
//     - `cold_req` maps to the HPS cold-reset button (DE10-nano has none on
//       GPIO), tie 0 if unused.
//
//   Command / data interface (synchronous to core_clk):
//     - cmd[6:0], data[31:0], cmd_valid : host drives cmd+data and pulses
//       cmd_valid=1 for one cycle to submit a command.
//     - cmd_done : output, goes 1 for one cycle *one clock before* the engine
//       is ready to accept a new command.
//     - ready    : level output, 1 while idle and able to accept a command.
//
//   Commands:
//     0..63   : write `data` into register file entry cmd[5:0].
//     64..127 : engine command (data ignored for now):
//       64 TILE_CLEAR           : depth/tag tile <= {REG_BG_DEPTH, REG_BG_TAG}
//       65 TRIANGLE_ISP_SETUP   : compute edge/invW-plane coeffs (isp_setup_min)
//       66 TRIANGLE_ISP_RASTERIZE: opaque; one line/clock, depth test + tag write
//       67 TRIANGLE_TSP_SETUP    : nop
//       68 TRIANGLE_TSP_SHADE    : nop
//       69 TILE_DRAW_TAGS        : color[y][x] <= {8'hFF, tag[y][x]}
//       70 TILE_FLUSH            : color tile -> DDR3 at REG_TILE_BASE + y*32+x
//
// The depth/tag buffer holds {32-bit depth, 24-bit tag}; the color buffer holds
// a 32-bit color per pixel. Both are 32x32 (1024 entries).
//
module tile_engine_top #(
    parameter integer TILE_W   = 32,
    parameter integer TILE_H   = 32,
    parameter integer TAG_BITS = 24
) (
    // ---- HPS reset control (no DDR3 pins: DDR3 is on the HPS) ----
    input             reset_req,   // async: request a core reset
    input             cold_req,    // HPS cold-reset button (tie 0 if unused)
    output            core_clk,    // 100MHz core clock (from HPS bridge)
    output            core_reset,  // active-high synchronous core reset

    // ---- command / data interface ----
    input      [6:0]  cmd,
    input      [31:0] data,
    input             cmd_valid,   // 1-cycle strobe: submit cmd+data
    output reg        cmd_done,    // 1-cycle pulse, one clock before ready
    output            ready        // level: idle and able to accept a command
);
    localparam integer N_PIX   = TILE_W * TILE_H;         // 1024
    localparam integer IDX_W   = $clog2(N_PIX);           // 10
    localparam integer XW      = $clog2(TILE_W);          // 5
    localparam integer YW      = $clog2(TILE_H);          // 5
    localparam [IDX_W-1:0] LAST_IDX = IDX_W'(N_PIX - 1);  // last pixel index

    // ---- register file ----
    reg [31:0] regs [0:63];

    // Named registers (indices are part of the host ABI).
    localparam REG_BG_DEPTH = 6'd0;
    localparam REG_BG_TAG   = 6'd1;
    localparam REG_TILE_BASE= 6'd2;

    // ISP triangle setup inputs (host writes these before CMD_TRIANGLE_ISP_SETUP)
    localparam REG_X1 = 6'd8,  REG_Y1 = 6'd9,  REG_Z1 = 6'd10;
    localparam REG_X2 = 6'd11, REG_Y2 = 6'd12, REG_Z2 = 6'd13;
    localparam REG_X3 = 6'd14, REG_Y3 = 6'd15, REG_Z3 = 6'd16;
    localparam REG_X_BASE = 6'd17, REG_Y_BASE = 6'd18;   // tile origin (screen)
    localparam REG_ISP_WORD = 6'd19;                     // params->isp (Cull/Depth/ZWrite)
    localparam REG_ISP_TAG  = 6'd20;                     // parameter_tag_t written on pass

    // Convenient reads of the input registers (assign-from-regs style).
    wire [31:0] isp_x1 = regs[REG_X1], isp_y1 = regs[REG_Y1], isp_z1 = regs[REG_Z1];
    wire [31:0] isp_x2 = regs[REG_X2], isp_y2 = regs[REG_Y2], isp_z2 = regs[REG_Z2];
    wire [31:0] isp_x3 = regs[REG_X3], isp_y3 = regs[REG_Y3], isp_z3 = regs[REG_Z3];
    wire [31:0] isp_xbase = regs[REG_X_BASE], isp_ybase = regs[REG_Y_BASE];
    wire [31:0] isp_word  = regs[REG_ISP_WORD];

    // ---- command opcodes ----
    localparam CMD_TILE_CLEAR            = 7'd64;
    localparam CMD_TRIANGLE_ISP_SETUP    = 7'd65;
    localparam CMD_TRIANGLE_ISP_RASTERIZE= 7'd66;
    localparam CMD_TRIANGLE_TSP_SETUP    = 7'd67;
    localparam CMD_TRIANGLE_TSP_SHADE    = 7'd68;
    localparam CMD_TILE_DRAW_TAGS        = 7'd69;
    localparam CMD_TILE_FLUSH            = 7'd70;

    // ---- tile buffers ----
    // Organized as TILE_W banks (one per x column), each TILE_H deep (indexed by
    // y). This lets the rasterizer write/read a whole scanline (all x, fixed y)
    // in a single clock. Linear index idx = y*TILE_W + x maps to
    //   bank x = idx[XW-1:0], row y = idx[IDX_W-1:XW].
    // depth/tag entry: {32-bit invW depth, TAG_BITS tag}
    (* ramstyle = "M10K" *) reg [32+TAG_BITS-1:0] dt_buf  [0:TILE_W-1][0:TILE_H-1];
    // color: 32-bit
    (* ramstyle = "M10K" *) reg [31:0]            col_buf [0:TILE_W-1][0:TILE_H-1];

    // linear-index split helpers (for the CLEAR/DRAWTAGS/FLUSH iterators)
    wire [XW-1:0] idx_x;
    wire [YW-1:0] idx_y;

    // ------------------------------------------------------------------
    // HPS DDR3 bridge + ddram controller
    // ------------------------------------------------------------------
    // sysmem_lite is the HPS f2sdram bridge (cyclonev_hps_interface_* hard IP).
    // It gives us the core clock + reset and three Avalon RAM ports; we use
    // ram1. DDR3 therefore lives on the HPS - no top-level DDRAM pins.
    wire         clk_100m;
    wire         reset_100m;   // active-high, from bridge

    assign core_clk   = clk_100m;
    assign core_reset = reset_100m;

    // ddram <-> bridge (ram1) Avalon wiring, all internal (no pins)
    wire        ram_clk;
    wire        ram_busy;         // ram1_waitrequest
    wire  [7:0] ram_burstcnt;
    wire [28:0] ram_addr;
    wire [63:0] ram_readdata;
    wire        ram_readdatavalid;
    wire        ram_rd;
    wire [63:0] ram_writedata;
    wire  [7:0] ram_byteenable;
    wire        ram_we;

    sysmem_lite u_sysmem (
        .reset_core_req    (reset_req),
        .reset_out         (reset_100m),
        .clock             (clk_100m),
        .reset_hps_cold_req(cold_req),
        .reset_hps_warm_req(1'b0),

        // ram1 : driven by ddram controller below
        .ram1_clk          (ram_clk),
        .ram1_address      (ram_addr),
        .ram1_burstcount   (ram_burstcnt),
        .ram1_waitrequest  (ram_busy),
        .ram1_readdata     (ram_readdata),
        .ram1_readdatavalid(ram_readdatavalid),
        .ram1_read         (ram_rd),
        .ram1_writedata    (ram_writedata),
        .ram1_byteenable   (ram_byteenable),
        .ram1_write        (ram_we),

        // ram2 : unused
        .ram2_clk          (clk_100m),
        .ram2_address      (29'd0),
        .ram2_burstcount   (8'd0),
        .ram2_waitrequest  (),
        .ram2_readdata     (),
        .ram2_readdatavalid(),
        .ram2_read         (1'b0),
        .ram2_writedata    (64'd0),
        .ram2_byteenable   (8'd0),
        .ram2_write        (1'b0),

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

    // ---- DDR3 controller instance ----
    // simple side of the DC_MiSTer ddram wrapper:
    //   mem_addr[27:1] word address, mem_din/mem_dout 32-bit, mem_wr 4-bit BE.
    reg  [27:1] mem_addr;
    reg  [31:0] mem_din;
    reg         mem_rd;
    reg  [3:0]  mem_wr;
    reg  [1:0]  mem_chan;
    wire [31:0] mem_dout;
    wire        mem_busy;

    ddram u_ddram (
        .DDRAM_CLK       (ram_clk),
        .DDRAM_BUSY      (ram_busy),
        .DDRAM_BURSTCNT  (ram_burstcnt),
        .DDRAM_ADDR      (ram_addr),
        .DDRAM_DOUT      (ram_readdata),
        .DDRAM_DOUT_READY(ram_readdatavalid),
        .DDRAM_RD        (ram_rd),
        .DDRAM_DIN       (ram_writedata),
        .DDRAM_BE        (ram_byteenable),
        .DDRAM_WE        (ram_we),

        .clk     (clk_100m),
        .mem_addr(mem_addr),
        .mem_dout(mem_dout),
        .mem_din (mem_din),
        .mem_rd  (mem_rd),
        .mem_wr  (mem_wr),
        .mem_chan(mem_chan),
        .mem_16b (1'b0),
        .mem_busy(mem_busy)
    );

    // ------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------
    localparam [3:0]
        S_IDLE      = 4'd0,
        S_CLEAR     = 4'd1,
        S_DRAWTAGS  = 4'd2,
        S_FLUSH_RD  = 4'd3,  // present pixel, issue write
        S_FLUSH_WAIT= 4'd4,  // wait for ddram to accept (not busy)
        S_ISP_SETUP = 4'd6,  // wait for isp_setup_min to finish
        S_ISP_RASTER= 4'd7,  // rasterize one line per clock (opaque)
        S_DONE      = 4'd5;

    reg [3:0]       state;
    reg [IDX_W-1:0] idx;         // pixel iterator (0..N_PIX-1)
    integer         bi;          // rasterizer per-bank loop var
    assign idx_x = idx[XW-1:0];
    assign idx_y = idx[IDX_W-1:XW];

    // pixel color read for flush (registered one cycle for block-ram read)
    reg [31:0]      flush_col;

    // 32-bit word address for the current flush pixel: REG_TILE_BASE + idx.
    wire [31:0] word32_addr = regs[REG_TILE_BASE] + {{(32-IDX_W){1'b0}}, idx};

    assign ready = (state == S_IDLE);

    // ------------------------------------------------------------------
    // ISP triangle setup (CMD_TRIANGLE_ISP_SETUP)
    // ------------------------------------------------------------------
    // 4-lane, ~14-cycle, minimal-DSP setup. invW plane only. Tile-local:
    // C1..C4 / c_invw anchored at (REG_X_BASE, REG_Y_BASE). Reduced precision
    // (16-bit-mantissa mults, ~24-bit adds, cheap 1/x). Outputs are latched into
    // the isp_* result registers below for the rasterizer to consume.
    reg         isp_start;
    wire        isp_done;
    wire        isp_sgn_neg, isp_cull;
    wire [31:0] w_dx12,w_dx23,w_dx31,w_dx41, w_dy12,w_dy23,w_dy31,w_dy41;
    wire [31:0] w_c1,w_c2,w_c3,w_c4, w_ddx,w_ddy,w_cinvw;

    isp_setup_min u_isp (
        .clk(clk_100m), .reset(reset_100m), .start(isp_start), .done(isp_done),
        .isp_word(isp_word),
        .x1(isp_x1), .y1(isp_y1), .z1(isp_z1),
        .x2(isp_x2), .y2(isp_y2), .z2(isp_z2),
        .x3(isp_x3), .y3(isp_y3), .z3(isp_z3),
        .xbase(isp_xbase), .ybase(isp_ybase),
        .sgn_neg(isp_sgn_neg), .cull(isp_cull),
        .dx12(w_dx12), .dx23(w_dx23), .dx31(w_dx31), .dx41(w_dx41),
        .dy12(w_dy12), .dy23(w_dy23), .dy31(w_dy31), .dy41(w_dy41),
        .c1(w_c1), .c2(w_c2), .c3(w_c3), .c4(w_c4),
        .ddx_invw(w_ddx), .ddy_invw(w_ddy), .c_invw(w_cinvw)
    );

    // Latched setup results (the inner-loop rasterizer consumes these).
    reg        isp_r_sgn_neg, isp_r_cull;
    reg [31:0] isp_dx12,isp_dx23,isp_dx31,isp_dx41;
    reg [31:0] isp_dy12,isp_dy23,isp_dy31,isp_dy41;
    reg [31:0] isp_c1,isp_c2,isp_c3,isp_c4;
    reg [31:0] isp_ddx_invw, isp_ddy_invw, isp_c_invw;

    // ------------------------------------------------------------------
    // ISP rasterize (CMD_TRIANGLE_ISP_RASTERIZE) - opaque only.
    // ------------------------------------------------------------------
    // Processes one 32-pixel scanline per clock. isp_raster_line evaluates all
    // 32 Xhs/invW combinationally for the current line; per inside-pixel we run
    // the opaque DepthMode test and, on pass, write invW depth + tag into the
    // 32 depth/tag banks (all in the same clock).
    reg  [4:0]  ras_y;                 // current line
    wire [31:0] ras_inside;
    wire [31:0] ras_invw [0:TILE_W-1];

    isp_raster_line u_line (
        .y(ras_y),
        .c1(isp_c1), .c2(isp_c2), .c3(isp_c3), .c4(isp_c4),
        .dx12(isp_dx12),.dx23(isp_dx23),.dx31(isp_dx31),.dx41(isp_dx41),
        .dy12(isp_dy12),.dy23(isp_dy23),.dy31(isp_dy31),.dy41(isp_dy41),
        .ddx(isp_ddx_invw),.ddy(isp_ddy_invw),.c_invw(isp_c_invw),
        .inside_mask(ras_inside),
        .invw_0(ras_invw[0]),  .invw_1(ras_invw[1]),  .invw_2(ras_invw[2]),  .invw_3(ras_invw[3]),
        .invw_4(ras_invw[4]),  .invw_5(ras_invw[5]),  .invw_6(ras_invw[6]),  .invw_7(ras_invw[7]),
        .invw_8(ras_invw[8]),  .invw_9(ras_invw[9]),  .invw_10(ras_invw[10]),.invw_11(ras_invw[11]),
        .invw_12(ras_invw[12]),.invw_13(ras_invw[13]),.invw_14(ras_invw[14]),.invw_15(ras_invw[15]),
        .invw_16(ras_invw[16]),.invw_17(ras_invw[17]),.invw_18(ras_invw[18]),.invw_19(ras_invw[19]),
        .invw_20(ras_invw[20]),.invw_21(ras_invw[21]),.invw_22(ras_invw[22]),.invw_23(ras_invw[23]),
        .invw_24(ras_invw[24]),.invw_25(ras_invw[25]),.invw_26(ras_invw[26]),.invw_27(ras_invw[27]),
        .invw_28(ras_invw[28]),.invw_29(ras_invw[29]),.invw_30(ras_invw[30]),.invw_31(ras_invw[31])
    );

    // opaque DepthMode compare: does new invW pass against stored depth?
    //  refsw: 0 never,1 less(reject if new>=old),2 equal,3 <=,4 greater,
    //         5 !=,6 >=,7 always.  ("reject" cases inverted to a pass flag)
    function fcmp_pass(input [2:0] mode, input [31:0] nw, input [31:0] ob);
        reg lt,eq,gt;
        begin
            eq = (nw[30:0]==31'd0 && ob[30:0]==31'd0) ? 1'b1 : (nw==ob);
            gt = fgt(nw,ob);
            lt = ~eq & ~gt;
            case (mode)
                3'd0: fcmp_pass = 1'b0;      // never
                3'd1: fcmp_pass = lt;        // less  (new < old)
                3'd2: fcmp_pass = eq;        // equal
                3'd3: fcmp_pass = lt|eq;     // less-or-equal
                3'd4: fcmp_pass = gt;        // greater
                3'd5: fcmp_pass = ~eq;       // not-equal
                3'd6: fcmp_pass = gt|eq;     // greater-or-equal
                3'd7: fcmp_pass = 1'b1;      // always
            endcase
        end
    endfunction
    // signed-float greater-than a > b (no NaN/inf; DaZ handled by ==0 test)
    function fgt(input [31:0] a, input [31:0] b);
        reg az,bz; reg [30:0] am,bm;
        begin
            az=(a[30:0]==0); bz=(b[30:0]==0);
            am=a[30:0]; bm=b[30:0];
            if (az&&bz)          fgt=1'b0;
            else if (a[31]^b[31]) fgt = b[31];         // a>b if b negative
            else if (~a[31])      fgt = (am>bm);        // both >=0
            else                  fgt = (am<bm);        // both <0
        end
    endfunction

    wire [2:0] depth_mode = isp_word[29:27];
    wire       zwrite_dis = isp_word[24];

    always @(posedge clk_100m) begin
        if (reset_100m) begin
            state    <= S_IDLE;
            idx      <= 0;
            cmd_done <= 1'b0;
            mem_rd   <= 1'b0;
            mem_wr   <= 4'b0;
            mem_chan <= 2'b0;
            isp_start<= 1'b0;
        end else begin
            cmd_done <= 1'b0;    // default: single-cycle pulse
            mem_rd   <= 1'b0;
            mem_wr   <= 4'b0;
            isp_start<= 1'b0;    // default: single-cycle start pulse

            case (state)
            // --------------------------------------------------------
            S_IDLE: begin
                if (cmd_valid) begin
                    if (cmd[6] == 1'b0) begin
                        // 0..63 : register write. Completes immediately;
                        // stay ready and pulse done next cycle.
                        regs[cmd[5:0]] <= data;
                        cmd_done <= 1'b1;   // ready again next cycle (still idle)
                    end else begin
                        idx <= 0;
                        case (cmd)
                            CMD_TILE_CLEAR:     state <= S_CLEAR;
                            CMD_TILE_DRAW_TAGS: state <= S_DRAWTAGS;
                            CMD_TILE_FLUSH:     state <= S_FLUSH_RD;
                            CMD_TRIANGLE_ISP_SETUP: begin
                                isp_start <= 1'b1;      // kick the setup pipeline
                                state     <= S_ISP_SETUP;
                            end
                            CMD_TRIANGLE_ISP_RASTERIZE: begin
                                ras_y <= 5'd0;          // start at line 0
                                state <= S_ISP_RASTER;
                            end
                            // nop commands: acknowledge, one cycle before ready
                            default: begin
                                cmd_done <= 1'b1;
                                // stays in S_IDLE -> ready next cycle
                            end
                        endcase
                    end
                end
            end

            // --------------------------------------------------------
            // TILE_CLEAR: every entry <= {REG_BG_DEPTH, REG_BG_TAG}
            S_CLEAR: begin
                dt_buf[idx_x][idx_y] <= {regs[REG_BG_DEPTH], regs[REG_BG_TAG][TAG_BITS-1:0]};
                if (idx == LAST_IDX) state <= S_DONE;
                else                idx   <= idx + 1'b1;
            end

            // --------------------------------------------------------
            // TILE_DRAW_TAGS: color <= {8'hFF, tag}
            S_DRAWTAGS: begin
                col_buf[idx_x][idx_y] <= {8'hFF, dt_buf[idx_x][idx_y][TAG_BITS-1:0]};
                if (idx == LAST_IDX) state <= S_DONE;
                else                idx   <= idx + 1'b1;
            end

            // --------------------------------------------------------
            // TILE_FLUSH: color[y][x] -> DDR3[REG_TILE_BASE + y*32 + x]
            //  idx already encodes y*TILE_W + x (row-major).
            S_FLUSH_RD: begin
                flush_col <= col_buf[idx_x][idx_y];
                state     <= S_FLUSH_WAIT;
            end

            S_FLUSH_WAIT: begin
                if (!mem_busy) begin
                    // REG_TILE_BASE and idx are 32-bit *word* indices; the
                    // ddram bus mem_addr[27:1] indexes 16-bit words, so a
                    // 32-bit word maps to (word_index << 1) with bit 0 = 0.
                    mem_addr <= word32_addr[26:0] << 1;
                    mem_din  <= flush_col;
                    mem_wr   <= 4'b1111;      // full 32-bit write
                    if (idx == LAST_IDX) state <= S_DONE;
                    else begin
                        idx   <= idx + 1'b1;
                        state <= S_FLUSH_RD;
                    end
                end
            end

            // --------------------------------------------------------
            // TRIANGLE_ISP_SETUP: wait for the setup pipeline, latch results.
            S_ISP_SETUP: begin
                if (isp_done) begin
                    isp_r_sgn_neg <= isp_sgn_neg; isp_r_cull <= isp_cull;
                    isp_dx12<=w_dx12; isp_dx23<=w_dx23; isp_dx31<=w_dx31; isp_dx41<=w_dx41;
                    isp_dy12<=w_dy12; isp_dy23<=w_dy23; isp_dy31<=w_dy31; isp_dy41<=w_dy41;
                    isp_c1<=w_c1; isp_c2<=w_c2; isp_c3<=w_c3; isp_c4<=w_c4;
                    isp_ddx_invw<=w_ddx; isp_ddy_invw<=w_ddy; isp_c_invw<=w_cinvw;
                    state <= S_DONE;
                end
            end

            // --------------------------------------------------------
            // TRIANGLE_ISP_RASTERIZE (opaque): one scanline per clock.
            // Cull skips the whole triangle. For each inside pixel that passes
            // the DepthMode test: write invW depth (unless ZWriteDis) and tag.
            S_ISP_RASTER: begin
                if (isp_r_cull) begin
                    state <= S_DONE;
                end else begin
                    for (bi = 0; bi < TILE_W; bi = bi + 1) begin
                        if (ras_inside[bi] &&
                            fcmp_pass(depth_mode, ras_invw[bi],
                                      dt_buf[bi][ras_y][32+TAG_BITS-1:TAG_BITS])) begin
                            // depth (top 32b) written unless ZWriteDis; tag always
                            dt_buf[bi][ras_y] <= {
                                zwrite_dis ? dt_buf[bi][ras_y][32+TAG_BITS-1:TAG_BITS]
                                           : ras_invw[bi],
                                regs[REG_ISP_TAG][TAG_BITS-1:0] };
                        end
                    end
                    if (ras_y == YW'(TILE_H-1)) state <= S_DONE;
                    else                        ras_y <= ras_y + 1'b1;
                end
            end

            // --------------------------------------------------------
            S_DONE: begin
                cmd_done <= 1'b1;   // pulse: ready next cycle
                state    <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
