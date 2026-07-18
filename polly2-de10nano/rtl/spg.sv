// spg.sv - fixed-mode display controller, loosely modelled on the Dreamcast
// (HOLLY) spg, but generating the FINAL 1080p raster directly at the HDMI
// pixel clock. Replaces ascal on the HDMI path:
//
//  - 1920x1080 CEA-861 timing (2200 x 1125 totals, both syncs positive).
//    At a 148.352 MHz pixel clock this is 1080p59.94 (the DC VGA field
//    rate); at 148.5 MHz it is exactly 60.000 Hz.
//  - The 640x480 RGB565 framebuffer in DDR3 is displayed pixel- and
//    line-doubled (2x nearest) as a 1280x960 window centred in the frame
//    (x 320..1599, y 60..1019 - the same window the stripped ascal used).
//    fb_line_dbl displays a 640x240 source at 4x vertically instead (the
//    240p menu option).
//  - fb_split replicates the Dreamcast 32-bit-view VRAM layout (same rule
//    as ascal's fb_split_word): FB 32-bit word W lives at DDR byte W*8, in
//    the low (fb_disp_half=1) or high (fb_disp_half=0) 32-bit half of each
//    64-bit word. A line is then 2560 DDR bytes with half the bits useful
//    (2x overfetch - still trivial bandwidth). fb_stride stays in FB-view
//    bytes (1280) in both modes; the DDR advance doubles internally.
//  - Two optional border bands: 640x30 RGB565 LINEAR framebuffers (stride
//    fixed at 1280 bytes) displayed 2x-doubled as 1280x60, exactly filling
//    the top (lines 0..59) and bottom (lines 1020..1079) borders,
//    x-centred like the game window. fb_top_base / fb_bot_base are DDR
//    BYTE addresses, 128-byte aligned, sampled once per frame at the start
//    of vertical blanking (line 1080); 0 disables a band (black border).
//    Band lines reuse the game fetch/line-buffer engine (linear mode,
//    beat-aligned: 80 beats per line, never split, never misaligned).
//  - DDR access is a 128-bit Avalon read master, intended for the vbuf
//    port on sysmem_lite that ascal used to own (16-byte word addresses).
//    The FB base only has to be 8-byte aligned: an odd 64-bit start word
//    (fb_base[3]) is handled by fetching one extra beat and shifting the
//    write-side word mapping.
//  - Two line buffers ("current" / "next"): while one source line is
//    displayed (two output lines), the next is burst-read into the other.
//    Prefetch slack is two output lines (~29.7 us) per line, so no DMA
//    engine and no tight arbiter deadline - just a port that eventually
//    serves 5-11 bursts of 16 beats. Bursts are serialized: a new command
//    is issued only after every beat of the previous one has arrived
//    (deliberately conservative w.r.t. the real-DDR3 read-beat desync
//    behaviour seen on hardware). A request landing mid-fetch (severe
//    starvation) is queued, not lost.
//  - Storage is four 32-bit banks: FB 32-bit word N -> bank N[1:0],
//    address {buffer, N[8:2]} - the same mapping in both fetch modes, so
//    the read side is mode-agnostic.
//
// All video outputs are registered and mutually aligned (2 clk latency
// from the internal counters). Border pixels are black. Pixel 0 of each
// 32-bit FB word is bits [15:0] (little-endian), matching the peel_core
// 16bpp tile writeback; expansion to 8:8:8 is MSB-replicated.

module spg
#(
	// CEA-861 1080p timing
	parameter H_ACTIVE = 1920,
	parameter H_FP     = 88,
	parameter H_SYNC   = 44,
	parameter H_BP     = 148,   // H total 2200
	parameter V_ACTIVE = 1080,
	parameter V_FP     = 4,
	parameter V_SYNC   = 5,
	parameter V_BP     = 36,    // V total 1125
	// source framebuffer (doubled to SRC_W*2 x SRC_H*2 on screen)
	parameter SRC_W    = 640,
	parameter SRC_H    = 480
)
(
	input  wire        clk,          // 1080p pixel clock (148.352 / 148.5 MHz)
	input  wire        reset,        // video-domain reset

	// Framebuffer location. Sampled once per frame at the top of the raster;
	// byte address must be 8-byte aligned, stride a multiple of 16 bytes
	// (low bits of both are ignored).
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [31:0] fb_base,      // BYTE address of the top-left pixel
	input  wire [13:0] fb_stride,    // FB-view BYTEs per source line (1280 for 640 x 16bpp)
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        fb_line_dbl,  // 240p source: 4x vertical instead of 2x
	input  wire        fb_split,     // Dreamcast split-VRAM layout (see header)
	input  wire        fb_disp_half, // split: which 32-bit half of each 64-bit word

	// Border bands (see header). BYTE addresses, 128-byte aligned, 0 = off;
	// sampled once per frame at the start of vertical blanking.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [31:0] fb_top_base,  // 640x30 linear RGB565 above the window
	input  wire [31:0] fb_bot_base,  // 640x30 linear RGB565 below the window
	/* verilator lint_on UNUSEDSIGNAL */

	// 128-bit Avalon read master (sysmem vbuf port), avl_clk domain
	input  wire         avl_clk,
	output reg          avl_read,
	output reg  [27:0]  avl_address,    // 16-byte word address
	output reg  [7:0]   avl_burstcount,
	input  wire         avl_waitrequest,
	input  wire [127:0] avl_readdata,
	input  wire         avl_readdatavalid,

	// Video out (registered, aligned; syncs active high)
	output reg  [7:0]  red,
	output reg  [7:0]  green,
	output reg  [7:0]  blue,
	output reg         hsync,
	output reg         vsync,
	output reg         de,           // full 1920x1080 active area
	output reg         vblank,       // output-raster vertical blank (ascal o_vbl)
	output reg         border,       // active raster but outside the image window (ascal o_brd)

	// Raster status / frame pacing (video domain)
	output reg  [9:0]  src_line,     // source line currently displayed
	output reg         vblank_in,    // 1-clk pulse: image window finished
	output reg         vblank_out,   // 1-clk pulse: image window starts
	output reg         underrun      // sticky: a line was displayed before its fetch finished
);

// NOTE: no SystemVerilog size casts anywhere in this file - Quartus
// Standard 17.0 does not support N'(expr).
/* verilator lint_off WIDTHTRUNC */
localparam [11:0] H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;
localparam [10:0] V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;
localparam [11:0] HS_BEG   = H_ACTIVE + H_FP;
localparam [11:0] HS_END   = H_ACTIVE + H_FP + H_SYNC;
localparam [10:0] VS_BEG   = V_ACTIVE + V_FP;
localparam [10:0] VS_END   = V_ACTIVE + V_FP + V_SYNC;
localparam [11:0] H_ACT    = H_ACTIVE;
localparam [10:0] V_ACT    = V_ACTIVE;
localparam [11:0] X0       = (H_ACTIVE - SRC_W*2)/2;        // 320
localparam [11:0] X1       = (H_ACTIVE - SRC_W*2)/2 + SRC_W*2; // 1600
localparam [10:0] Y0       = (V_ACTIVE - SRC_H*2)/2;        // 60
localparam [10:0] Y1       = (V_ACTIVE - SRC_H*2)/2 + SRC_H*2; // 1020
localparam [10:0] FB_WORDS = SRC_W/2;    // 320 x 32-bit FB words per source line

localparam        BURST_LEN = 16;        // 128-bit beats per burst (256 bytes)
localparam [7:0]  BC_FULL   = BURST_LEN; // full burstcount
localparam [4:0]  BB_FULL   = BURST_LEN; // full per-burst beat count
localparam [8:0]  B9_FULL   = BURST_LEN;
localparam [27:0] ADDR_STEP = BURST_LEN; // address advance per full burst

// border bands: 640x30 linear, stride 1280 bytes = 80 16-byte words/beats
localparam [10:0] BAND_ADV   = 11'd80;
localparam [8:0]  BAND_BEATS = 9'd80;

// fetch/display region encoding ({region, src line} keys the request dedup)
localparam [1:0] RGN_GAME = 2'd0;
localparam [1:0] RGN_TOP  = 2'd1;
localparam [1:0] RGN_BOT  = 2'd2;
/* verilator lint_on WIDTHTRUNC */

//////////////////////////////////////////////////////////////////////////
// Output raster counters
//////////////////////////////////////////////////////////////////////////

reg [11:0] hcnt = 12'd0;
reg [10:0] vcnt = 11'd0;

always @(posedge clk or posedge reset) begin
	if (reset) begin
		hcnt <= 12'd0;
		vcnt <= 11'd0;
	end
	else begin
		if (hcnt == H_TOTAL - 12'd1) begin
			hcnt <= 12'd0;
			vcnt <= (vcnt == V_TOTAL - 11'd1) ? 11'd0 : vcnt + 11'd1;
		end
		else hcnt <= hcnt + 12'd1;
	end
end

wire img_v = (vcnt >= Y0) && (vcnt < Y1);

// Per-frame latch of the framebuffer location (quasi-static for the Avalon
// domain: the first fetch request is issued Y0-2 lines later).
reg [27:0] base_lat   = 28'd0;   // 16-byte word address of the FB start
reg        odd0_lat   = 1'b0;    // FB starts in the odd 64-bit half of its beat
reg [10:0] adv_lat    = 11'd0;   // 16-byte words to advance per source line
reg        dbl_lat    = 1'b0;
reg        split_lat  = 1'b0;
reg        half_lat   = 1'b0;

// Band latches: sampled at the start of vertical blanking (line V_ACT), so
// they are stable before the top band's first request (2 lines before the
// raster wraps) and its display at line 0.
reg [27:0] top_base_lat = 28'd0;
reg [27:0] bot_base_lat = 28'd0;
reg        top_en_lat   = 1'b0;
reg        bot_en_lat   = 1'b0;

// line-doubling shift: output lines per source line = 2 (480) or 4 (240p)
wire [1:0] vshift = dbl_lat ? 2'd2 : 2'd1;

//////////////////////////////////////////////////////////////////////////
// Fetch requests (video domain -> Avalon domain)
//////////////////////////////////////////////////////////////////////////

// Two output lines ahead of the raster, decide which SOURCE line (of which
// region) will be needed and request it once. Buffer parity = source line
// LSB, so the line being written is never the line being displayed - this
// holds across region seams too (each region restarts at src 0 an even
// number of output lines after the previous region's last line began).
reg        req_toggle = 1'b0;
reg        req_sof    = 1'b0;   // first line of its region: reload base address
reg        req_buf    = 1'b0;
reg  [1:0] req_region = RGN_GAME;
reg [11:0] last_req   = 12'hFFF;   // {region, src}
reg  [1:0] cnt_req    = 2'd0;

// wraps only for the top band, whose first request lands 2 lines before
// the raster does (V_TOTAL-2 -> line 0)
wire [10:0] y_look_raw = vcnt + 11'd2;
wire [10:0] y_look     = (y_look_raw >= V_TOTAL) ? y_look_raw - V_TOTAL
                                                 : y_look_raw;

wire        look_top  = (y_look < Y0) && top_en_lat;
wire        look_game = (y_look >= Y0) && (y_look < Y1);
wire        look_bot  = (y_look >= Y1) && (y_look < V_ACT) && bot_en_lat;
wire        look_in   = look_top || look_game || look_bot;
wire  [1:0] look_rgn  = look_top ? RGN_TOP : look_game ? RGN_GAME : RGN_BOT;
wire [10:0] look_rel  = look_top  ? y_look
                      : look_game ? y_look - Y0
                                  : y_look - Y1;
wire  [1:0] look_vsh  = look_game ? vshift : 2'd1;   // bands are always 2x
/* verilator lint_off UNUSEDSIGNAL */
wire [10:0] look_shf = look_rel >> look_vsh;
/* verilator lint_on UNUSEDSIGNAL */
wire  [9:0] look_src = look_shf[9:0];
wire [11:0] look_key = {look_rgn, look_src};

always @(posedge clk or posedge reset) begin
	if (reset) begin
		req_toggle <= 1'b0;
		last_req   <= 12'hFFF;
		cnt_req    <= 2'd0;
	end
	else begin
		if (vcnt == 11'd0 && hcnt == 12'd0) begin
			base_lat   <= fb_base[31:4];
			odd0_lat   <= fb_base[3];
			// DDR bytes per line double in split mode (8 bytes per FB word)
			adv_lat    <= fb_split ? fb_stride[13:3] : {1'b0, fb_stride[13:4]};
			dbl_lat    <= fb_line_dbl;
			split_lat  <= fb_split;
			half_lat   <= fb_disp_half;
		end

		if (vcnt == V_ACT && hcnt == 12'd0) begin
			top_base_lat <= fb_top_base[31:4];   // [6:4] zero: 128B aligned
			bot_base_lat <= fb_bot_base[31:4];
			top_en_lat   <= (fb_top_base[31:7] != 25'd0);
			bot_en_lat   <= (fb_bot_base[31:7] != 25'd0);
			last_req     <= 12'hFFF;
		end

		if (hcnt == 12'd0 && look_in && look_key != last_req) begin
			req_sof    <= (look_src == 10'd0);
			req_buf    <= look_src[0];
			req_region <= look_rgn;
			last_req   <= look_key;
			cnt_req    <= cnt_req + 2'd1;
			req_toggle <= ~req_toggle;   // payload above is stable when this lands
		end
	end
end

//////////////////////////////////////////////////////////////////////////
// Avalon fetch FSM (avl_clk domain)
//////////////////////////////////////////////////////////////////////////

reg  [2:0] rt_sync     = 3'd0;
reg  [1:0] rst_sync    = 2'd0;
reg        fetching    = 1'b0;
reg        pending     = 1'b0;   // request arrived while still fetching
reg [27:0] line_addr   = 28'd0;  // 16-byte word address of the current line
reg  [8:0] w           = 9'd0;   // beat index within the line
reg  [8:0] beats_left  = 9'd0;   // beats still expected for the line
reg  [4:0] burst_beats = 5'd0;   // beats still expected for the current burst
reg        done_toggle = 1'b0;
reg        cur_split   = 1'b0;   // this fetch's layout: only game can be split
reg        cur_odd0    = 1'b0;   // ... or start mid-beat (bands are 128B aligned)

wire req_edge = rt_sync[2] ^ rt_sync[1];
wire req_game = (req_region == RGN_GAME);

// 80 beats linear / 160 split, +1 when the FB starts mid-beat
wire [8:0] total_beats = (split_lat ? 9'd160 : 9'd80) + {8'd0, odd0_lat};

// requests never interleave regions: line_addr advances within one region
// and req_sof reloads it from that region's base at the region's first line
wire [27:0] region_base = (req_region == RGN_TOP) ? top_base_lat
                        : (req_region == RGN_BOT) ? bot_base_lat
                                                  : base_lat;

always @(posedge avl_clk) begin : fetch_fsm
	reg [27:0] na;
	reg  [8:0] rem;

	// verilator lint_off SYNCASYNCNET
	rst_sync <= {rst_sync[0], reset};
	// verilator lint_on SYNCASYNCNET
	rt_sync  <= {rt_sync[1:0], req_toggle};

	if (avl_read && !avl_waitrequest) avl_read <= 1'b0;   // command accepted

	if (req_edge && fetching) pending <= 1'b1;

	if ((req_edge || pending) && !fetching) begin
		pending        <= 1'b0;
		na = req_sof ? region_base
		             : line_addr + {17'd0, req_game ? adv_lat : BAND_ADV};
		line_addr      <= na;
		avl_address    <= na;
		avl_burstcount <= BC_FULL;         // total_beats >= 80: first burst is full
		avl_read       <= 1'b1;
		burst_beats    <= BB_FULL;
		beats_left     <= req_game ? total_beats : BAND_BEATS;
		cur_split      <= req_game & split_lat;
		cur_odd0       <= req_game & odd0_lat;
		w              <= 9'd0;
		fetching       <= 1'b1;
	end
	else if (fetching && avl_readdatavalid) begin
		w           <= w + 9'd1;
		beats_left  <= beats_left - 9'd1;
		burst_beats <= burst_beats - 5'd1;
		if (burst_beats == 5'd1) begin                    // last beat of this burst
			rem = beats_left - 9'd1;
			if (rem == 9'd0) begin
				fetching    <= 1'b0;
				done_toggle <= ~done_toggle;
			end
			else begin
				// every burst except a shorter final one is BURST_LEN long
				avl_address    <= avl_address + ADDR_STEP;
				avl_burstcount <= (rem >= B9_FULL) ? BC_FULL : rem[7:0];
				burst_beats    <= (rem >= B9_FULL) ? BB_FULL : rem[4:0];
				avl_read       <= 1'b1;
			end
		end
	end

	if (rst_sync[1]) begin
		fetching <= 1'b0;
		avl_read <= 1'b0;
		pending  <= 1'b0;
	end
end

//////////////////////////////////////////////////////////////////////////
// Line buffers: four 32-bit banks of 256 (2 buffers x 80), dual clock.
// Global FB 32-bit word N -> bank N[1:0], address {buffer, N[8:2]}.
//
// Write side, beat w (128 bits): the candidate FB words in this beat are
//   linear: n = 4w + j - 2*odd0, j = 0..3, data = beat[32j +: 32]
//   split : n = 2w + h -   odd0, h = 0..1, data = the selected 32-bit half
//           of beat[64h +: 64]
// For each bank at most one candidate matches (n[1:0] == bank); out-of-
// range candidates (misaligned-start head, line tail) are dropped. n is
// computed in wrapping unsigned arithmetic: the head's negative indices
// wrap high, so one upper-bound compare covers head and tail.
//////////////////////////////////////////////////////////////////////////

wire [10:0] base_n = cur_split ? {1'b0, w, 1'b0} - {10'd0, cur_odd0}
                               : {w, 2'b00}      - {9'd0, cur_odd0, 1'b0};

wire  [9:0] spx;    // display-side source pixel (declared ahead of use)
wire        rbuf;
wire [31:0] rq [0:3];

generate
genvar gb;
for (gb = 0; gb < 4; gb = gb + 1) begin : bank
	reg [31:0] mem [0:255];
	reg [31:0] q = 32'd0;

	// candidate offset for this bank: (gb - base_n) mod 4
	/* verilator lint_off WIDTHTRUNC */
	localparam [1:0] GB = gb;
	/* verilator lint_on WIDTHTRUNC */
	wire  [1:0] o2   = GB - base_n[1:0];
	wire        cand = cur_split ? (o2 < 2'd2) : 1'b1;
	wire [10:0] n    = base_n + {9'd0, o2};
	wire        ok   = cand && (n < FB_WORDS);

	wire [63:0] h64  = o2[0] ? avl_readdata[127:64] : avl_readdata[63:0];
	wire [31:0] w32  = o2[1] ? (o2[0] ? avl_readdata[127:96] : avl_readdata[95:64])
	                         : (o2[0] ? avl_readdata[63:32]  : avl_readdata[31:0]);
	wire [31:0] wd   = cur_split ? (half_lat ? h64[31:0] : h64[63:32]) : w32;

	always @(posedge avl_clk) begin
		if (fetching && avl_readdatavalid && ok) mem[{req_buf, n[8:2]}] <= wd;
	end

	always @(posedge clk) q <= mem[{rbuf, spx[9:3]}];
	assign rq[gb] = q;
end
endgenerate

//////////////////////////////////////////////////////////////////////////
// Display read path (2-stage: RAM read, lane select). RGB emerges 2 clocks
// after the counters; syncs/de/border are piped by 2 to stay aligned.
//////////////////////////////////////////////////////////////////////////

// active display region this line: game window or one of the border bands
wire        top_v = (vcnt < Y0) && top_en_lat;
wire        bot_v = (vcnt >= Y1) && (vcnt < V_ACT) && bot_en_lat;

wire [10:0] y_rel   = vcnt - Y0;             // game-relative (underrun check)
wire [10:0] d_rel   = top_v ? vcnt : bot_v ? vcnt - Y1 : y_rel;
wire  [1:0] d_vsh   = (top_v || bot_v) ? 2'd1 : vshift;
/* verilator lint_off UNUSEDSIGNAL */
wire [10:0] y_shf   = d_rel >> d_vsh;
wire [11:0] x_rel   = hcnt - X0;
/* verilator lint_on UNUSEDSIGNAL */
wire  [9:0] src_cur = y_shf[9:0];
assign      rbuf    = src_cur[0];

wire        img_h   = (hcnt >= X0) && (hcnt < X1);
assign      spx     = x_rel[10:1];           // /2: source pixel 0..639

wire de_c  = (hcnt < H_ACT) && (vcnt < V_ACT);
wire hs_c  = (hcnt >= HS_BEG) && (hcnt < HS_END);
// VS edges must coincide with the HS leading edge (CEA-861; same equation
// as ascal's o_vsv): rise at HS_BEG of line VS_BEG, fall at HS_BEG of line
// VS_END. Toggling at hcnt==0 instead puts the VS edge 2008 px before HS,
// which some HDMI sinks reject as an unsupported mode.
wire vs_c  = (vcnt == VS_BEG && hcnt >= HS_BEG) ||
             (vcnt >  VS_BEG && vcnt <  VS_END) ||
             (vcnt == VS_END && hcnt <  HS_BEG);
wire vbl_c = (vcnt >= V_ACT);
wire img_c = img_h && (img_v || top_v || bot_v);

reg [2:0] lane1  = 3'd0;   // [2:1]=bank, [0]=16-bit half
reg [4:0] pipe1  = 5'd0;   // {img, de, hs, vs, vbl}

always @(posedge clk) begin
	// stage 1 (the bank RAM reads are inside the generate above)
	lane1 <= spx[2:0];
	pipe1 <= {img_c, de_c, hs_c, vs_c, vbl_c};

	// stage 2
	begin : lane_mux
		reg [31:0] p32;
		reg [15:0] p;
		p32 = rq[lane1[2:1]];
		p   = lane1[0] ? p32[31:16] : p32[15:0];   // little-endian 16-bit lanes
		if (pipe1[4]) begin                  // inside the image window
			red   <= {p[15:11], p[15:13]};   // RGB565 -> 888, MSB replicate
			green <= {p[10:5],  p[10:9]};
			blue  <= {p[4:0],   p[4:2]};
		end
		else begin
			red   <= 8'd0;
			green <= 8'd0;
			blue  <= 8'd0;
		end
	end
	de     <= pipe1[3];
	hsync  <= pipe1[2];
	vsync  <= pipe1[1];
	vblank <= pipe1[0];
	border <= pipe1[3] & ~pipe1[4];
end

//////////////////////////////////////////////////////////////////////////
// Raster status, frame pacing pulses, underrun detect
//////////////////////////////////////////////////////////////////////////

reg [2:0] dt_sync  = 3'd0;
reg [1:0] cnt_done = 2'd0;

always @(posedge clk or posedge reset) begin
	if (reset) begin
		dt_sync    <= 3'd0;
		cnt_done   <= 2'd0;
		underrun   <= 1'b0;
		vblank_in  <= 1'b0;
		vblank_out <= 1'b0;
		src_line   <= 10'd0;
	end
	else begin
		dt_sync <= {dt_sync[1:0], done_toggle};
		if (dt_sync[2] ^ dt_sync[1]) cnt_done <= cnt_done + 2'd1;

		src_line <= img_v ? src_cur : 10'd0;

		vblank_in  <= (vcnt == Y1 && hcnt == 12'd0);
		vblank_out <= (vcnt == Y0 && hcnt == 12'd0);

		// Just before the window pixels of the FIRST output line of each
		// source line: the fetch for this line (issued 2 output lines ago)
		// must have completed; only the next line's fetch may be in flight.
		if (img_v && hcnt == X0 - 12'd4 && y_rel[0] == 1'b0
		    && (cnt_req - cnt_done) >= 2'd2) underrun <= 1'b1;
	end
end

endmodule
