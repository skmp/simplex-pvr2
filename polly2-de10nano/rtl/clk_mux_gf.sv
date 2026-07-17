// Glitch-free 4:1 clock mux in core logic (see pixclk_mux instance for why the
// hard Cyclone V clkctrl cannot do this). Classic break-before-make: the
// select is synchronized into each clock's own domain, a clock may only
// enable once every other clock's gate is observed OFF, and the final gate
// enable is registered on the FALLING edge so the AND gate never slices a
// high pulse. Worst-case dead time on a switch is ~2.5 cycles of the old
// clock + ~2.5 of the new; the core is held in reset across the window by
// clk_switch_reset anyway. The OR output is promoted to a global clock
// network via the altera_attribute.
module clk_mux_gf
(
	input  wire [3:0] clks,
	input  wire [1:0] sel,     // may be async to all clks (double-synced per domain)
	(* altera_attribute = "-name GLOBAL_SIGNAL \"GLOBAL CLOCK\"" *)
	output wire       outclk
);

wire [3:0] gate;
wire [3:0] gated;

genvar i;
generate
for (i = 0; i < 4; i = i + 1) begin : g
	reg req = 1'b0;   // sel decode + interlock, 1st posedge stage (sync)
	reg snc = 1'b0;   // 2nd posedge stage
	reg gat = 1'b0;   // negedge-registered clock gate

	wire others_off = ~|(gate & ~(4'b0001 << i));

	always @(posedge clks[i]) begin
		req <= (sel == i) && others_off;
		snc <= req && (sel == i);
	end
	always @(negedge clks[i]) gat <= snc;

	assign gate[i] = gat;

	// Own LUT per gated branch ((* keep *) stops synthesis collapsing all four
	// clocks into one LUT, where a toggling DESELECTED clock is a multi-input
	// change that can glitch the output). Each AND sees one toggling input;
	// the final OR sees constant-0 on every deselected branch.
	(* keep *) wire gclk = clks[i] & gat;
	assign gated[i] = gclk;
end
endgenerate

assign outclk = |gated;

endmodule
