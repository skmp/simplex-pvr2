// hps_lw_bridge.sv - HPS lightweight HPS-to-FPGA bridge (0xFF200000, 2MB)
// exposed as a dead-simple 32-bit Avalon-MM master, hand-decoded downstream.
//
// The cyclonev_hps_interface_hps2fpga_light_weight WYSIWYG atom is
// instantiated directly (same style as the framework's hand-instantiated
// HPS atoms in sysmem.sv - a Qsys system cannot coexist with those, as both
// would instantiate HPS interface atoms). The atom presents an AXI-3 32-bit
// MASTER (the HPS side) into the fabric; this module converts it to
// single-beat Avalon-MM:
//
//   address[20:0] (byte), read, write, writedata[31:0], byteenable[3:0],
//   readdata[31:0], readdatavalid, waitrequest
//
// Only single-beat transfers are supported (ARM /dev/mem Device-memory
// mappings issue single 32-bit accesses on this window; bursts don't occur).
// One transaction outstanding at a time; write AW/W channels may arrive in
// either order. All responses are OKAY.
//
// If Quartus errors on an atom port name (this list matches Qsys 17.0
// output), generate a throwaway Qsys system with the HPS + LW bridge
// enabled and diff the generated cyclonev_hps_interface_hps2fpga_light_weight
// instantiation against this one.

module hps_lw_bridge
(
	input  wire        clk,            // Avalon clock; also drives the bridge

	// Avalon-MM master towards the fabric decoder
	output reg  [20:0] avm_address,
	output reg         avm_read,
	output reg         avm_write,
	output reg  [31:0] avm_writedata,
	output reg   [3:0] avm_byteenable,
	input  wire [31:0] avm_readdata,
	input  wire        avm_readdatavalid,
	input  wire        avm_waitrequest
);

// ---- AXI-3 signals to/from the LW bridge atom (HPS is the master) ----
wire [11:0] awid;
wire [20:0] awaddr;
/* verilator lint_off UNUSEDSIGNAL */
wire  [3:0] awlen;
wire  [2:0] awsize;
wire  [1:0] awburst;
wire  [1:0] awlock;
wire  [3:0] awcache;
wire  [2:0] awprot;
wire  [3:0] arlen;
wire  [2:0] arsize;
wire  [1:0] arburst;
wire  [1:0] arlock;
wire  [3:0] arcache;
wire  [2:0] arprot;
wire [11:0] wid;
/* verilator lint_on UNUSEDSIGNAL */
wire        awvalid;
reg         awready = 1'b0;
wire [31:0] wdata;
wire  [3:0] wstrb;
wire        wlast;
wire        wvalid;
reg         wready  = 1'b0;
reg  [11:0] bid     = 12'd0;
reg         bvalid  = 1'b0;
wire        bready;
wire [11:0] arid;
wire [20:0] araddr;
wire        arvalid;
reg         arready = 1'b0;
reg  [11:0] rid     = 12'd0;
reg  [31:0] rdata   = 32'd0;
reg         rvalid  = 1'b0;
wire        rready;

cyclonev_hps_interface_hps2fpga_light_weight hps2fpga_light_weight
(
	.clk    (clk),

	.awid   (awid),
	.awaddr (awaddr),
	.awlen  (awlen),
	.awsize (awsize),
	.awburst(awburst),
	.awlock (awlock),
	.awcache(awcache),
	.awprot (awprot),
	.awvalid(awvalid),
	.awready(awready),

	.wid    (wid),
	.wdata  (wdata),
	.wstrb  (wstrb),
	.wlast  (wlast),
	.wvalid (wvalid),
	.wready (wready),

	.bid    (bid),
	.bresp  (2'b00),          // OKAY
	.bvalid (bvalid),
	.bready (bready),

	.arid   (arid),
	.araddr (araddr),
	.arlen  (arlen),
	.arsize (arsize),
	.arburst(arburst),
	.arlock (arlock),
	.arcache(arcache),
	.arprot (arprot),
	.arvalid(arvalid),
	.arready(arready),

	.rid    (rid),
	.rdata  (rdata),
	.rresp  (2'b00),          // OKAY
	.rlast  (1'b1),           // single-beat only
	.rvalid (rvalid),
	.rready (rready)
);

// ---- AXI-3 single-beat slave -> Avalon-MM master ----
localparam S_IDLE   = 3'd0;
localparam S_WADDR  = 3'd1;   // have W, waiting for AW
localparam S_WDATA  = 3'd2;   // have AW, waiting for W
localparam S_WISSUE = 3'd3;   // Avalon write until !waitrequest
localparam S_BRESP  = 3'd4;
localparam S_RISSUE = 3'd5;   // Avalon read until !waitrequest
localparam S_RWAIT  = 3'd6;   // waiting for readdatavalid
localparam S_RRESP  = 3'd7;

reg [2:0] st = S_IDLE;

always @(posedge clk) begin
	// single-cycle ready pulses
	awready <= 1'b0;
	wready  <= 1'b0;
	arready <= 1'b0;

	case (st)
	S_IDLE: begin
		// writes win arbitration; one transaction at a time either way
		if (awvalid) begin
			awready     <= 1'b1;
			bid         <= awid;
			avm_address <= awaddr;
			st          <= S_WDATA;
		end
		else if (wvalid) begin
			wready         <= 1'b1;
			avm_writedata  <= wdata;
			avm_byteenable <= wstrb;
			st             <= S_WADDR;
		end
		else if (arvalid) begin
			arready     <= 1'b1;
			rid         <= arid;
			avm_address <= araddr;
			st          <= S_RISSUE;
			avm_read    <= 1'b1;
		end
	end

	S_WADDR: if (awvalid) begin
		awready     <= 1'b1;
		bid         <= awid;
		avm_address <= awaddr;
		avm_write   <= 1'b1;
		st          <= S_WISSUE;
	end

	S_WDATA: if (wvalid) begin
		wready         <= 1'b1;
		avm_writedata  <= wdata;
		avm_byteenable <= wstrb;
		avm_write      <= 1'b1;
		st             <= S_WISSUE;
	end

	S_WISSUE: if (!avm_waitrequest) begin
		avm_write <= 1'b0;
		bvalid    <= 1'b1;
		st        <= S_BRESP;
	end

	S_BRESP: if (bready) begin
		bvalid <= 1'b0;
		st     <= S_IDLE;
	end

	S_RISSUE: if (!avm_waitrequest) begin
		avm_read <= 1'b0;
		st       <= S_RWAIT;
	end

	S_RWAIT: if (avm_readdatavalid) begin
		rdata  <= avm_readdata;
		rvalid <= 1'b1;
		st     <= S_RRESP;
	end

	S_RRESP: if (rready) begin
		rvalid <= 1'b0;
		st     <= S_IDLE;
	end

	default: st <= S_IDLE;
	endcase
end

initial begin
	avm_address    = 21'd0;
	avm_read       = 1'b0;
	avm_write      = 1'b0;
	avm_writedata  = 32'd0;
	avm_byteenable = 4'd0;
end

endmodule
