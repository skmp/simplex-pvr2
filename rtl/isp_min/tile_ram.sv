//
// tile_ram - a 32x32 tile buffer as NBANKS independent single-port M10K RAMs,
// so a whole NBANKS-pixel span of a scanline can be read/written per clock while
// still mapping to block RAM (not registers).
//
// Addressing for tile-local pixel (x,y), x,y in 0..31:
//   bank = x[BW-1:0]                     (NBANKS = 2**BW, default 8)
//   addr = {y[4:0], x[4:BW]}             (DEPTH = 1024/NBANKS entries)
// In an NBANKS-wide chunk with x_base a multiple of NBANKS, the NBANKS lanes
// span x_base..x_base+NBANKS-1, whose low BW bits are 0..NBANKS-1 (distinct
// bank each) and whose high bits x[4:BW] are identical (one address/bank).
//
// Single-port per bank, registered read (1-cycle latency). Per bank per cycle:
// either a read (present addr, data valid next cycle) or a write.
//
module tile_ram #(
    parameter integer WIDTH  = 56,
    parameter integer NBANKS = 8
) (
    input                        clk,
    // per-bank access (lane i drives bank i)
    input      [NBANKS-1:0]      we,          // write-enable per bank
    input      [7*NBANKS-1:0]    addr,        // 7-bit addr per bank (packed)
    input      [WIDTH*NBANKS-1:0] wdata,      // write data per bank (packed)
    output reg [WIDTH*NBANKS-1:0] rdata       // read data per bank (packed, 1-cyc)
);
    localparam integer DEPTH = 1024 / NBANKS;  // 128 for 8 banks
    localparam integer AW    = 7;              // clog2(128)

    genvar b;
    generate
      for (b = 0; b < NBANKS; b = b + 1) begin : bank
        (* ramstyle = "M10K, no_rw_check" *) reg [WIDTH-1:0] mem [0:DEPTH-1];
        wire [AW-1:0]    a  = addr [7*b +: AW];
        wire [WIDTH-1:0] wd = wdata[WIDTH*b +: WIDTH];
        always @(posedge clk) begin
            if (we[b]) mem[a] <= wd;
            rdata[WIDTH*b +: WIDTH] <= mem[a];   // registered read (read-first)
        end
      end
    endgenerate
endmodule
