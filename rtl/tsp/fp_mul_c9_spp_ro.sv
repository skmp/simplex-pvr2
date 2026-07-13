//
// fp_mul_c9_spp_ro - STREAMING PIPELINED, REGISTERED-OUTPUT variant of fp_mul_c9.
//   y = f * k, f float32 (reduced), k 9-bit SIGNED colour/offset channel.
//
// BIT-EXACT to the combinational fp_mul_c9 for the same (f, k), split into a 2-clock
// streaming pipeline so its latency matches fp_mul16_spp_ro - the two sit side by side
// as the per-vertex attribute "prime" units in tsp_setup_stream, and a uniform 2-clock
// prime keeps that schedule kind-agnostic (uv vs colour planes).
//
// CONVENTION (matches the other *_spp_ro units):
//   ports (clk, reset, stall, in_valid, f, k, out_valid, y).
//   in_valid @N -> out_valid @N+2, y @N+2 (both registered).
//   stall=1 freezes every stage. one result/clock throughput when !stall.
//
// Pipeline:
//   (comb, off inputs) decode + |k| + 16x9 product
//   [S1 REG] prod(25) + ef + sign + zero flags
//   (comb)   MSB scan + normalize + pack
//   [S2 REG] y ; out_valid
//
module fp_mul_c9_spp_ro (
    input               clk,
    input               reset,
    input               stall,
    input               in_valid,
    input        [31:0] f,
    input  signed [8:0] k,      // 9-bit signed colour value
    output reg          out_valid,
    output reg   [31:0] y
);
    // ---- combinational front (off module inputs), same as fp_mul_c9 ----
    wire        sf = f[31];
    wire [7:0]  ef = f[30:23];
    wire        f_zero = (ef == 8'd0);               // DaZ
    wire        k_zero = (k == 9'sd0);
    wire        ksign  = k[8];
    wire [8:0]  kabs   = ksign ? (~k + 9'sd1) : k;   // |k|, 0..256

    wire [15:0] sig    = {1'b1, f[22:8]};
    wire [24:0] prod_c = sig * {16'd0, kabs};        // 16 x 9 -> 25 bits

    // ================= S1 REGISTER: product + carried decode =================
    reg        v1;
    reg [24:0] s1_prod;
    reg [7:0]  s1_ef;
    reg        s1_sign, s1_zero;
    always @(posedge clk) begin
        if (reset) v1 <= 1'b0;
        else if (!stall) begin
            v1      <= in_valid;
            s1_prod <= prod_c;
            s1_ef   <= ef;
            s1_sign <= sf ^ ksign;
            s1_zero <= f_zero | k_zero;
        end
    end

    // ================= combinational normalize + pack from S1 ================
    // leading one is between bit15 (k==1) and bit24 (k up to 256).
    reg  [4:0] msb;
    integer i;
    always @(*) begin
        msb = 5'd15;
        for (i = 15; i <= 24; i = i + 1)
            if (s1_prod[i]) msb = i[4:0];
    end

    wire [5:0]  sh   = msb - 5'd15;                  // 0..9
    wire [24:0] norm = s1_prod >> sh;                // leading one -> bit15
    wire [22:0] mant = {norm[14:0], 8'b0};           // 15 real frac bits, padded

    wire signed [10:0] e = $signed({3'b0, s1_ef}) + $signed({5'b0, sh});
    wire overflow = (e >= 255);

    wire [31:0] y_c = s1_zero   ? {s1_sign, 31'd0}
                    : overflow  ? {s1_sign, 8'hFE, 23'h7FFFFF}
                                : {s1_sign, e[7:0], mant};

    // ================= S2 REGISTER: the module's registered output ==========
    always @(posedge clk) begin
        if (reset) out_valid <= 1'b0;
        else if (!stall) begin
            out_valid <= v1;
            y         <= y_c;
        end
    end
endmodule
