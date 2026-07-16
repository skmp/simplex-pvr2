//
// fmac_seq - issue-and-wait wrapper presenting one mul+add as a single
//            "compute q = (neg_p ? -(a*b) : a*b) (sub ? - : +) c" operation.
//
// Two builds, selected by `SYNTHESIS:
//
//   sim   (Verilator): behavioral fp_mul -> fp_add (round-to-nearest-even),
//                      2-cycle pipe via fp_mac. Bit-comparable to the C golden.
//
//   synth (Quartus)  : Altera FP IP - altera_fp_mul (LAT_MUL) feeding
//                      altera_fp_short_add (LAT_ADD), DSP-mapped and deeply
//                      pipelined for high Fmax. The IP has NO valid handshake,
//                      so a shift register delays 'req' by the total pipeline
//                      latency to produce 'ack'. The IP rounds "nearest, ties
//                      away from zero" (not RNE) - results differ from the
//                      behavioral path by ~1 ULP, so the golden check uses a
//                      tolerance.
//
// Negation/subtraction: the IP mul/add have no sign controls, so neg_p flips
// the product's sign bit (applied to operand 'a' going into the multiplier),
// and 'sub' flips the addend's sign bit going into the adder. (a*b) sign flip
// via either operand is exact in IEEE-754.
//
module fmac_seq #(
    parameter LAT_MUL = 7,   // altera_fp_mul latency (cycles)
    parameter LAT_ADD = 7    // altera_fp_short_add latency (cycles)
)(
    input         clk,
    input         reset,
    input         req,
    input  [31:0] a,
    input  [31:0] b,
    input  [31:0] c,
    input         sub,
    input         neg_p,
    output        ack,
    output [31:0] q
);
`ifdef SYNTHESIS
    // ---- Altera FP IP datapath ----
    localparam LAT = LAT_MUL + LAT_ADD;

    // latch operands at issue so they are stable through the multiplier
    reg [31:0] ra, rb, rc;
    reg        rsub, rneg;
    always @(posedge clk) begin
        if (req) begin ra<=a; rb<=b; rc<=c; rsub<=sub; rneg<=neg_p; end
    end

    // neg_p: flip sign of the multiplier's 'a' input -> product is negated.
    wire [31:0] mul_a = rneg ? {~ra[31], ra[30:0]} : ra;
    wire [31:0] prod;
    altera_fp_mul u_mul (.clk(clk), .areset(reset), .a(mul_a), .b(rb), .q(prod));

    // 'rsub' delayed to align with the product arriving at the adder, then
    // applied as a sign flip on the adder's 'b' (addend) input.
    reg [LAT_MUL-1:0] subpipe;
    always @(posedge clk) subpipe <= {subpipe[LAT_MUL-2:0], rsub};
    wire sub_at_add = subpipe[LAT_MUL-1];

    // rc must reach the adder aligned with the product: delay it LAT_MUL cycles.
    reg [31:0] cpipe [0:LAT_MUL-1];
    integer i;
    always @(posedge clk) begin
        cpipe[0] <= rc;
        for (i=1;i<LAT_MUL;i=i+1) cpipe[i] <= cpipe[i-1];
    end
    wire [31:0] c_aligned = cpipe[LAT_MUL-1];
    wire [31:0] add_b = sub_at_add ? {~c_aligned[31], c_aligned[30:0]} : c_aligned;

    altera_fp_short_add u_add (.clk(clk), .areset(reset), .a(prod), .b(add_b), .q(q));

    // valid: operands latch 1 cycle after 'req', then traverse the mul+add
    // pipeline (LAT cycles) -> ack is LAT+1 after req. The c-align and sub
    // pipelines above start from the latched (T+1) operands, matching this.
    localparam VLEN = LAT + 1;
    reg [VLEN-1:0] vpipe;
    always @(posedge clk) begin
        if (reset) vpipe <= 0;
        else       vpipe <= {vpipe[VLEN-2:0], req};
    end
    assign ack = vpipe[VLEN-1];

`else
    // ---- behavioral datapath (Verilator) ----
    reg        mac_in_valid;
    wire       mac_out_valid;
    wire [31:0] mac_q;
    reg [31:0] ra, rb, rc;
    reg        rsub, rneg;
    fp_mac u_mac (
        .clk(clk), .reset(reset),
        .in_valid(mac_in_valid), .a(ra), .b(rb), .c(rc),
        .sub(rsub), .neg_p(rneg), .out_valid(mac_out_valid), .q(mac_q)
    );
    reg        ack_r; reg [31:0] q_r;
    assign ack = ack_r; assign q = q_r;
    always @(posedge clk) begin
        if (reset) begin mac_in_valid<=0; ack_r<=0; end
        else begin
            ack_r<=0; mac_in_valid<=0;
            if (req) begin ra<=a; rb<=b; rc<=c; rsub<=sub; rneg<=neg_p; mac_in_valid<=1; end
            if (mac_out_valid) begin q_r<=mac_q; ack_r<=1; end
        end
    end
`endif
endmodule
