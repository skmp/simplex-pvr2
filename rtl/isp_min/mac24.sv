//
// mac24 - full-precision (24-bit mantissa) multiply-add lane, PIPELINED (3 stages).
//   q = (a * b)  (sub ? - : +)  c
// Identical to mac16 (same ports, same 3-cycle latency, same fp_add24 for the sum)
// but the product uses fp_mul24 (full 24x24 mantissa) instead of fp_mul16, so the
// setup datapath keeps full float32 precision on the multiplicands. ~2 DSPs per
// product vs 1 for mac16.
//
// Stages, each ending in a register so the max path is ~one FP sub-operation:
//   s1: fp_mul24 (product)                         -> reg p, (c,sub delayed)
//   s2: fp_add24_s1 (align + add/sub)              -> reg {sum,e_big,s_big}
//   s3: fp_add24_s2 (normalize + pack)             -> reg q
// Inputs sampled in cycle N -> q valid in cycle N+3.
//
// As a plain add/sub drive b = 1.0; as a plain multiply drive c = 0, sub = 0.
//
module mac24 (
    input         clk,
    input         reset,
    input  [31:0] a,
    input  [31:0] b,
    input  [31:0] c,
    input         sub,
    output [31:0] q
);
    // ---- stage 1: product ----
    wire [31:0] p_c;
    fp_mul24 u_mul (.a(a), .b(b), .y(p_c));
    reg [31:0] p_r, c_r1;
    reg        sub_r1;
    always @(posedge clk) begin
        if (reset) begin p_r <= 0; c_r1 <= 0; sub_r1 <= 0; end
        else       begin p_r <= p_c; c_r1 <= c; sub_r1 <= sub; end
    end

    // ---- stage 2: align + add/sub ----
    wire [24:0] sum_c; wire [7:0] eb_c; wire sb_c;
    fp_add24_s1 u_a1 (.a(p_r), .b_in(c_r1), .sub(sub_r1),
                      .sum(sum_c), .e_big(eb_c), .s_big(sb_c));
    reg [24:0] sum_r; reg [7:0] eb_r; reg sb_r;
    always @(posedge clk) begin
        if (reset) begin sum_r <= 0; eb_r <= 0; sb_r <= 0; end
        else       begin sum_r <= sum_c; eb_r <= eb_c; sb_r <= sb_c; end
    end

    // ---- stage 3: normalize + pack ----
    wire [31:0] q_c;
    fp_add24_s2 u_a2 (.sum(sum_r), .e_big(eb_r), .s_big(sb_r), .y(q_c));
    reg [31:0] q_r;
    always @(posedge clk) begin
        if (reset) q_r <= 0;
        else       q_r <= q_c;
    end
    assign q = q_r;
endmodule
