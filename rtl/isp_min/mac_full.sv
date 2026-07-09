//
// mac_full - extended-precision multiply-add lane on the setup float (xf) bus,
// the wide analogue of mac16 used for the precision-critical triangle setup:
//   q = (a * b)  (sub ? - : +)  c        (all operands in 41-bit xf format)
//
// PIPELINED (3 stages) for timing, each ending in a register:
//   s1: fp_mul_full (full 32x32 product)          -> reg p, (c,sub delayed)
//   s2: fp_add32_s1 (align + add/sub)             -> reg {sum,e_big,s_big}
//   s3: fp_add32_s2 (normalize + pack)            -> reg q
// Inputs sampled in cycle N -> q valid in cycle N+3. c/sub are delayed to line
// up with the registered product, exactly like mac16.
//
// As a plain add/sub drive b = 1.0 (xf); as a plain multiply drive c = 0, sub=0.
// This is a drop-in wide replacement for mac16 with 41-bit ports instead of 32.
//
module mac_full (
    input         clk,
    input         reset,
    input  [40:0] a,
    input  [40:0] b,
    input  [40:0] c,
    input         sub,
    output [40:0] q
);
    // ---- stage 1: product ----
    wire [40:0] p_c;
    fp_mul_full u_mul (.a(a), .b(b), .y(p_c));
    reg [40:0] p_r, c_r1;
    reg        sub_r1;
    always @(posedge clk) begin
        if (reset) begin p_r <= 0; c_r1 <= 0; sub_r1 <= 0; end
        else       begin p_r <= p_c; c_r1 <= c; sub_r1 <= sub; end
    end

    // ---- stage 2: align + add/sub ----
    wire [32:0] sum_c; wire [7:0] eb_c; wire sb_c;
    fp_add32_s1 u_a1 (.a(p_r), .b_in(c_r1), .sub(sub_r1),
                      .sum(sum_c), .e_big(eb_c), .s_big(sb_c));
    reg [32:0] sum_r; reg [7:0] eb_r; reg sb_r;
    always @(posedge clk) begin
        if (reset) begin sum_r <= 0; eb_r <= 0; sb_r <= 0; end
        else       begin sum_r <= sum_c; eb_r <= eb_c; sb_r <= sb_c; end
    end

    // ---- stage 3: normalize + pack ----
    wire [40:0] q_c;
    fp_add32_s2 u_a2 (.sum(sum_r), .e_big(eb_r), .s_big(sb_r), .y(q_c));
    reg [40:0] q_r;
    always @(posedge clk) begin
        if (reset) q_r <= 0;
        else       q_r <= q_c;
    end
    assign q = q_r;
endmodule
