//
// fp_mac - clocked fused multiply-add:  q = (neg_p ? -(a*b) : (a*b)) (op) c
//          where op is add (sub=0) or subtract (sub=1):  q = prod +/- c
//
// This is the single shared arithmetic primitive for each setup sequencer. It
// wraps the proven combinational fp_mul and fp_add behind a 2-stage register
// pipeline so timing is broken across the multiply and the add:
//
//   stage 0 (issue):  latch a,b,c,sub,neg_p ; compute prod = a*b   -> reg
//   stage 1        :  compute q = prod (+/-) c                     -> reg, valid
//
// LATENCY = 2 cycles from 'in_valid' to 'out_valid'. Fully pipelined: a new op
// may be issued every cycle. Result rounding is the composition mul-then-add,
// i.e. it matches a software (a*b) followed by a separate +/- c (NOT a true
// single-rounding FMA) - which is exactly what the original flat datapath did,
// so the only numeric change vs. that datapath comes from the 1/C reciprocal.
//
module fp_mac (
    input         clk,
    input         reset,

    input         in_valid,
    input  [31:0] a,
    input  [31:0] b,
    input  [31:0] c,
    input         sub,      // 0: prod + c   1: prod - c
    input         neg_p,    // negate the product before the add/sub

    output reg        out_valid,
    output reg [31:0] q
);
    // ---- stage 0: multiply ----
    wire [31:0] prod_comb;
    fp_mul u_mul (.a(a), .b(b), .y(prod_comb));
    wire [31:0] prod_signed = neg_p ? {~prod_comb[31], prod_comb[30:0]} : prod_comb;

    reg [31:0] s0_prod;
    reg [31:0] s0_c;
    reg        s0_sub;
    reg        s0_valid;

    always @(posedge clk) begin
        if (reset) begin
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= in_valid;
            s0_prod  <= prod_signed;
            s0_c     <= c;
            s0_sub   <= sub;
        end
    end

    // ---- stage 1: add/sub ----
    wire [31:0] q_comb;
    fp_add u_add (.a(s0_prod), .b_in(s0_c), .sub(s0_sub), .y(q_comb));

    always @(posedge clk) begin
        if (reset) begin
            out_valid <= 1'b0;
        end else begin
            out_valid <= s0_valid;
            q         <= q_comb;
        end
    end
endmodule
