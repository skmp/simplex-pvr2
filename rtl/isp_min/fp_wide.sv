//
// fp_wide.sv - wide-significand (32-bit) intermediate float format and the ops
// that operate on it, for the higher-precision setup datapath.
//
// The C++ reference (tools/fpm.h, do_triangle_setup_pvr) carries some values as
// fpm<32> - a float with a 32-bit significand, NOT float32's 24. Products that get
// summed (fp_mul<32,24>) keep 32 output significand bits, and the sums (fpm<32>
// operator+/-) run at 32 bits, before a final truncate back to float32. Truncating
// the multiply output to float32 first would throw away exactly the 8 low bits the
// wide add needs (the point of the widening), so those ops must exchange a WIDE
// value, not a float32 word.
//
// WIDE FORMAT (a 41-bit bus, but we pass the pieces):
//   sgn : 1     sign
//   exp : [7:0] biased exponent (0 => the whole value is zero, DaZ)
//   sig : [31:0] normalized significand, leading 1 at bit31 (i.e. value =
//                sig/2^31 * 2^(exp-127)).  0 iff exp==0.
// This mirrors fpm<32>'s {sgn_, exp_, sig_} representation exactly.
//
// Rules, as everywhere: DaZ, no inf/NaN, truncate, saturate/underflow.
//
// --------------------------------------------------------------------------
// float32 -> wide (place the 24-bit significand at the top, 8 zero low bits)
module fp_to_wide (
    input  [31:0] f,
    output        sgn,
    output [7:0]  exp,
    output [31:0] sig
);
    wire [7:0] e = f[30:23];
    assign sgn = (e == 8'd0) ? 1'b0 : f[31];       // DaZ: zero is +0
    assign exp = (e == 8'd0) ? 8'd0 : e;
    assign sig = (e == 8'd0) ? 32'd0 : {1'b1, f[22:0], 8'd0};
endmodule

// --------------------------------------------------------------------------
// wide -> float32 (truncate the 32-bit significand to the 23-bit mantissa)
module fp_from_wide (
    input         sgn,
    input  [7:0]  exp,
    input  [31:0] sig,
    output [31:0] y
);
    assign y = (exp == 8'd0) ? {sgn, 31'd0}
                             : {sgn, exp, sig[30:8]};   // leading 1 at bit31 dropped, take 23
endmodule

// --------------------------------------------------------------------------
// fp_mul24_w - 24x24 float multiply, WIDE (32-bit significand) output.
//   inputs are float32 (24-bit significands); output is the wide format keeping 32
//   product significand bits (vs fp_mul24 which truncates to float32's 24). This is
//   the C++ fp_mul<32,24>. DaZ, no inf/NaN, truncate, saturate.
module fp_mul24_w (
    input  [31:0] a,
    input  [31:0] b,
    output        o_sgn,
    output [7:0]  o_exp,
    output [31:0] o_sig
);
    wire        sa = a[31], sb = b[31];
    wire [7:0]  ea = a[30:23], eb = b[30:23];
    wire a_zero = (ea == 8'd0), b_zero = (eb == 8'd0);

    wire [23:0] sig_a = {1'b1, a[22:0]};
    wire [23:0] sig_b = {1'b1, b[22:0]};
    wire        res_sign = sa ^ sb;

    // 24x24 -> 48 product. Leading 1 at bit47 (>=2) or bit46 ([1,2)).
    wire [47:0] prod = sig_a * sig_b;
    wire signed [10:0] e_sum = $signed({3'b0, ea}) + $signed({3'b0, eb}) - 11'sd127;
    wire        top = prod[47];
    // keep 32 significand bits below the leading 1 (leading 1 -> bit31).
    //   if top: leading 1 at bit47 -> sig = prod[47:16]
    //   else  : leading 1 at bit46 -> sig = prod[46:15]
    wire [31:0] sig_m = top ? prod[47:16] : prod[46:15];
    wire signed [10:0] e_adj = top ? (e_sum + 11'sd1) : e_sum;

    wire is_zero   = a_zero | b_zero;
    wire underflow = (e_adj <= 0);
    wire overflow  = (e_adj >= 255);

    assign o_sgn = res_sign;
    assign o_exp = is_zero ? 8'd0 : underflow ? 8'd0 : overflow ? 8'hFE : e_adj[7:0];
    assign o_sig = is_zero ? 32'd0 : underflow ? 32'd0 : overflow ? 32'hFFFFFFFF : sig_m;
endmodule

// --------------------------------------------------------------------------
// fp_add32_w - wide +/- wide -> wide (32-bit significand throughout).
//   y = a (sub ? - : +) b, on the wide format. Mirrors fpm<32> operator+/-.
module fp_add32_w (
    input         a_sgn, input [7:0] a_exp, input [31:0] a_sig,
    input         b_sgn, input [7:0] b_exp, input [31:0] b_sig,
    input         sub,
    output        o_sgn, output reg [7:0] o_exp, output reg [31:0] o_sig
);
    wire sbb = b_sgn ^ sub;

    // DaZ handling and larger/smaller selection (a zero operand has exp 0 and sig 0,
    // so it loses the a_ge compare and contributes 0 to the sum -> other operand).
    wire a_ge = (a_exp > b_exp) || (a_exp == b_exp && a_sig >= b_sig);
    wire        s_big_pre = a_ge ? a_sgn : sbb;
    wire [7:0]  e_big = a_ge ? a_exp : b_exp;
    wire [7:0]  e_sml = a_ge ? b_exp : a_exp;
    wire [31:0] sig_big = a_ge ? a_sig : b_sig;
    wire [31:0] sig_sml = a_ge ? b_sig : a_sig;
    wire        s_sml   = a_ge ? sbb   : a_sgn;

    wire [7:0]  shamt  = e_big - e_sml;
    wire [31:0] sml_sh = (shamt >= 8'd32) ? 32'd0 : (sig_sml >> shamt);
    wire        same   = (s_big_pre == s_sml);
    wire [32:0] sum    = same ? ({1'b0, sig_big} + {1'b0, sml_sh})
                              : ({1'b0, sig_big} - {1'b0, sml_sh});

    // normalize (leading 1 -> bit31)
    reg  [31:0] nsig; reg signed [10:0] e_norm; integer i; reg found;
    always @(*) begin
        found = 1'b0; nsig = sum[31:0]; e_norm = $signed({3'b0, e_big});
        if (sum[32]) begin nsig = sum[32:1]; e_norm = $signed({3'b0, e_big}) + 11'sd1; end
        else if (sum[31]) begin nsig = sum[31:0]; e_norm = $signed({3'b0, e_big}); end
        else begin
            for (i = 1; i < 32; i = i + 1)
                if (!found && sum[31-i]) begin
                    nsig = sum[31:0] << i; e_norm = $signed({3'b0, e_big}) - i; found = 1'b1;
                end
        end
    end

    // when one operand is zero the general path already yields the other operand
    // (big = nonzero, sml = 0) with its correct sign, so no special-case needed.
    wire res_zero  = (sum == 33'd0);
    wire underflow = (e_norm <= 0);
    wire overflow  = (e_norm >= 255);

    assign o_sgn = s_big_pre;
    always @(*) begin
        if (res_zero)       begin o_exp = 8'd0;  o_sig = 32'd0;       end
        else if (underflow) begin o_exp = 8'd0;  o_sig = 32'd0;       end
        else if (overflow)  begin o_exp = 8'hFE; o_sig = 32'hFFFFFFFF; end
        else                begin o_exp = e_norm[7:0]; o_sig = nsig;  end
    end
endmodule

// --------------------------------------------------------------------------
// fp_add3_32_w - wide + wide + wide -> wide. Aligns all three to the max exponent
// and sums at 32 significand bits (single normalize), mirroring summing fpm<32>.
module fp_add3_32_w (
    input        a_sgn, input [7:0] a_exp, input [31:0] a_sig,
    input        b_sgn, input [7:0] b_exp, input [31:0] b_sig,
    input        c_sgn, input [7:0] c_exp, input [31:0] c_sig,
    output       o_sgn, output reg [7:0] o_exp, output reg [31:0] o_sig
);
    wire za = (a_exp==8'd0), zb = (b_exp==8'd0), zc = (c_exp==8'd0);
    wire [7:0] exa = za?8'd0:a_exp, exb = zb?8'd0:b_exp, exc = zc?8'd0:c_exp;
    wire [7:0] e_ab = (exa>=exb)?exa:exb;
    wire [7:0] e_max = (e_ab>=exc)?e_ab:exc;

    wire [7:0] sha = e_max-exa, shb = e_max-exb, shc = e_max-exc;
    wire [31:0] al_a = za||(sha>=8'd32)?32'd0:(a_sig>>sha);
    wire [31:0] al_b = zb||(shb>=8'd32)?32'd0:(b_sig>>shb);
    wire [31:0] al_c = zc||(shc>=8'd32)?32'd0:(c_sig>>shc);

    wire signed [35:0] va = a_sgn?-$signed({4'b0,al_a}):$signed({4'b0,al_a});
    wire signed [35:0] vb = b_sgn?-$signed({4'b0,al_b}):$signed({4'b0,al_b});
    wire signed [35:0] vc = c_sgn?-$signed({4'b0,al_c}):$signed({4'b0,al_c});
    wire signed [35:0] ssum = va + vb + vc;

    wire        s_res = ssum[35];
    wire [34:0] mag   = s_res ? (~ssum[34:0] + 35'd1) : ssum[34:0];

    reg [31:0] nsig; reg signed [10:0] e_norm; integer i; reg found;
    always @(*) begin
        found=1'b0; nsig=32'd0; e_norm=$signed({3'b0,e_max});
        if (mag[34]) begin nsig=mag[34:3]; e_norm=$signed({3'b0,e_max})+11'sd3; found=1'b1; end
        else if (mag[33]) begin nsig=mag[33:2]; e_norm=$signed({3'b0,e_max})+11'sd2; found=1'b1; end
        else if (mag[32]) begin nsig=mag[32:1]; e_norm=$signed({3'b0,e_max})+11'sd1; found=1'b1; end
        else if (mag[31]) begin nsig=mag[31:0]; e_norm=$signed({3'b0,e_max}); found=1'b1; end
        else for (i=1;i<32;i=i+1) if(!found && mag[31-i]) begin nsig=mag[31:0]<<i; e_norm=$signed({3'b0,e_max})-i; found=1'b1; end
    end

    wire res_zero=(mag==35'd0), underflow=(e_norm<=0), overflow=(e_norm>=255);
    assign o_sgn = s_res;
    always @(*) begin
        if (res_zero) begin o_exp=8'd0; o_sig=32'd0; end
        else if (underflow) begin o_exp=8'd0; o_sig=32'd0; end
        else if (overflow) begin o_exp=8'hFE; o_sig=32'hFFFFFFFF; end
        else begin o_exp=e_norm[7:0]; o_sig=nsig; end
    end
endmodule
