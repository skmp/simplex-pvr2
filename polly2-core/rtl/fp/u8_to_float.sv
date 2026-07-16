//
// u8_to_float - convert an unsigned 8-bit integer (0..255) to IEEE-754 single.
//
// Combinational. Used to widen the packed vertex colour bytes into the float
// domain before the perspective multiply in tsp_setup (matches the implicit
// u8->float promotion in refsw2's  v.col[i] * v.z ).
//
module u8_to_float (
    input  [7:0]  u,
    output [31:0] f
);
    // Find the position of the most-significant set bit (0..7).
    reg [2:0] msb;
    reg       any;
    integer   i;
    always @(*) begin
        msb = 3'd0;
        any = 1'b0;
        for (i = 0; i < 8; i = i + 1) begin
            if (u[i]) begin
                msb = i[2:0];
                any = 1'b1;
            end
        end
    end

    // value = 1.mant * 2^msb ; exponent biased = 127 + msb
    // mantissa: the bits below the leading one, left-justified into 23 bits.
    wire [7:0]  exp   = 8'd127 + {5'd0, msb};
    // shift u left so the leading 1 moves to bit 7, then drop it and place the
    // remaining low bits at the top of the 23-bit mantissa.
    wire [7:0]  norm  = u << (3'd7 - msb);   // leading 1 now at bit 7
    wire [22:0] mant  = {norm[6:0], 16'd0};  // bits below leading 1

    assign f = (~any) ? 32'h00000000 : {1'b0, exp, mant};
endmodule
