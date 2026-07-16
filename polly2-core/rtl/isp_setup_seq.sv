//
// isp_setup_seq - clocked ISP triangle setup using ONE shared mul+add
//                 (fmac_seq) and the design's single reciprocal (fp_recip).
//
// Every arithmetic value - tri_area, the PlaneStepper geometry, edge diffs,
// edge DX/DY, the four Cn, and the invW plane - is produced by the SAME MAC,
// one micro-op at a time, results landing in a scratchpad register file (rf).
// The "C optimization" replaces all divides with one reciprocal: recip_C = 1/C
// computed once (between the area phase and the rest) and reused as a multiply.
// recip_C and the shared geometry are exported to tsp_setup_seq, so the whole
// design has exactly one inverter.
//
// Top-left flags, sgn, and cull are pure sign/zero tests on rf values (no extra
// arithmetic units). is_quad selects the 31/41 edge variants at capture time
// from precomputed rf entries.
//
// Numerics match refsw2 except the reciprocal-multiply (vs true divide) on
// ddx/ddy/c -> a few ULP; the TB uses a tolerance check.
//
// Structure: ONE op ROM + ONE executor (issue->wait->store->next) + a capture
// state. Authoring is a plain sequential dependency list; no hazard tracking.
//
module isp_setup_seq (
    input         clk,
    input         reset,
    input         start,
    output reg    done,

    input  [31:0] isp_tsp,
    input         is_quad,

    input  [31:0] v1x, input [31:0] v1y, input [31:0] v1z,
    input  [31:0] v2x, input [31:0] v2y, input [31:0] v2z,
    input  [31:0] v3x, input [31:0] v3y, input [31:0] v3z,
    input  [31:0] v4x, input [31:0] v4y,
    input  [31:0] rect_left,
    input  [31:0] rect_top,

    output reg        sgn_neg,
    output reg        cull,
    output reg [31:0] dx12, output reg [31:0] dx23, output reg [31:0] dx31, output reg [31:0] dx41,
    output reg [31:0] dy12, output reg [31:0] dy23, output reg [31:0] dy31, output reg [31:0] dy41,
    output reg [31:0] c1, output reg [31:0] c2, output reg [31:0] c3, output reg [31:0] c4,
    output reg        t1, output reg t2, output reg t3, output reg t4,
    output reg [31:0] z_ddx, output reg [31:0] z_ddy, output reg [31:0] z_c,

    output reg [31:0] geo_dx12, output reg [31:0] geo_dx13,
    output reg [31:0] geo_dy12, output reg [31:0] geo_dy13,
    output reg [31:0] geo_recipC,
    output reg [31:0] geo_v1x_l, output reg [31:0] geo_v1y_t,
    output reg [31:0] geo_v1z,  output reg [31:0] geo_v2z, output reg [31:0] geo_v3z
);
    localparam [31:0] ONE = 32'h3f800000, ZERO = 32'h0, NEG1 = 32'hbf800000;

    // ---------------- scratchpad ----------------
    localparam RF_AW = 7, RF_N = 96;
    reg [31:0] rf [0:RF_N-1];

    // constants / inputs
    localparam R_ZERO=0,R_ONE=1,R_LEFT=2,R_TOP=3;
    localparam R_X1=4,R_Y1=5,R_Z1=6, R_X2=7,R_Y2=8,R_Z2=9, R_X3=10,R_Y3=11,R_Z3=12, R_X4=13,R_Y4=14;
    localparam R_SGN=15, R_RECIPC=16;
    // tri_area
    localparam R_X1X3=17,R_Y2Y3=18,R_Y1Y3=19,R_X2X3=20,R_AREA0=21,R_TRIAREA=22;
    // plane geo
    localparam R_DX12r=23,R_DX13r=24,R_DY12r=25,R_DY13r=26,R_C0=27,R_CAREA=28;
    // coord-rect diffs (for Cn) and v1x_l/v1y_t
    localparam R_X1mL=29,R_X2mL=30,R_X3mL=31,R_X4mL=32,R_Y1mT=33,R_Y2mT=34,R_Y3mT=35,R_Y4mT=36;
    // edge raw diffs (pre-sgn)
    localparam R_X1mX2=37,R_X2mX3=38,R_X3mX1=39,R_X3mX4=40,R_X4mX1=41;
    localparam R_Y1mY2=42,R_Y2mY3=43,R_Y3mY1=44,R_Y3mY4=45,R_Y4mY1=46;
    // edge DX/DY (post-sgn)
    localparam R_DX12=47,R_DX23=48,R_DX31=49,R_DX41=50,R_DY12=51,R_DY23=52,R_DY31=53,R_DY41=54;
    // Cn temps + results
    localparam R_CT=55,R_C1=56,R_C2=57,R_C3=58,R_C4=59;
    // top-left diffs (sign tested only)
    localparam R_X2mX1=60,R_Y2mY1=61,R_X3mX2=62,R_Y3mY2=63,R_X1mX3=64,R_Y1mY3=65;
    localparam R_X4mX3=66,R_Y4mY3=67,R_X1mX4=68,R_Y1mY4=69;
    // invW plane
    localparam R_DA2=70,R_DA3=71,R_AA0=72,R_AA=73,R_BA0=74,R_BA=75,R_ZDDX=76,R_ZDDY=77,R_ZC0=78,R_ZC=79;
    localparam R_T=80;

    // ---------------- one MAC ----------------
    reg        req; reg [31:0] opa,opb,opc; reg osub,oneg;
    wire ack; wire [31:0] qres;
    fmac_seq u_mac (.clk(clk),.reset(reset),.req(req),.a(opa),.b(opb),.c(opc),
                    .sub(osub),.neg_p(oneg),.ack(ack),.q(qres));

    // ---------------- one reciprocal ----------------
    reg rc_req; wire rc_ack; wire [31:0] rc_y;
    fp_recip u_recip (.clk(clk),.reset(reset),.in_valid(rc_req),.x(rf[R_CAREA]),
                      .out_valid(rc_ack),.y(rc_y));

    // ---------------- op ROM ----------------
    // rf[dst] = (neg?-:+)(rf[sa]*rf[sb]) (sub?-:+) rf[sc]
    // subtract x-y := x*ONE - y ; multiply x*y := x*y + ZERO
    localparam NOPS = 60;
    reg [RF_AW-1:0] o_d [0:NOPS-1], o_a [0:NOPS-1], o_b [0:NOPS-1], o_c [0:NOPS-1];
    reg             o_s [0:NOPS-1], o_n [0:NOPS-1];

    // phase boundaries (pc values)
    localparam P_AREA_END = 12;   // ops 0..11 then reciprocal
    // ops 12..NOPS-1 run after reciprocal

    task SET(input integer i, input [RF_AW-1:0] d,a,b,c, input s,n);
        begin o_d[i]=d; o_a[i]=a; o_b[i]=b; o_c[i]=c; o_s[i]=s; o_n[i]=n; end
    endtask

    initial begin
        // ---- ops 0..11: tri_area + plane geo + C ----
        SET(0, R_X1X3, R_X1,R_ONE,R_X3, 1,0);
        SET(1, R_Y2Y3, R_Y2,R_ONE,R_Y3, 1,0);
        SET(2, R_Y1Y3, R_Y1,R_ONE,R_Y3, 1,0);
        SET(3, R_X2X3, R_X2,R_ONE,R_X3, 1,0);
        SET(4, R_AREA0,   R_Y1Y3,R_X2X3,R_ZERO, 0,0);
        SET(5, R_TRIAREA, R_X1X3,R_Y2Y3,R_AREA0,1,0);
        SET(6, R_DX12r, R_X2,R_ONE,R_X1, 1,0);
        SET(7, R_DX13r, R_X3,R_ONE,R_X1, 1,0);
        SET(8, R_DY12r, R_Y2,R_ONE,R_Y1, 1,0);
        SET(9, R_DY13r, R_Y3,R_ONE,R_Y1, 1,0);
        SET(10,R_C0,    R_DX13r,R_DY12r,R_ZERO, 0,0);
        SET(11,R_CAREA, R_DX12r,R_DY13r,R_C0,   1,0);
        // ---- ops 12..: post-reciprocal ----
        // coord-rect diffs
        SET(12,R_X1mL,R_X1,R_ONE,R_LEFT,1,0);
        SET(13,R_X2mL,R_X2,R_ONE,R_LEFT,1,0);
        SET(14,R_X3mL,R_X3,R_ONE,R_LEFT,1,0);
        SET(15,R_X4mL,R_X4,R_ONE,R_LEFT,1,0);
        SET(16,R_Y1mT,R_Y1,R_ONE,R_TOP,1,0);
        SET(17,R_Y2mT,R_Y2,R_ONE,R_TOP,1,0);
        SET(18,R_Y3mT,R_Y3,R_ONE,R_TOP,1,0);
        SET(19,R_Y4mT,R_Y4,R_ONE,R_TOP,1,0);
        // edge raw diffs (both tri & quad variants)
        SET(20,R_X1mX2,R_X1,R_ONE,R_X2,1,0);
        SET(21,R_X2mX3,R_X2,R_ONE,R_X3,1,0);
        SET(22,R_X3mX1,R_X3,R_ONE,R_X1,1,0);
        SET(23,R_X3mX4,R_X3,R_ONE,R_X4,1,0);
        SET(24,R_X4mX1,R_X4,R_ONE,R_X1,1,0);
        SET(25,R_Y1mY2,R_Y1,R_ONE,R_Y2,1,0);
        SET(26,R_Y2mY3,R_Y2,R_ONE,R_Y3,1,0);
        SET(27,R_Y3mY1,R_Y3,R_ONE,R_Y1,1,0);
        SET(28,R_Y3mY4,R_Y3,R_ONE,R_Y4,1,0);
        SET(29,R_Y4mY1,R_Y4,R_ONE,R_Y1,1,0);
        // top-left diffs
        SET(30,R_X2mX1,R_X2,R_ONE,R_X1,1,0);
        SET(31,R_Y2mY1,R_Y2,R_ONE,R_Y1,1,0);
        SET(32,R_X3mX2,R_X3,R_ONE,R_X2,1,0);
        SET(33,R_Y3mY2,R_Y3,R_ONE,R_Y2,1,0);
        SET(34,R_X1mX3,R_X1,R_ONE,R_X3,1,0);
        SET(35,R_Y1mY3,R_Y1,R_ONE,R_Y3,1,0);
        SET(36,R_X4mX3,R_X4,R_ONE,R_X3,1,0);
        SET(37,R_Y4mY3,R_Y4,R_ONE,R_Y3,1,0);
        SET(38,R_X1mX4,R_X1,R_ONE,R_X4,1,0);
        SET(39,R_Y1mY4,R_Y1,R_ONE,R_Y4,1,0);
        // invW plane
        SET(40,R_DA2,R_Z2,R_ONE,R_Z1,1,0);
        SET(41,R_DA3,R_Z3,R_ONE,R_Z1,1,0);
        SET(42,R_AA0,R_DA2,R_DY13r,R_ZERO,0,0);
        SET(43,R_AA, R_DA3,R_DY12r,R_AA0,1,0);
        SET(44,R_BA0,R_DX12r,R_DA3,R_ZERO,0,0);
        SET(45,R_BA, R_DX13r,R_DA2,R_BA0,1,0);
        SET(46,R_ZDDX,R_AA,R_RECIPC,R_ZERO,0,1);
        SET(47,R_ZDDY,R_BA,R_RECIPC,R_ZERO,0,1);
        SET(48,R_ZC0, R_ZDDX,R_X1mL,R_Z1,0,1);
        SET(49,R_ZC,  R_ZDDY,R_Y1mT,R_ZC0,0,1);
        // pad to NOPS
        SET(50,R_T,R_ZERO,R_ZERO,R_ZERO,0,0); SET(51,R_T,R_ZERO,R_ZERO,R_ZERO,0,0);
        SET(52,R_T,R_ZERO,R_ZERO,R_ZERO,0,0); SET(53,R_T,R_ZERO,R_ZERO,R_ZERO,0,0);
        SET(54,R_T,R_ZERO,R_ZERO,R_ZERO,0,0); SET(55,R_T,R_ZERO,R_ZERO,R_ZERO,0,0);
        SET(56,R_T,R_ZERO,R_ZERO,R_ZERO,0,0); SET(57,R_T,R_ZERO,R_ZERO,R_ZERO,0,0);
        SET(58,R_T,R_ZERO,R_ZERO,R_ZERO,0,0); SET(59,R_T,R_ZERO,R_ZERO,R_ZERO,0,0);
    end
    localparam LAST_OP = 49;

    // ---------------- sign/zero helpers ----------------
    function fzero(input [31:0] f); fzero=(f[30:0]==31'd0); endfunction
    function fneg (input [31:0] f); fneg = f[31]&&(f[30:0]!=31'd0); endfunction
    function fpos (input [31:0] f); fpos = !f[31]&&(f[30:0]!=31'd0); endfunction
    function istl (input [31:0] fx, input [31:0] fy);
        istl=(fzero(fy)&&fpos(fx))||fneg(fy); endfunction

    // ---------------- FSM ----------------
    localparam S_IDLE=0,S_LOAD=1,S_A_ISS=2,S_A_WT=3,S_CFIX=4,S_RREQ=5,S_RWT=6,
               S_B_ISS=7,S_B_WT=8,S_EDGE=9,S_CN=10,S_CAP=11,S_DONE=12;
    reg [3:0] state;
    reg [RF_AW-1:0] pc;
    reg [2:0] cn_i; reg cn_sub;
    reg       issued;   // in EDGE/CN: 1 after the single req pulse, until ack

    always @(posedge clk) begin
        if (reset) begin state<=S_IDLE; done<=0; req<=0; rc_req<=0; end
        else begin
            done<=0; req<=0; rc_req<=0;
            case (state)
            S_IDLE: if (start) state<=S_LOAD;
            S_LOAD: begin
                rf[R_ZERO]<=ZERO; rf[R_ONE]<=ONE; rf[R_LEFT]<=rect_left; rf[R_TOP]<=rect_top;
                rf[R_X1]<=v1x;rf[R_Y1]<=v1y;rf[R_Z1]<=v1z;
                rf[R_X2]<=v2x;rf[R_Y2]<=v2y;rf[R_Z2]<=v2z;
                rf[R_X3]<=v3x;rf[R_Y3]<=v3y;rf[R_Z3]<=v3z;
                rf[R_X4]<=v4x;rf[R_Y4]<=v4y;
                pc<=0; state<=S_A_ISS;
            end
            // area phase ops 0..11
            S_A_ISS: begin req<=1; opa<=rf[o_a[pc]];opb<=rf[o_b[pc]];opc<=rf[o_c[pc]];
                           osub<=o_s[pc];oneg<=o_n[pc]; state<=S_A_WT; end
            S_A_WT: if (ack) begin rf[o_d[pc]]<=qres;
                        if (pc==P_AREA_END-1) state<=S_CFIX;
                        else begin pc<=pc+1; state<=S_A_ISS; end end
            S_CFIX: begin if (fzero(rf[R_CAREA])) rf[R_CAREA]<=ONE; state<=S_RREQ; end
            S_RREQ: begin rc_req<=1; state<=S_RWT; end
            S_RWT: if (rc_ack) begin
                        rf[R_RECIPC]<=rc_y; geo_recipC<=rc_y;
                        sgn_neg<=fpos(rf[R_TRIAREA]);
                        rf[R_SGN]<= fpos(rf[R_TRIAREA]) ? NEG1 : ONE;
                        // cull (FPU_CULL_VAL=0 so 'small' never true)
                        begin : cb
                          reg [1:0] cm; reg tapos,taneg,wrong;
                          cm=isp_tsp[4:3]; tapos=fpos(rf[R_TRIAREA]); taneg=fneg(rf[R_TRIAREA]);
                          wrong=(cm[0]==0&&taneg)||(cm[0]==1&&tapos);
                          cull<=(cm!=0)&&((cm>=2)&&wrong);
                        end
                        pc<=12; state<=S_B_ISS;
                    end
            // ops 12..LAST_OP (diffs, edges-raw, TL diffs, plane)
            S_B_ISS: begin req<=1; opa<=rf[o_a[pc]];opb<=rf[o_b[pc]];opc<=rf[o_c[pc]];
                           osub<=o_s[pc];oneg<=o_n[pc]; state<=S_B_WT; end
            S_B_WT: if (ack) begin rf[o_d[pc]]<=qres;
                        if (pc==LAST_OP) begin pc<=0; issued<=0; state<=S_EDGE; end
                        else begin pc<=pc+1; state<=S_B_ISS; end end
            // DX/DY = SGN * (selected raw diff). 8 edges via pc=0..7.
            S_EDGE: begin
                if (!issued) begin
                    req<=1; issued<=1; osub<=0; oneg<=0; opc<=ZERO; opa<=rf[R_SGN];
                    case (pc)
                      0: opb<=rf[R_X1mX2];
                      1: opb<=rf[R_X2mX3];
                      2: opb<= is_quad ? rf[R_X3mX4] : rf[R_X3mX1];
                      3: opb<= is_quad ? rf[R_X4mX1] : ZERO;
                      4: opb<=rf[R_Y1mY2];
                      5: opb<=rf[R_Y2mY3];
                      6: opb<= is_quad ? rf[R_Y3mY4] : rf[R_Y3mY1];
                      7: opb<= is_quad ? rf[R_Y4mY1] : ZERO;
                    endcase
                end else if (ack) begin
                    case (pc)
                      0: dx12<=qres; 1: dx23<=qres; 2: dx31<=qres; 3: dx41<=qres;
                      4: dy12<=qres; 5: dy23<=qres; 6: dy31<=qres; 7: dy41<=qres;
                    endcase
                    issued<=0;
                    if (pc==7) begin cn_i<=0; cn_sub<=0; state<=S_CN; end
                    else pc<=pc+1;
                end
            end
            // Cn = DYnn*Xn_l - DXnn*Yn_t (2 ops)
            S_CN: begin
                if (cn_sub==0) begin
                    if (!issued) begin
                        req<=1; issued<=1; osub<=0; oneg<=0; opc<=ZERO;
                        case (cn_i)
                          0: begin opa<=dy12; opb<=rf[R_X1mL]; end
                          1: begin opa<=dy23; opb<=rf[R_X2mL]; end
                          2: begin opa<=dy31; opb<=rf[R_X3mL]; end
                          3: begin opa<=dy41; opb<=rf[R_X4mL]; end
                        endcase
                    end else if (ack) begin rf[R_CT]<=qres; cn_sub<=1; issued<=0; end
                end else begin
                    if (!issued) begin
                        req<=1; issued<=1; osub<=0; oneg<=1; opc<=rf[R_CT];
                        case (cn_i)
                          0: begin opa<=dx12; opb<=rf[R_Y1mT]; end
                          1: begin opa<=dx23; opb<=rf[R_Y2mT]; end
                          2: begin opa<=dx31; opb<=rf[R_Y3mT]; end
                          3: begin opa<=dx41; opb<=rf[R_Y4mT]; end
                        endcase
                    end else if (ack) begin
                        case (cn_i)
                          0: c1<=qres; 1: c2<=qres; 2: c3<=qres;
                          3: c4<= is_quad ? qres : ONE;
                        endcase
                        issued<=0; cn_sub<=0;
                        if (cn_i==3) state<=S_CAP;
                        else cn_i<=cn_i+1;
                    end
                end
            end
            // final captures: plane, geo, edge 31/41 fixup, top-left
            S_CAP: begin
                dx41<= is_quad ? dx41 : ZERO; dy41<= is_quad ? dy41 : ZERO;
                z_ddx<=rf[R_ZDDX]; z_ddy<=rf[R_ZDDY]; z_c<=rf[R_ZC];
                geo_dx12<=rf[R_DX12r]; geo_dx13<=rf[R_DX13r];
                geo_dy12<=rf[R_DY12r]; geo_dy13<=rf[R_DY13r];
                geo_v1x_l<=rf[R_X1mL]; geo_v1y_t<=rf[R_Y1mT];
                geo_v1z<=rf[R_Z1]; geo_v2z<=rf[R_Z2]; geo_v3z<=rf[R_Z3];
                t1<=istl(rf[R_X2mX1],rf[R_Y2mY1]);
                t2<=istl(rf[R_X3mX2],rf[R_Y3mY2]);
                t3<= is_quad ? istl(rf[R_X4mX3],rf[R_Y4mY3]) : istl(rf[R_X1mX3],rf[R_Y1mY3]);
                t4<= is_quad ? istl(rf[R_X1mX4],rf[R_Y1mY4]) : 1'b1;
                state<=S_DONE;
            end
            S_DONE: begin done<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
            endcase
        end
    end
endmodule
