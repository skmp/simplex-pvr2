//
// tsp_setup_seq - clocked TSP triangle setup using ONE shared mul+add
//                 (fmac_seq). Reuses recip_C from isp_setup_seq, so there is
//                 NO second reciprocal in the design.
//
// Computes the 10 interpolation planes (U, V, Col[0..3], Ofs[0..3]) for the
// single-volume case, each as a PlaneStepper solve on perspective-scaled
// attributes (attr*invW). Same numerics as the combinational tsp_setup, but the
// ddx/ddy use recip_C * (-Aa) (the C optimization) instead of a divide.
//
// Sequencing: phase 1 multiplies every attribute by its vertex invW (30 MACs).
// Phase 2 runs the plane solver 10 times; the solver itself is a fixed 8-op
// micro-sequence (DA2,DA3,Aa(2),Ba(2),ddx,ddy is 7; plus c is 2 more = ... see
// PLANE_OPS) over one MAC. Per-plane attribute triples are addressed by index.
//
// Colour inputs arrive as floats (u8->float done in the wrapper). Gouraud=1
// uses per-vertex colour; flat uses vertex-3 colour for all three (selected
// when the attribute products are formed in phase 1).
//
module tsp_setup_seq (
    input         clk,
    input         reset,
    input         start,
    output reg    done,

    input         gouraud,

    // shared geometry from isp_setup_seq
    input  [31:0] dx12, input [31:0] dx13, input [31:0] dy12, input [31:0] dy13,
    input  [31:0] recipC,
    input  [31:0] v1x_l, input [31:0] v1y_t,
    input  [31:0] v1z, input [31:0] v2z, input [31:0] v3z,

    input  [31:0] v1u, input [31:0] v1v,
    input  [31:0] v2u, input [31:0] v2v,
    input  [31:0] v3u, input [31:0] v3v,

    input  [31:0] v1col0,input [31:0] v1col1,input [31:0] v1col2,input [31:0] v1col3,
    input  [31:0] v2col0,input [31:0] v2col1,input [31:0] v2col2,input [31:0] v2col3,
    input  [31:0] v3col0,input [31:0] v3col1,input [31:0] v3col2,input [31:0] v3col3,
    input  [31:0] v1spc0,input [31:0] v1spc1,input [31:0] v1spc2,input [31:0] v1spc3,
    input  [31:0] v2spc0,input [31:0] v2spc1,input [31:0] v2spc2,input [31:0] v2spc3,
    input  [31:0] v3spc0,input [31:0] v3spc1,input [31:0] v3spc2,input [31:0] v3spc3,

    // 10 planes x (ddx,ddy,c). idx: 0=U 1=V 2..5=Col 6..9=Ofs
    output reg [31:0] u_ddx,output reg [31:0] u_ddy,output reg [31:0] u_c,
    output reg [31:0] v_ddx,output reg [31:0] v_ddy,output reg [31:0] v_c,
    output reg [31:0] col0_ddx,output reg [31:0] col0_ddy,output reg [31:0] col0_c,
    output reg [31:0] col1_ddx,output reg [31:0] col1_ddy,output reg [31:0] col1_c,
    output reg [31:0] col2_ddx,output reg [31:0] col2_ddy,output reg [31:0] col2_c,
    output reg [31:0] col3_ddx,output reg [31:0] col3_ddy,output reg [31:0] col3_c,
    output reg [31:0] ofs0_ddx,output reg [31:0] ofs0_ddy,output reg [31:0] ofs0_c,
    output reg [31:0] ofs1_ddx,output reg [31:0] ofs1_ddy,output reg [31:0] ofs1_c,
    output reg [31:0] ofs2_ddx,output reg [31:0] ofs2_ddy,output reg [31:0] ofs2_c,
    output reg [31:0] ofs3_ddx,output reg [31:0] ofs3_ddy,output reg [31:0] ofs3_c
);
    localparam [31:0] ONE=32'h3f800000, ZERO=32'h0;

    // ---------------- scratchpad ----------------
    // Layout:
    //   0  ZERO, 1 ONE
    //   2..4   geo: dx12,dx13,dy12,dy13 (4)  -> 2,3,4,5
    //   recipC, v1x_l, v1y_t           -> 6,7,8
    //   z1,z2,z3                        -> 9,10,11
    //   ATT base 12: 10 planes x 3 verts = 30 -> 12..41  (attr*z products)
    //   plane temps 42..51
    localparam R_ZERO=0,R_ONE=1,R_DX12=2,R_DX13=3,R_DY12=4,R_DY13=5,
               R_RECIPC=6,R_V1XL=7,R_V1YT=8,R_Z1=9,R_Z2=10,R_Z3=11;
    localparam ATT=12;     // ATT + plane*3 + vert
    localparam R_DA2=42,R_DA3=43,R_AA0=44,R_AA=45,R_BA0=46,R_BA=47,
               R_DDX=48,R_DDY=49,R_C0=50,R_CC=51;
    localparam RF_N=64, RF_AW=6;
    reg [31:0] rf [0:RF_N-1];

    function [RF_AW-1:0] attidx(input integer plane, input integer vert);
        attidx = ATT + plane*3 + vert; endfunction

    // raw (pre-z) attribute source for (plane,vert), honoring gouraud/flat.
    // vert: 0,1,2.  For flat (gouraud=0) colour/offset use vertex-3 source for
    // verts 0,1 (vertex2 is itself vertex-3 source already).
    reg [31:0] att_raw;   // combinational selection (below)
    reg [31:0] att_z;

    // ---------------- one MAC ----------------
    reg req; reg [31:0] opa,opb,opc; reg osub,oneg;
    wire ack; wire [31:0] qres;
    fmac_seq u_mac (.clk(clk),.reset(reset),.req(req),.a(opa),.b(opb),.c(opc),
                    .sub(osub),.neg_p(oneg),.ack(ack),.q(qres));

    // ---------------- plane solver micro-program ----------------
    // operates on rf[a1]=att(p,0), a2=att(p,1), a3=att(p,2) at ATT+p*3+{0,1,2}
    // 10 ops; results ddx/ddy/c in R_DDX/R_DDY/R_CC.
    localparam POPS=10;
    // we index attribute regs dynamically by current plane, so the ROM stores
    // symbolic codes for the three attribute slots; resolved at issue.
    // Attribute slots use sentinel codes (61..63) that do NOT collide with any
    // real rf index (which run 0..51); res() maps them to the current plane's
    // attribute triple. All other codes are literal rf indices passed through.
    localparam C_A1=61,C_A2=62,C_A3=63,
               C_DA2=R_DA2,C_DA3=R_DA3,C_AA0=R_AA0,C_AA=R_AA,C_BA0=R_BA0,C_BA=R_BA,
               C_DDX=R_DDX,C_DDY=R_DDY,C_C0=R_C0,C_CC=R_CC,
               C_ZERO=R_ZERO,C_ONE=R_ONE,C_RECIPC=R_RECIPC,
               C_DX12=R_DX12,C_DX13=R_DX13,C_DY12=R_DY12,C_DY13=R_DY13,
               C_V1XL=R_V1XL,C_V1YT=R_V1YT;
    reg [RF_AW-1:0] p_d[0:POPS-1],p_a[0:POPS-1],p_b[0:POPS-1],p_c[0:POPS-1];
    reg             p_s[0:POPS-1],p_n[0:POPS-1];
    task PSET(input integer i, input [RF_AW-1:0] d,a,b,c, input s,n);
        begin p_d[i]=d;p_a[i]=a;p_b[i]=b;p_c[i]=c;p_s[i]=s;p_n[i]=n; end endtask
    initial begin
        // da2=a2-a1, da3=a3-a1
        PSET(0,C_DA2,C_A2,C_ONE,C_A1,1,0);
        PSET(1,C_DA3,C_A3,C_ONE,C_A1,1,0);
        // Aa=da3*dy12 - da2*dy13
        PSET(2,C_AA0,C_DA2,C_DY13,C_ZERO,0,0);
        PSET(3,C_AA, C_DA3,C_DY12,C_AA0,1,0);
        // Ba=dx13*da2 - dx12*da3
        PSET(4,C_BA0,C_DX12,C_DA3,C_ZERO,0,0);
        PSET(5,C_BA, C_DX13,C_DA2,C_BA0,1,0);
        // ddx=-Aa*recipC, ddy=-Ba*recipC
        PSET(6,C_DDX,C_AA,C_RECIPC,C_ZERO,0,1);
        PSET(7,C_DDY,C_BA,C_RECIPC,C_ZERO,0,1);
        // c = a1 - ddx*v1xl - ddy*v1yt
        PSET(8,C_C0, C_DDX,C_V1XL,C_A1,0,1);
        PSET(9,C_CC, C_DDY,C_V1YT,C_C0,0,1);
    end

    // resolve a program code to an actual rf index for the current plane
    function [RF_AW-1:0] res(input [RF_AW-1:0] code, input integer plane);
        case (code)
          C_A1: res=attidx(plane,0);
          C_A2: res=attidx(plane,1);
          C_A3: res=attidx(plane,2);
          default: res=code;
        endcase
    endfunction

    // ---------------- attribute source mux (phase 1) ----------------
    // For (plane p, vert v) pick the raw float source. planes:
    //  0=U 1=V 2=Col0 3=Col1 4=Col2 5=Col3 6=Ofs0 7=Ofs1 8=Ofs2 9=Ofs3
    reg [3:0] ph1_p; reg [1:0] ph1_v;
    always @(*) begin
        att_raw = ZERO;
        case (ph1_p)
          0: att_raw = (ph1_v==0)?v1u:(ph1_v==1)?v2u:v3u;
          1: att_raw = (ph1_v==0)?v1v:(ph1_v==1)?v2v:v3v;
          2: att_raw = (ph1_v==2)?v3col0 : gouraud?((ph1_v==0)?v1col0:v2col0):v3col0;
          3: att_raw = (ph1_v==2)?v3col1 : gouraud?((ph1_v==0)?v1col1:v2col1):v3col1;
          4: att_raw = (ph1_v==2)?v3col2 : gouraud?((ph1_v==0)?v1col2:v2col2):v3col2;
          5: att_raw = (ph1_v==2)?v3col3 : gouraud?((ph1_v==0)?v1col3:v2col3):v3col3;
          6: att_raw = (ph1_v==2)?v3spc0 : gouraud?((ph1_v==0)?v1spc0:v2spc0):v3spc0;
          7: att_raw = (ph1_v==2)?v3spc1 : gouraud?((ph1_v==0)?v1spc1:v2spc1):v3spc1;
          8: att_raw = (ph1_v==2)?v3spc2 : gouraud?((ph1_v==0)?v1spc2:v2spc2):v3spc2;
          9: att_raw = (ph1_v==2)?v3spc3 : gouraud?((ph1_v==0)?v1spc3:v2spc3):v3spc3;
          default: att_raw = ZERO;
        endcase
    end
    wire [31:0] vz_for_v = (ph1_v==0)?rf[R_Z1]:(ph1_v==1)?rf[R_Z2]:rf[R_Z3];

    // ---------------- FSM ----------------
    localparam S_IDLE=0,S_LOAD=1,S_P1=2,S_P2=3,S_DONE=4;
    reg [2:0] state;
    reg [3:0] plane;       // 0..9
    reg [5:0] ph1_i;       // 0..29
    reg [3:0] pop;         // plane op index 0..9
    reg       issued;

    // Capture at the moment the last plane op acks. ddx/ddy were written to rf
    // several cycles earlier (stable); the c value is the just-acked qres (its
    // rf write is non-blocking and not yet visible), so pass it in as cval.
    task captplane(input [3:0] p, input [31:0] cval);
        case (p)
          0: begin u_ddx<=rf[R_DDX]; u_ddy<=rf[R_DDY]; u_c<=cval; end
          1: begin v_ddx<=rf[R_DDX]; v_ddy<=rf[R_DDY]; v_c<=cval; end
          2: begin col0_ddx<=rf[R_DDX]; col0_ddy<=rf[R_DDY]; col0_c<=cval; end
          3: begin col1_ddx<=rf[R_DDX]; col1_ddy<=rf[R_DDY]; col1_c<=cval; end
          4: begin col2_ddx<=rf[R_DDX]; col2_ddy<=rf[R_DDY]; col2_c<=cval; end
          5: begin col3_ddx<=rf[R_DDX]; col3_ddy<=rf[R_DDY]; col3_c<=cval; end
          6: begin ofs0_ddx<=rf[R_DDX]; ofs0_ddy<=rf[R_DDY]; ofs0_c<=cval; end
          7: begin ofs1_ddx<=rf[R_DDX]; ofs1_ddy<=rf[R_DDY]; ofs1_c<=cval; end
          8: begin ofs2_ddx<=rf[R_DDX]; ofs2_ddy<=rf[R_DDY]; ofs2_c<=cval; end
          9: begin ofs3_ddx<=rf[R_DDX]; ofs3_ddy<=rf[R_DDY]; ofs3_c<=cval; end
        endcase
    endtask

    always @(posedge clk) begin
        if (reset) begin state<=S_IDLE; done<=0; req<=0; issued<=0; end
        else begin
            done<=0; req<=0;
            case (state)
            S_IDLE: if (start) state<=S_LOAD;
            S_LOAD: begin
                rf[R_ZERO]<=ZERO; rf[R_ONE]<=ONE;
                rf[R_DX12]<=dx12; rf[R_DX13]<=dx13; rf[R_DY12]<=dy12; rf[R_DY13]<=dy13;
                rf[R_RECIPC]<=recipC; rf[R_V1XL]<=v1x_l; rf[R_V1YT]<=v1y_t;
                rf[R_Z1]<=v1z; rf[R_Z2]<=v2z; rf[R_Z3]<=v3z;
                ph1_i<=0; ph1_p<=0; ph1_v<=0; issued<=0; state<=S_P1;
            end
            // phase 1: att_z = att_raw * vz  (30 products)
            S_P1: begin
                if (!issued) begin
                    req<=1; issued<=1; osub<=0; oneg<=0; opc<=ZERO;
                    opa<=att_raw; opb<=vz_for_v;
                end else if (ack) begin
                    rf[ATT + ph1_p*3 + ph1_v] <= qres;
                    issued<=0;
                    if (ph1_i==29) begin plane<=0; pop<=0; state<=S_P2; end
                    else begin
                        ph1_i<=ph1_i+1;
                        if (ph1_v==2) begin ph1_v<=0; ph1_p<=ph1_p+1; end
                        else ph1_v<=ph1_v+1;
                    end
                end
            end
            // phase 2: run plane solver per plane
            S_P2: begin
                if (!issued) begin
                    req<=1; issued<=1;
                    opa<=rf[res(p_a[pop],plane)];
                    opb<=rf[res(p_b[pop],plane)];
                    opc<=rf[res(p_c[pop],plane)];
                    osub<=p_s[pop]; oneg<=p_n[pop];
                end else if (ack) begin
                    rf[res(p_d[pop],plane)] <= qres;
                    issued<=0;
                    if (pop==POPS-1) begin
                        captplane(plane, qres);
                        if (plane==9) state<=S_DONE;
                        else begin plane<=plane+1; pop<=0; end
                    end else pop<=pop+1;
                end
            end
            S_DONE: begin done<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
            endcase
        end
    end
endmodule
