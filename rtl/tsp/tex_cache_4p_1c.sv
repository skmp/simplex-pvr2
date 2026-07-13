//
// tex_cache_4p_1c - 4-READ-PORT texture cache, DEEP-PIPELINED, TRUE 4-WAY ASSOCIATIVE,
// M10K-friendly variant. Backs the 4 bilinear corner fetchers.
//
// NOTE ON THE NAME: historically this was the "1-cycle reply" version. It is no longer
// 1-cycle - the lookup is now split into a multi-stage pipeline (see below) so every stage
// maps cleanly onto registered-read block RAM (M10K). The reply is a LATE ACK (variable
// latency); every downstream consumer (tex_fetch4_ob -> tex_unit -> tsp_shade_v2_pp) is
// already variable-latency-tolerant (payload rides THROUGH the fetch, drains on ack/valid
// pulses), so deepening the pipe needs no timing changes above tex_fetch4_ob's capture.
//
// ============================================================================
// WHY 16 PHYSICAL BANKS (a 4x4 grid). ---------------------------------------
// A block RAM (M10K) has ONE read port. To answer 4 corner lookups at once we need 4
// physical duplicates of the data (one per port) - that is the classic 4-copy scheme.
// To ALSO be 4-way ASSOCIATIVE (a port may be satisfied by data that lives in a sibling
// LOGICAL copy, defeating the direct-mapped alias-livelock) each of the 4 logical copies
// must itself be readable by all 4 ports in the same clock. Two ports wanting the SAME
// logical copy at DIFFERENT indexes is a genuine single-port bank conflict - physically
// impossible to serve in one clock from one RAM. The only conflict-free realization is to
// give each logical copy its OWN 4 physical duplicates: a 4x4 grid bank[c][p], c=logical
// copy 0..3, p=port 0..3. Port p reads physical instance bank[wcopy[p]][p] - ITS OWN
// column p - so 4 ports reading even the SAME logical copy hit 4 DIFFERENT physical RAMs.
// Zero conflict, no bubble, no arbitration.
//
// AREA: 16 data copies of the full 1024-line cache would be ~416 M10K (won't fit Cyclone
// V). So the cache is SHRUNK to NLINE=256 lines: ~7 M10K per 256b x 256 copy x 16 = ~112
// M10K for data - fits with room for the VQ$/plane$/tile buffers.
//
// ============================================================================
// PIPELINE (per accepted request):
//   ACCEPT (T0): creq[i].req && cresp[i].ready. Registers the 4 ports' request fields
//                (line/tag/wsel/valid) and drives the 16 META reads bmeta[c][p][ix[p]].
//   MATCH  (T1): the 16 registered tag/valid compares resolve, per port, WHICH logical
//                copy c holds its line (cmatch[p][c]) and whether it HIT anywhere
//                (hit[p]=|cmatch[p]). Picks wcopy[p]=lowest matching copy. Drives the 4
//                DATA reads bdata[wcopy[p]][p][ix[p]] (own column p -> conflict-free).
//   DATA   (T2): the 4 registered data words land; the word/lane mux + group-atomic ack
//                are driven THIS cycle (combinational off the registered read). A hit is
//                therefore 2 pipe stages after accept (ACCEPT->MATCH->DATA).
//   MISS   : any valid port that hit NOWHERE freezes the group and fills its line from
//                DDR (S_MISS/S_FILL), writing the line into ALL 16 banks (so every port's
//                own column becomes resident), then re-searches (S_RETEST). Group-atomic:
//                a hitting port does NOT ack while a sibling is still missing/filling.
//
// PROTOCOL per port i (unchanged contract, just later ack):
//   creq[i].req    : client wants to issue creq[i].waddr this cycle
//   cresp[i].ready : cache can accept it this cycle (LOW during a fill / reset sweep)
//   ACCEPTED       : creq[i].req && cresp[i].ready
//   cresp[i].ack   : a result is valid this cycle (combinational; group-atomic) - IN ORDER
//   cresp[i].rdata : the requested 64-bit word
// Line addr = waddr[28:2]; word-in-line = waddr[1:0]. index=line[IXW-1:0], tag=line[hi:IXW].
//
module tex_cache_4p_1c import tsp_pkg::*; (
    input                clk,
    input                reset,
    input  cache_req_t   creq  [0:3],
    output cache_resp_t  cresp [0:3],
    input                resp_take,      // consumer took this cycle's result (backpressure)
    output ddr_rd_req_t  dreq,
    input  ddr_rd_resp_t dresp
);
    localparam integer NLINE = 256;                 // SHRUNK (was 1024) so 16 copies fit
    localparam integer IXW   = 8;                    // log2(NLINE)
    localparam integer LAW   = 27;                   // line-addr width (waddr[28:2])
    localparam integer TAGW  = LAW - IXW;            // 19

    // ---- 4x4 grid of physical banks: bank[c][p], c=logical copy, p=port column ----
    //      Each holds full line DATA (256b) + META {valid, tag}. All 16 are written on a
    //      fill (identical contents per logical copy across its 4 columns), so port p's
    //      column read finds any resident line. genvar-generated so the 16 arrays get
    //      distinct names for the fitter.
    genvar gc, gp, gi;

    // data + meta storage, one array per (c,p). Declared via generate so each is a real
    // separate RAM. Access helpers below.
    // NB: SystemVerilog can't index a generate-scope array by variable at runtime from
    // outside, so we declare them as unpacked [4][4] arrays of memories directly.
    (* ramstyle = "M10K, no_rw_check" *) reg [255:0]  bdata [0:3][0:3][0:NLINE-1];
    (* ramstyle = "M10K, no_rw_check" *) reg [TAGW:0] bmeta [0:3][0:3][0:NLINE-1]; // {vld,tag}

    integer i, k, c, p;

    localparam S_RST=0, S_RUN=1, S_MISS=2, S_FILL=3, S_RETEST=4;
    reg [2:0] st;
    reg [IXW:0] rst_i;

    // A miss is detected at the MATCH stage (a valid port that hit in NO copy). While a miss
    // is being serviced (or during reset) the cache cannot accept, so ready is low.
    wire miss_now;                                   // = !fm[2]

    // ---- OUTPUT BACKPRESSURE. The D (DATA) stage is a 1-deep HOLDING stage: once a group
    //      lands there it asserts ack and HOLDS until the consumer pulses resp_take. While a
    //      group is held un-taken the WHOLE pipe freezes (d_stall): no new accept, no
    //      MATCH->D advance, and the META/DATA reads are frozen (rd_en low) so nothing is
    //      re-read/lost. This lets a consumer that can't take every cycle (e.g. the fetch's
    //      VQ 2nd-trip) stall the cache instead of dropping ack pulses. ----
    reg  d_v [0:3];                                  // (declared here; used by d_occupied)
    wire d_occupied = d_v[0] || d_v[1] || d_v[2] || d_v[3];
    wire d_stall    = d_occupied && !resp_take;      // D full and not being taken -> freeze

    wire accept = (st == S_RUN) && !miss_now && !d_stall;
    wire [3:0] acc;
    generate
      for (gp=0; gp<4; gp=gp+1) begin : ac
        assign acc[gp]           = accept && creq[gp].req;
        assign cresp[gp].ready   = accept;           // backpressure (same all ports)
      end
    endgenerate

    // ---- decode the incoming (accepted) request per port ----
    wire [LAW-1:0]  in_line[0:3];
    wire [IXW-1:0]  in_ix  [0:3];
    wire [TAGW-1:0] in_tag [0:3];
    wire [1:0]      in_wsel[0:3];
    generate
      for (gp=0; gp<4; gp=gp+1) begin : ind
        assign in_line[gp] = creq[gp].waddr[28:2];
        assign in_ix[gp]   = in_line[gp][IXW-1:0];
        assign in_tag[gp]  = in_line[gp][LAW-1:IXW];
        assign in_wsel[gp] = creq[gp].waddr[1:0];
      end
    endgenerate

    // ============================================================================
    // STAGE ACCEPT -> META READ ADDRESS. On a re-test after a fill we re-present the frozen
    // indexes instead of new requests.
    // ============================================================================
    reg  [IXW-1:0] retest_ix [0:3];
    reg            retesting;
    wire [IXW-1:0] meta_ix [0:3];                     // index driving META reads, per port
    generate
      for (gp=0; gp<4; gp=gp+1) begin : mix
        assign meta_ix[gp] = retesting ? retest_ix[gp] : in_ix[gp];
      end
    endgenerate

    // ---- T1 (MATCH) request register: the request whose META read is arriving this cycle.
    reg            t_v   [0:3];
    reg [LAW-1:0]  t_line[0:3];
    reg [TAGW-1:0] t_tag [0:3];
    reg [1:0]      t_wsel[0:3];
    reg [IXW-1:0]  t_ix  [0:3];

    // ============ STAGE 1: 16 META reads (registered). bmeta[c][p][meta_ix[p]]. ============
    // Port p reads its OWN column p across all 4 logical copies c.
    reg [TAGW:0] rmeta [0:3][0:3];                    // rmeta[c][p]
    // Reads advance the pipe; FREEZE them when the output is back-pressured (d_stall) so
    // rmeta/rdat hold in step with the frozen t_*/d_* registers.
    wire         rd_en = ((st == S_RUN) || retesting) && !d_stall;
    always @(posedge clk) if (rd_en) begin
        for (p=0; p<4; p=p+1) begin
            rmeta[0][p] <= bmeta[0][p][meta_ix[p]];
            rmeta[1][p] <= bmeta[1][p][meta_ix[p]];
            rmeta[2][p] <= bmeta[2][p][meta_ix[p]];
            rmeta[3][p] <= bmeta[3][p][meta_ix[p]];
        end
    end

    // ============ STAGE 2 (MATCH combinational): which copy hit, per port ============
    // cmatch[p][c] : logical copy c holds port p's line (valid & tag match). hit = any.
    wire [3:0]  cmatch[0:3];
    reg  [1:0]  wcopy [0:3];                          // chosen copy (lowest match) per port
    wire        t_hit [0:3];
    generate
      for (gp=0; gp<4; gp=gp+1) begin : mt
        assign cmatch[gp][0] = t_v[gp] && rmeta[0][gp][TAGW] && (rmeta[0][gp][TAGW-1:0]==t_tag[gp]);
        assign cmatch[gp][1] = t_v[gp] && rmeta[1][gp][TAGW] && (rmeta[1][gp][TAGW-1:0]==t_tag[gp]);
        assign cmatch[gp][2] = t_v[gp] && rmeta[2][gp][TAGW] && (rmeta[2][gp][TAGW-1:0]==t_tag[gp]);
        assign cmatch[gp][3] = t_v[gp] && rmeta[3][gp][TAGW] && (rmeta[3][gp][TAGW-1:0]==t_tag[gp]);
        assign t_hit[gp]     = |cmatch[gp];
      end
    endgenerate
    always @(*) begin
        for (int q=0; q<4; q=q+1)
            wcopy[q] = cmatch[q][0] ? 2'd0 :
                       cmatch[q][1] ? 2'd1 :
                       cmatch[q][2] ? 2'd2 : 2'd3;
    end

    // lowest-index MATCH-stage port that MISSED (valid, hit nowhere). fm[2]=1 => none.
    wire [2:0] fm = (t_v[0] && !t_hit[0]) ? 3'd0 :
                    (t_v[1] && !t_hit[1]) ? 3'd1 :
                    (t_v[2] && !t_hit[2]) ? 3'd2 :
                    (t_v[3] && !t_hit[3]) ? 3'd3 : 3'b100;
    assign miss_now = !fm[2];

    // ============ STAGE 2 -> 3: DATA read address. Port p reads bdata[wcopy[p]][p][t_ix[p]]
    //             - its OWN column p of the chosen logical copy. Conflict-free (each port a
    //             different physical column). Only issued for a HIT with the WHOLE group
    //             resident; on a miss the group freezes instead. ============
    // D-stage request register: mirrors the MATCH-stage request one cycle later, plus the
    // resolved winning copy per port. (d_v declared up top for the d_stall computation.)
    reg [1:0]      d_wsel [0:3];
    reg [1:0]      d_wcopy[0:3];
    // data read happens combinationally-addressed / registered-out below, so the D-stage
    // just needs the index + which copy. It re-uses t_ix latched forward.
    reg [IXW-1:0]  d_ix   [0:3];

    // ============ STAGE 3: 4 DATA reads (registered). bdata[d_wcopy[p]][p][d_ix[p]]. ======
    reg [255:0] rdat [0:3];                           // one resolved data word per port
    // group_ready (below) qualifies advancing MATCH->DATA; only then do the data reads and
    // the request advances. The read address uses the MATCH-stage winners (wcopy/t_ix) so
    // the data lands aligned with the D-stage request register.
    wire group_ready = (st == S_RUN) && !miss_now;    // ALL valid ports resident at MATCH
    always @(posedge clk) if (rd_en) begin
        // address with the MATCH-stage winners (advancing into D next cycle).
        rdat[0] <= bdata[wcopy[0]][0][t_ix[0]];
        rdat[1] <= bdata[wcopy[1]][1][t_ix[1]];
        rdat[2] <= bdata[wcopy[2]][2][t_ix[2]];
        rdat[3] <= bdata[wcopy[3]][3][t_ix[3]];
    end

    // ============ STAGE 3 (DATA, combinational output): word/lane mux + group-atomic ack ==
    wire [63:0] d_word[0:3];
    generate
      for (gp=0; gp<4; gp=gp+1) begin : dw
        assign d_word[gp] = rdat[gp][64*d_wsel[gp] +: 64];
      end
    endgenerate
    // The 4 corners are one bilinear sample: serve the GROUP atomically. A hitting port does
    // not ack while a sibling is missing (that case froze at MATCH and never advanced to D).
    // So a request that REACHED the D stage is, by construction, an all-hit group.
    generate
      for (gp=0; gp<4; gp=gp+1) begin : od
        assign cresp[gp].ack   = d_v[gp];
        assign cresp[gp].rdata = d_word[gp];
      end
    endgenerate

    reg        rd_r;   reg [28:0] addr_r; reg [7:0] burst_r;
    assign dreq.rd    = rd_r;
    assign dreq.addr  = addr_r;
    assign dreq.burst = burst_r;

    // fill bookkeeping
    reg [LAW-1:0]  m_line; reg [IXW-1:0] m_ix; reg [TAGW-1:0] m_tag;
    reg [1:0]      m_beat; reg [255:0] m_acc;
    reg [1:0]      m_copy;               // LOGICAL copy to fill = the missing port index (fm)
    wire [28:0] m_base = {m_line, 2'b00};

`ifndef SYNTHESIS
    integer stat_hit [0:4];
    integer stat_n;
`endif

    always @(posedge clk) begin
        if (reset) begin
            st <= S_RST; rd_r <= 0; rst_i <= 0; retesting <= 0;
            for (i=0;i<4;i=i+1) begin t_v[i]<=0; d_v[i]<=0; end
`ifndef SYNTHESIS
            for (i=0;i<5;i=i+1) stat_hit[i] <= 0;
            stat_n <= 0;
`endif
        end else begin
            rd_r <= 1'b0;
            retesting <= 1'b0;

            case (st)
            // clear valid bits one entry/cycle after reset (all 16 banks' meta).
            S_RST: begin
                for (c=0;c<4;c=c+1) for (p=0;p<4;p=p+1)
                    bmeta[c][p][rst_i[IXW-1:0]][TAGW] <= 1'b0;
                for (i=0;i<4;i=i+1) begin t_v[i] <= 1'b0; d_v[i] <= 1'b0; end
                if (rst_i == NLINE-1) st <= S_RUN;
                else rst_i <= rst_i + 1'b1;
            end

            // steady state:
            //   * MATCH-test the request whose META read arrived this cycle (t_*). If the
            //     whole group hit, ADVANCE it into the D stage (its data read is issued this
            //     cycle, lands next cycle -> ack at D). On the first miss, FREEZE + fill.
            //   * Simultaneously ACCEPT a new request per port into t_* for next cycle's
            //     MATCH (only when the group advanced, i.e. no miss).
            // Under output back-pressure (d_stall) the ENTIRE steady-state advance freezes:
            // D holds its un-taken group (ack stays high), t_*/rmeta/rdat hold (rd_en low),
            // and no new request is accepted. The consumer releases by pulsing resp_take.
            S_RUN: if (!d_stall) begin
`ifndef SYNTHESIS
                if (t_v[0] || t_v[1] || t_v[2] || t_v[3]) begin
                    stat_hit[(t_hit[0]?1:0)+(t_hit[1]?1:0)
                            +(t_hit[2]?1:0)+(t_hit[3]?1:0)]
                        <= stat_hit[(t_hit[0]?1:0)+(t_hit[1]?1:0)
                                   +(t_hit[2]?1:0)+(t_hit[3]?1:0)] + 1;
                    stat_n <= stat_n + 1;
                end
`endif
                if (!fm[2]) begin
                    // ---- MISS in the group: latch the lowest missing line and go fill.
                    //      FREEZE the whole group's t_* (including hitters) so all 4 ack
                    //      together once resident. The D stage produces NO ack this cycle
                    //      (the group didn't advance) -> clear d_v. Each frozen port re-
                    //      presents its index (retest_ix) and re-tests after the fill.
                    m_line <= t_line[fm[1:0]];
                    m_ix   <= t_line[fm[1:0]][IXW-1:0];
                    m_tag  <= t_line[fm[1:0]][LAW-1:IXW];
                    m_copy <= fm[1:0];               // fill into LOGICAL copy = missing port
                    m_beat <= 2'd0;
                    for (k=0;k<4;k=k+1) begin
                        retest_ix[k] <= t_line[k][IXW-1:0];   // keep ALL t_v (group waits)
                        d_v[k]       <= 1'b0;                 // no D-stage ack while filling
                    end
                    st <= S_MISS;
                end else begin
                    // ---- all-hit (or empty): advance the group into D and accept anew.
                    for (k=0;k<4;k=k+1) begin
                        // MATCH -> DATA advance: the data read for these winners was issued
                        // by the rd_en block THIS cycle; it lands next cycle aligned with d_*.
                        d_v[k]     <= t_v[k];
                        d_wsel[k]  <= t_wsel[k];
                        d_wcopy[k] <= wcopy[k];
                        d_ix[k]    <= t_ix[k];
                        // ACCEPT -> MATCH: new request per port for next cycle's MATCH.
                        t_v[k]    <= acc[k];
                        t_line[k] <= in_line[k];
                        t_tag[k]  <= in_tag[k];
                        t_wsel[k] <= in_wsel[k];
                        t_ix[k]   <= in_ix[k];
                    end
                end
            end

            // burst-read the 4 words of the missing line.
            S_MISS: if (!dresp.busy) begin
                rd_r    <= 1'b1;
                addr_r  <= {4'b0011, m_base[24:0]};
                burst_r <= 8'd4;
                st      <= S_FILL;
            end
            S_FILL: if (dresp.dready) begin
                m_acc[64*m_beat +: 64] <= dresp.dout;
                if (m_beat == 2'd3) begin
                    // write the filled line into ONE LOGICAL COPY (= m_copy, the missing port
                    // index), across all 4 COLUMNS of that copy so any port's own-column read
                    // of copy m_copy sees it. Filling a single logical copy - NOT all copies -
                    // is what lets ALIASING lines (same index, different tag, on different
                    // corners) COEXIST: corner 0's line goes to copy 0, corner 3's to copy 3,
                    // etc., so a fill never evicts a sibling corner's resident line. The
                    // associative read (bdata[wcopy[p]][p]) then finds each line in its copy.
                    for (p=0;p<4;p=p+1) begin
                        bdata[m_copy][p][m_ix] <= { dresp.dout, m_acc[191:0] };
                        bmeta[m_copy][p][m_ix] <= { 1'b1, m_tag };
                    end
                    // reload the frozen reads from the (now updated) store, re-test next cyc.
                    retesting <= 1'b1;
                    st <= S_RETEST;
                end else m_beat <= m_beat + 2'd1;
            end
            // one cycle for the re-presented META reads to land, then S_RUN re-tests t_*. A
            // frozen port whose line == the just-filled line now hits; a port to a different
            // still-missing line misses again -> another fill.
            S_RETEST: st <= S_RUN;
            default: st <= S_RUN;
            endcase
        end
    end

`ifndef SYNTHESIS
    // ---- LIVELOCK detector: fills that never resolve the group ----
    // Alias livelock is structurally prevented: each fill writes ONE logical copy (= the
    // missing port index), so corner c's lines live in copy c and a fill NEVER evicts a
    // sibling corner's resident line. The associative read (bdata[wcopy[p]][p]) finds each
    // line in whichever copy holds it. This detector remains a safety net for regressions -
    // dozens of back-to-back fills with no group_ready still means something is wrong (e.g.
    // two aliasing lines forced onto the SAME corner over successive requests, which is a
    // genuine capacity miss, not a livelock).
    integer tc_fills; reg tc_reported;
    always @(posedge clk) begin
        if (reset) begin tc_fills <= 0; tc_reported <= 1'b0; end
        else begin
            if (group_ready)             tc_fills <= 0;
            else if (st==S_MISS && !dresp.busy) tc_fills <= tc_fills + 1;
            if (tc_fills > 64 && !tc_reported) begin
                tc_reported <= 1'b1;
                $display("\n$$$$$$ TEX$ LIVELOCK %m (%0d fills, no group_ready) $$$$$$", tc_fills);
                $display("  filling line=%08x (index=%0d tag=%0d) via port %0d", m_line, m_ix, m_tag, fm[1:0]);
                for (k=0;k<4;k=k+1)
                    $display("  port%0d: v=%0d line=%08x  index=%0d tag=%0d  hit=%0d",
                             k, t_v[k], t_line[k], t_line[k][IXW-1:0], t_line[k][LAW-1:IXW], t_hit[k]);
                $display("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
            end
        end
    end

    final begin
        $display("=== TEX$1c %m: %0d lookup-cycles: HIT4=%0d HIT3=%0d HIT2=%0d HIT1=%0d HIT0=%0d ===",
                 stat_n, stat_hit[4], stat_hit[3], stat_hit[2], stat_hit[1], stat_hit[0]);
    end
`endif
endmodule
