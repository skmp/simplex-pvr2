//
// region_array_parser - walks the PVR REGION ARRAY (refsw RenderCORE /
// ReadRegionArrayEntry, refsw_lists.cpp) and, for each tile (region entry),
// strobes the tile's enabled render STATES one at a time, in order:
//     clear -> op -> pt -> tr -> flush
// Only enabled states are strobed; a tile with no enabled states is silently
// skipped. Each state is presented via state_ready (LEVEL) with tile x/y, a
// one-hot `state`, and the list base pointer (op/pt/tr only); the consumer
// pulses list_done when finished, and the parser advances.
//
// Region entry layout (24 bytes v2 / 20 bytes v1; refsw ReadRegionArrayEntry):
//   +0  control : res0[1:0] tilex[7:2] tiley[13:8] res1[27:14]
//                 no_writeout[28] pre_sort[29] z_keep[30] last_region[31]
//   +4  opaque      ListPointer : ptr_in_words[23:2], empty[31]
//   +8  opaque_mod  ListPointer   (modvol - ignored for now)
//   +12 trans       ListPointer
//   +16 trans_mod   ListPointer   (modvol - ignored for now)
//   +20 puncht      ListPointer   (v2 only; v1 forces puncht empty, stride=20)
// List byte address = ptr_in_words * 4.
//
// State enables (refsw RenderCORE):
//   clear = !control.z_keep      op = !opaque.empty   pt = !puncht.empty
//   tr    = !trans.empty         flush = !control.no_writeout
//
// Termination: control.last_region set on the just-read entry, OR after 16384
// tiles read (runaway guard). tiles_parsed pulses when done.
//
// DI: the VRAM read port is the INJECTED 256-bit data cache (creq/cresp).
//
module region_array_parser import tsp_pkg::*; (
    input                  clk,
    input                  reset,
    input                  start,          // 1-cycle: begin at region_base
    input      [26:0]      region_base,    // byte addr of the region array
    input                  region_v1,      // 1: 20-byte entries, puncht forced empty
    output reg             busy,
    output reg             tiles_parsed,   // 1-cycle: whole region array consumed

    output region_out_t    rout,
    input  region_ack_t    ack,

    // injected 256-bit data cache (region-array region)
    output cache_req256_t  creq,
    input  cache_resp256_t cresp
);
    // ---- 32-bit word reader over the 256-bit line cache ----
    reg  [26:0] raddr; reg rd_go; reg [31:0] rword; reg rword_v; reg [2:0] rsel;
    reg  [26:0] creq_laddr_r; reg creq_req_r;
    assign creq.req   = creq_req_r;
    assign creq.laddr = creq_laddr_r;

    always @(posedge clk) begin
        rword_v    <= 1'b0;
        creq_req_r <= 1'b0;                // 1-cycle pulse
        if (rd_go) begin
            creq_req_r   <= 1'b1;
            creq_laddr_r <= raddr[26:5];
            rsel         <= raddr[4:2];
        end
        if (cresp.ack) begin rword <= cresp.rdata[32*rsel +: 32]; rword_v <= 1'b1; end
        if (reset) creq_req_r <= 1'b0;
    end

    // ---- output regs ----
    reg          o_ready_r;
    reg [5:0]    o_tx_r, o_ty_r;
    reg [4:0]    o_state_r;
    reg [26:0]   o_ptr_r;
    assign rout.list_ready  = o_ready_r;
    assign rout.tile_x      = o_tx_r;
    assign rout.tile_y      = o_ty_r;
    assign rout.state       = o_state_r;
    assign rout.list_ptr    = o_ptr_r;

    // ---- entry storage ----
    reg [26:0] base;             // byte addr of current entry
    reg [13:0] tiles_seen;       // runaway guard (>=16384)
    reg [31:0] ctrl;             // control word
    reg [21:0] op_ptr, pt_ptr, tr_ptr;    // ptr_in_words per list
    reg        op_en, pt_en, tr_en, clear_en, flush_en, last_r;

    // derived tile coords
    wire [5:0] tilex = ctrl[7:2];
    wire [5:0] tiley = ctrl[13:8];

    localparam S_IDLE=4'd0,
               S_CTRL=4'd1, S_CTRLW=4'd2,   // read control word
               S_OPW=4'd3,                  // wait opaque ptr
               S_TRW=4'd4,                  // wait trans ptr
               S_PTW=4'd5,                  // wait puncht ptr
               S_SEEK=4'd6,                 // pick next enabled phase; load output
               S_EMIT=4'd7,                 // strobe the state, wait list_done
               S_NEXT=4'd8,                 // advance to next entry / finish
               S_DONE=4'd9;
    reg [3:0] st;
    reg [2:0] phase;   // phase to try next: 0=clear 1=op 2=pt 3=tr 4=flush, 5=none

    // is phase p enabled?
    function automatic phase_en(input [2:0] p);
        case (p)
        3'd0: phase_en = clear_en;
        3'd1: phase_en = op_en;
        3'd2: phase_en = pt_en;
        3'd3: phase_en = tr_en;
        3'd4: phase_en = flush_en;
        default: phase_en = 1'b0;
        endcase
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            st<=S_IDLE; busy<=0; tiles_parsed<=0; o_ready_r<=0; rd_go<=0; tiles_seen<=0;
        end else begin
            tiles_parsed <= 1'b0;
            rd_go        <= 1'b0;

            case (st)
            S_IDLE: if (start) begin base<=region_base; busy<=1; tiles_seen<=0; st<=S_CTRL; end

            // read control(+0), opaque(+4), trans(+12), puncht(+20). modvols
            // (opaque_mod +8, trans_mod +16) are skipped (address math only).
            S_CTRL:  begin raddr<=base; rd_go<=1'b1; st<=S_CTRLW; end
            S_CTRLW: if (rword_v) begin
                        ctrl<=rword;
                        clear_en <= ~rword[30];    // !z_keep
                        flush_en <= ~rword[28];    // !no_writeout
                        last_r   <=  rword[31];     // last_region
                        raddr<=base+27'd4; rd_go<=1'b1; st<=S_OPW;
                    end
            S_OPW: if (rword_v) begin
                        op_ptr<=rword[23:2]; op_en<=~rword[31];
                        raddr<=base+27'd12; rd_go<=1'b1; st<=S_TRW;
                    end
            S_TRW: if (rword_v) begin
                        tr_ptr<=rword[23:2]; tr_en<=~rword[31];
                        if (region_v1) begin
                            pt_ptr<=22'd0; pt_en<=1'b0; phase<=3'd0; st<=S_SEEK;
                        end else begin
                            raddr<=base+27'd20; rd_go<=1'b1; st<=S_PTW;
                        end
                    end
            S_PTW: if (rword_v) begin
                        pt_ptr<=rword[23:2]; pt_en<=~rword[31];
                        phase<=3'd0; st<=S_SEEK;
                    end

            // seek to the next enabled phase (>= phase); load the output regs.
            S_SEEK: begin
                if (phase >= 3'd5) st <= S_NEXT;            // no more enabled states
                else if (!phase_en(phase)) phase <= phase + 3'd1;
                else begin
                    o_tx_r <= tilex; o_ty_r <= tiley;
                    case (phase)
                    3'd0: begin o_state_r<=RSTATE_CLEAR; o_ptr_r<=27'd0;             end
                    3'd1: begin o_state_r<=RSTATE_OP;    o_ptr_r<={op_ptr,2'b00};    end
                    3'd2: begin o_state_r<=RSTATE_PT;    o_ptr_r<={pt_ptr,2'b00};    end
                    3'd3: begin o_state_r<=RSTATE_TR;    o_ptr_r<={tr_ptr,2'b00};    end
                    default: begin o_state_r<=RSTATE_FLUSH; o_ptr_r<=27'd0;          end
                    endcase
                    st <= S_EMIT;
                end
            end

            S_EMIT: begin
                o_ready_r <= 1'b1;
                if (ack.list_done) begin
                    o_ready_r <= 1'b0;
                    phase <= phase + 3'd1;    // try the next phase
                    st    <= S_SEEK;
                end
            end

            S_NEXT: begin
                tiles_seen <= tiles_seen + 14'd1;
                if (last_r || tiles_seen == 14'h3FFF) st <= S_DONE;
                else begin base <= base + (region_v1 ? 27'd20 : 27'd24); st <= S_CTRL; end
            end

            S_DONE: begin busy<=0; tiles_parsed<=1'b1; st<=S_IDLE; end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
