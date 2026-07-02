//
// object_list_parser - pure list walker. Reads ONLY the object-list region of
// VRAM (never the parameter buffer) and presents ONE OBJECT-LIST ENTRY at a
// time: its type (strip / tri array / quad array) and decoded fields
// (param_offs_in_words, skip, shadow, mask, count). It does NOT iterate strip
// triangles or array elements, chase param_offs_in_words into the parameter
// buffer, decode vertices, or build the ISP_BACKGND_T_type core tag - all of
// that is the consumer's job (param_fetch, shared with the TSP plane cache's
// miss path), using the raw mask/count this module exposes.
//
// Mirrors refsw RenderObjectList (refsw_lists.cpp) at the level of walking
// entries and following links; RenderTriangleStrip/RenderTriangleArray/
// RenderQuadArray's per-element iteration is left entirely to the consumer.
//
// Object-list entry word E (32-bit). Bitfields are LSB-first (as in refsw's
// structs, refsw_lists_regtypes.h):
//   Tstrip  : param_offs=E[20:0] skip=E[23:21] shadow=E[24] mask=E[30:25] is_not_ts=E[31]
//   T/Qarray: param_offs=E[20:0] skip=E[23:21] shadow=E[24] prims=E[28:25] type=E[31:29]
//   Link    : next_ptr_words=E[23:2] end_of_list=E[28] type=E[31:29]
// type 0b111=link, 0b100=tri array, 0b101=quad array. bit31==0 => triangle strip.
// count (array) = prims+1 (refsw's `obj.tarray.prims + 1`).
//
// DI: the VRAM read port is the INJECTED 256-bit data cache (creq/cresp).
// Handshake: prim.entry_ready (LEVEL, stable) <-> ack.entry_done (1-cycle pulse).
//
module object_list_parser import tsp_pkg::*; (
    input                 clk,
    input                 reset,
    input                 start,        // 1-cycle: begin at list_ptr
    input      [26:0]     list_ptr,     // byte address of the object list
    output reg            busy,
    output reg            done,         // 1-cycle: list fully walked

    output prim_out_t      prim,
    input  prim_ack_t       ack,

    // injected 256-bit data cache (object-list region only)
    output cache_req256_t  creq,
    input  cache_resp256_t cresp
);
    // ---- 32-bit word reader over the 256-bit line cache ----
    reg  [26:0] raddr; reg rd_go; reg [31:0] rword; reg rword_v; reg [2:0] rsel;
    reg  [26:0] creq_laddr_r; reg creq_req_r;
    assign creq.req   = creq_req_r;
    assign creq.laddr = creq_laddr_r;

    // ---- entry output regs ----
    reg             e_ready_r;
    entry_type_e    e_type_r;
    objlist_entry_t e_fields_r;
    assign prim.entry_ready = e_ready_r;
    assign prim.entry_type  = e_type_r;
    assign prim.entry       = e_fields_r;

    // ---- walk state ----
    localparam S_IDLE   = 3'd0,
               S_RDENT  = 3'd1,   // issue read of entry word
               S_RDENTW = 3'd2,   // wait entry word
               S_CLASS  = 3'd3,   // classify entry
               S_PRESENT= 3'd4,   // hold entry_ready, wait entry_done
               S_DONE   = 3'd5;
    reg [2:0] st;

    reg [26:0] base;              // current entry byte addr
    reg [31:0] ent;               // current entry word

    always @(posedge clk) begin
        if (reset) begin
            st<=S_IDLE; busy<=0; done<=0; creq_req_r<=0; rword_v<=0; e_ready_r<=0;
        end else begin
            done<=0; creq_req_r<=0; rword_v<=0;

            // word-read sub-engine
            if (rd_go) begin
                creq_req_r   <= 1'b1;
                creq_laddr_r <= raddr[26:5];
                rsel         <= raddr[4:2];
                rd_go        <= 1'b0;
            end
            if (cresp.ack) begin rword <= cresp.rdata[32*rsel +: 32]; rword_v <= 1'b1; end

            case (st)
            S_IDLE: if (start) begin base<=list_ptr; busy<=1; st<=S_RDENT; end

            S_RDENT:  begin raddr<=base; rd_go<=1'b1; st<=S_RDENTW; end
            S_RDENTW: if (rword_v) begin ent<=rword; base<=base+27'd4; st<=S_CLASS; end

            S_CLASS: begin
                if (ent[31] == 1'b0) begin            // triangle strip
                    e_type_r   <= ENT_STRIP;
                    e_fields_r <= '{ param_offs_in_words:ent[20:0], skip:ent[23:21],
                        shadow:ent[24], mask:ent[30:25], count:5'd0 };
                    st <= S_PRESENT;
                end else begin
                    case (ent[31:29])
                    3'b111: begin                     // link
                        if (ent[28]) st<=S_DONE;
                        else begin base<={ent[23:2],2'b00}; st<=S_RDENT; end
                    end
                    3'b100, 3'b101: begin              // tri / quad array
                        e_type_r   <= (ent[31:29]==3'b101) ? ENT_QUAD : ENT_TRI;
                        e_fields_r <= '{ param_offs_in_words:ent[20:0], skip:ent[23:21],
                            shadow:ent[24], mask:6'd0, count:{1'b0,ent[28:25]}+5'd1 };
                        st <= S_PRESENT;
                    end
                    // unhandled type (e.g. modifier-volume/sprite entries): SKIP
                    // it and continue the list, matching refsw RenderObjectList
                    // (its default prints a warning + breaks; only end_of_list
                    // stops). Terminating here would drop all following entries
                    // (e.g. doa2's environment after an inline unhandled entry).
                    default: st<=S_RDENT;
                    endcase
                end
            end

            // ---- present the entry, wait for the consumer to finish it ----
            S_PRESENT: begin
                e_ready_r <= 1'b1;
                if (ack.entry_done) begin
                    e_ready_r <= 1'b0;
                    st <= S_RDENT;
                end
            end

            S_DONE: begin busy<=0; done<=1; st<=S_IDLE; end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
