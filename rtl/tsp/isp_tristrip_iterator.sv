//
// isp_tristrip_iterator - consumes ONE ENT_STRIP object-list entry and
// iterates its masked triangles, reading ONLY position (X,Y,Z) per vertex from
// the parameter buffer - UV/color/offset/tsp/tcw formats are ignored for now.
//
// Per refsw decode_pvr_vertices (refsw_tile.cpp:293): at the record base, read
// isp(+0); tsp(+4) and tcw(+8) are present but not read here (not needed for
// XYZ-only decode); if two_volumes (=shadow), a second tsp1/tcw1 pair follows
// (+12,+16), so the vertex array starts 8 bytes later. Vertex k begins at
//   rec_base + hdr_bytes + k*stride_bytes
// where hdr_bytes = 12 (+8 if two_volumes) and
//   stride_bytes  = (3 + skip*(1+two_volumes)) * 4     (matches decode_pvr_vertices)
// Only the first 3 words (X,Y,Z) of each vertex are read; any UV/color/offset
// words within the stride are skipped over (address arithmetic only, no read).
//
// Strip iteration (refsw RenderTriangleStrip, refsw_lists.cpp:176): up to 8
// vertices / 6 triangles; triangle i uses vertices (i+(i&1), i+((i&1)^1), i+2)
// - i.e. winding alternates - and exists only if mask[i] is set (entry.mask
// here is bit i = LSB-first, matching object_list_parser's E[30:25] mapping).
//
// Handshake: triangle_ready (LEVEL, v0/v1/v2/isp stable) <-> triangle_done
// (1-cycle pulse) per triangle; prim_done (1-cycle) when the whole entry's
// masked triangles are exhausted (this module's own "I'm done with the input"
// signal, analogous to object_list_parser's entry_done one level up).
//
// DI: the VRAM read port is the INJECTED 256-bit data cache (creq/cresp).
//
module isp_tristrip_iterator import tsp_pkg::*; (
    input                  clk,
    input                  reset,
    input                  start,          // 1-cycle: begin iterating `entry`
    input      [26:0]      param_base,     // PARAM_BASE & 0xF00000 (byte addr)
    input      objlist_entry_t entry,      // must be an ENT_STRIP entry (mask valid)
    output reg             busy,

    output triangle_out_t  trio,
    input  triangle_ack_t  ack,

    // injected 256-bit param data cache
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
        creq_req_r <= 1'b0;                // default: 1-cycle pulse
        if (rd_go) begin
            creq_req_r   <= 1'b1;
            creq_laddr_r <= raddr[26:5];
            rsel         <= raddr[4:2];
        end
        if (cresp.ack) begin rword <= cresp.rdata[32*rsel +: 32]; rword_v <= 1'b1; end
        if (reset) creq_req_r <= 1'b0;
    end

    // ---- geometry (latched at start) ----
    reg [26:0] rec_base;      // byte addr of this entry's record (isp word)
    reg [2:0]  skip_r; reg shadow_r; reg [5:0] mask_r;
    wire       two_vol   = shadow_r;
    wire [4:0] stride_w  = 5'd3 + skip_r * (two_vol ? 5'd2 : 5'd1);   // words/vertex
    wire [26:0] hdr_bytes = two_vol ? 27'd20 : 27'd12;
    wire [26:0] stride_bytes = {stride_w,2'b00};

    // vertex slot storage (up to 8 vertices' XYZ; strip needs random access by i,i^1,2)
    xyz_t vslot [0:7];
    reg [31:0] isp_r;

    // ---- output regs ----
    reg            tri_ready_r;
    xyz_t          v0_r, v1_r, v2_r;
    reg            pdone_r;
    assign trio.triangle_ready = tri_ready_r;
    assign trio.isp            = isp_r;
    assign trio.v0 = v0_r; assign trio.v1 = v1_r; assign trio.v2 = v2_r;
    assign trio.prim_done      = pdone_r;

    // ---- walk state ----
    localparam S_IDLE  = 4'd0,
               S_ISP    = 4'd1,   // issue read of isp word
               S_ISPW   = 4'd2,   // wait isp word
               S_VADDR  = 4'd3,   // compute this vertex's XYZ addr
               S_VX     = 4'd4,   // read X
               S_VXW    = 4'd5,
               S_VY     = 4'd6,   // read Y
               S_VYW    = 4'd7,
               S_VZ     = 4'd8,   // read Z
               S_VZW    = 4'd9,
               S_VNEXT  = 4'd10,  // store vertex, advance vidx or move to present
               S_SEEK   = 4'd11,  // seek s_i to next set mask bit
               S_PRESENT= 4'd12,  // hold triangle_ready, wait triangle_done
               S_PDONE  = 4'd13;  // pulse prim_done
    reg [3:0] st;

    reg [3:0] vidx;      // which vertex (0..7) currently being fetched
    reg [26:0] vaddr;    // running byte addr of the vertex's X word
    reg [2:0]  s_i;      // strip triangle index 0..5

    // vertex indices used by triangle i (refsw: not_even=i&1, even=!not_even)
    function automatic [3:0] va(input [2:0] i); va = {1'b0,i} + (i[0] ? 4'd1 : 4'd0); endfunction
    function automatic [3:0] vb(input [2:0] i); vb = {1'b0,i} + (i[0] ? 4'd0 : 4'd1); endfunction
    // vc(i) = i+2 always

    always @(posedge clk) begin
        if (reset) begin
            st<=S_IDLE; busy<=0; tri_ready_r<=0; pdone_r<=0; rd_go<=0;
        end else begin
            pdone_r <= 1'b0;
            rd_go   <= 1'b0;

            case (st)
            S_IDLE: if (start) begin
                rec_base <= param_base + {entry.param_offs_in_words,2'b00};
                skip_r   <= entry.skip;
                shadow_r <= entry.shadow;
                mask_r   <= entry.mask;
                busy     <= 1'b1;
                st       <= S_ISP;
            end

            S_ISP:  begin raddr<=rec_base; rd_go<=1'b1; st<=S_ISPW; end
            S_ISPW: if (rword_v) begin isp_r<=rword; vidx<=4'd0; st<=S_VADDR; end

            // ---- read all 8 vertices' XYZ up front (simplest; strip needs
            // random access across triangles anyway) ----
            S_VADDR: begin
                vaddr <= rec_base + hdr_bytes + (vidx * stride_bytes);
                st    <= S_VX;
            end
            S_VX:  begin raddr<=vaddr;          rd_go<=1'b1; st<=S_VXW; end
            S_VXW: if (rword_v) begin vslot[vidx].x<=rword; raddr<=vaddr+27'd4; rd_go<=1'b1; st<=S_VY; end
            S_VY:  if (rword_v) begin vslot[vidx].y<=rword; raddr<=vaddr+27'd8; rd_go<=1'b1; st<=S_VZ; end
            S_VZ:  if (rword_v) begin vslot[vidx].z<=rword; st<=S_VZW; end
            S_VZW: begin
                if (vidx == 4'd7) begin s_i<=3'd0; st<=S_SEEK; end
                else begin vidx<=vidx+4'd1; st<=S_VADDR; end
            end

            // ---- seek s_i to the next set mask bit ----
            // refsw gates triangle i by mask & (1 << (5-i)), so tri i uses mask[5-i].
            S_SEEK: begin
                if (mask_r[3'd5 - s_i]) begin
                    v0_r <= vslot[va(s_i)]; v1_r <= vslot[vb(s_i)]; v2_r <= vslot[{1'b0,s_i}+4'd2];
                    st <= S_PRESENT;
                end else if (s_i == 3'd5) st <= S_PDONE;
                else s_i <= s_i + 3'd1;
            end

            S_PRESENT: begin
                tri_ready_r <= 1'b1;
                if (ack.triangle_done) begin
                    tri_ready_r <= 1'b0;
                    if (s_i == 3'd5) st <= S_PDONE;
                    else begin s_i <= s_i + 3'd1; st <= S_SEEK; end
                end
            end

            S_PDONE: begin busy<=0; pdone_r<=1'b1; st<=S_IDLE; end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
