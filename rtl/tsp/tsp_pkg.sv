//
// tsp_pkg - packed-struct port bundles for dependency injection of the texture
// caches and the DDR3 read port. Using plain packed structs (no SV interfaces)
// keeps the design maximally tool-portable while still letting a cache be
// "injected" into a consumer (tex_fetch/tsp_shade) and the DDR3 port be injected
// into a cache - the concrete instances live at the top and are wired through by
// passing these bundles down the hierarchy.
//
package tsp_pkg;

    // ---- DDR3 raw 64-bit read port ----
    // request: cache -> DDR arbiter
    typedef struct packed {
        logic        rd;        // read strobe (accepted when !resp.busy)
        logic [28:0] addr;      // 64-bit-word address ({4'b0011, waddr[24:0]})
        logic [7:0]  burst;     // burst count (1 for a single 64-bit line)
    } ddr_rd_req_t;
    // response: DDR arbiter -> cache
    typedef struct packed {
        logic        busy;      // cannot accept a read this cycle
        logic [63:0] dout;      // read data
        logic        dready;    // dout valid this cycle
    } ddr_rd_resp_t;

    // ---- cache client port (a 64-bit direct-mapped line cache) ----
    // request: client (tex_fetch) -> cache
    typedef struct packed {
        logic        req;       // 1-cycle request strobe
        logic [28:0] waddr;     // 64-bit-word address
    } cache_req_t;
    // response: cache -> client
    typedef struct packed {
        logic        ack;       // 1-cycle response strobe
        logic [63:0] rdata;     // 64-bit line
    } cache_resp_t;

endpackage
