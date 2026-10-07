`default_nettype none

// Pipelined arg-min over K distances (K a power of two, K >= 2).
//
// The comparators form a binary tree stored in heap order: node n has
// children 2n and 2n+1, and nodes K .. 2K-1 are the leaves (leaf K+k is
// centroid k). Every internal node is a register, so the tree adds
// log2(K) pipeline stages. Stage 0 is the level next to the leaves; the
// root (node 1) is the last stage.
//
// Ties go to the left child, i.e. the lower centroid index, which makes the
// result deterministic and easy to match in a software model.
module min_tree #(
    parameter K      = 4,
    parameter DIST_W = 33,
    parameter IDX_W  = $clog2(K),
    parameter LVLS   = $clog2(K)
) (
    input  wire                clk,
    input  wire [LVLS-1:0]     en,         // en[j] loads tree stage j
    input  wire [K*DIST_W-1:0] leaf_dist,  // distance to centroid k at [k*DIST_W +: DIST_W]
    output wire [DIST_W-1:0]   min_dist,
    output wire [IDX_W-1:0]    min_idx
);
    logic [DIST_W-1:0] node_d [1:K-1];
    logic [IDX_W-1:0]  node_i [1:K-1];

    genvar n;
    generate
        for (n = 1; n < K; n = n + 1) begin : g_node
            localparam DEPTH = $clog2(n + 1) - 1;   // root is depth 0
            localparam STAGE = LVLS - 1 - DEPTH;    // leaves' parents load first

            wire [DIST_W-1:0] l_d, r_d;
            wire [IDX_W-1:0]  l_i, r_i;

            if (2 * n >= K) begin : g_leaves
                localparam integer L_IDX = 2 * n - K;
                localparam integer R_IDX = 2 * n + 1 - K;
                assign l_d = leaf_dist[L_IDX*DIST_W +: DIST_W];
                assign r_d = leaf_dist[R_IDX*DIST_W +: DIST_W];
                assign l_i = L_IDX[IDX_W-1:0];
                assign r_i = R_IDX[IDX_W-1:0];
            end else begin : g_inner
                assign l_d = node_d[2*n];
                assign r_d = node_d[2*n+1];
                assign l_i = node_i[2*n];
                assign r_i = node_i[2*n+1];
            end

            wire take_left = (l_d <= r_d);

            always_ff @(posedge clk) begin
                if (en[STAGE]) begin
                    node_d[n] <= take_left ? l_d : r_d;
                    node_i[n] <= take_left ? l_i : r_i;
                end
            end
        end
    endgenerate

    assign min_dist = node_d[1];
    assign min_idx  = node_i[1];

endmodule

`default_nettype wire
