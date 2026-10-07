`default_nettype none

// Squared Euclidean distance between one 2-D point and one centroid.
//
//   stage 1: dx = px - cx,  dy = py - cy      (DATA_W+1 bits, signed)
//   stage 2: sqx = dx*dx,   sqy = dy*dy       (2*DATA_W bits, unsigned)
//   stage 3: dist = sqx + sqy                 (2*DATA_W+1 bits, unsigned)
//
// Width argument (DATA_W = 16):
//   |dx| <= 2^16 - 1, so dx*dx <= (2^16 - 1)^2 < 2^32   -> fits in 32 unsigned bits
//   sqx + sqy < 2^33                                     -> fits in 33 bits
// The original v1 design truncated the final sum to 32 bits, so a far-away
// centroid could wrap around and look like the nearest one.
//
// Each stage has its own load enable (driven by the core) so registers only
// toggle when they carry a valid point. Datapath registers have no reset.
module distance_calc #(
    parameter DATA_W = 16,
    parameter DIST_W = 2*DATA_W + 1
) (
    input  wire                     clk,
    input  wire [2:0]               en,      // en[i] loads pipeline stage i+1
    input  wire signed [DATA_W-1:0] px,
    input  wire signed [DATA_W-1:0] py,
    input  wire signed [DATA_W-1:0] cx,
    input  wire signed [DATA_W-1:0] cy,
    output logic [DIST_W-1:0]       dist_o
);
    localparam DIFF_W = DATA_W + 1;
    localparam SQ_W   = 2 * DATA_W;

    logic signed [DIFF_W-1:0] dx, dy;
    logic        [SQ_W-1:0]   sqx, sqy;

    // Full-width signed products. The top two bits are always zero (the
    // square is non-negative and < 2^SQ_W); the formal check in formal/
    // proves the final distance is exact.
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [2*DIFF_W-1:0] prod_x = dx * dx;
    wire signed [2*DIFF_W-1:0] prod_y = dy * dy;
    /* verilator lint_on UNUSEDSIGNAL */

    always_ff @(posedge clk) begin
        if (en[0]) begin
            dx <= $signed({px[DATA_W-1], px}) - $signed({cx[DATA_W-1], cx});
            dy <= $signed({py[DATA_W-1], py}) - $signed({cy[DATA_W-1], cy});
        end
        if (en[1]) begin
            sqx <= prod_x[SQ_W-1:0];
            sqy <= prod_y[SQ_W-1:0];
        end
        if (en[2]) begin
            dist_o <= {1'b0, sqx} + {1'b0, sqy};
        end
    end

endmodule

`default_nettype wire
