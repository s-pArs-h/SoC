`default_nettype none

// Formal harness for kmeans_accel: the bus is driven by the solver with any
// sequence of reads and writes (any address, data and byte enables),
// including starting runs, rewriting NUM_POINTS and centroids at any time.
module accel_formal #(
    parameter K      = 2,
    parameter PTS_AW = 3
) (
    input wire        clk,
    input wire        rst_n,
    input wire        sel,
    input wire        re,
    input wire [3:0]  we,
    input wire [17:0] addr,
    input wire [31:0] wdata
);
    wire [31:0] rdata;

    kmeans_accel #(.K(K), .PTS_AW(PTS_AW)) dut (
        .clk(clk), .rst_n(rst_n),
        .sel(sel), .re(re), .we(we), .addr(addr), .wdata(wdata), .rdata(rdata)
    );

    reg f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;
    always @(*) assume(rst_n == f_past_valid);

    // the CPU never reads and writes in the same access
    always @(*) if (re) assume(we == 4'b0000);

endmodule

`default_nettype wire
