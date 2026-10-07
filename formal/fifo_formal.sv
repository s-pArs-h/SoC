`default_nettype none

// Formal harness for sync_fifo: every input is free, reset in the first cycle.
module fifo_formal #(
    parameter W     = 8,
    parameter DEPTH = 4
) (
    input wire         clk,
    input wire         rst_n,
    input wire         wr_en,
    input wire [W-1:0] wr_data,
    input wire         rd_en
);
    localparam AW = $clog2(DEPTH);
    wire         full, empty;
    wire [W-1:0] rd_data;
    wire [AW:0]  count;

    sync_fifo #(.W(W), .DEPTH(DEPTH)) dut (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en), .wr_data(wr_data), .full(full),
        .rd_en(rd_en), .rd_data(rd_data), .empty(empty),
        .count(count)
    );

    reg f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;
    always @(*) assume(rst_n == f_past_valid);

    // flags agree with the count
    always @(*) if (rst_n) begin
        assert(empty == (count == 0));
        assert(full  == (count == (AW+1)'(DEPTH)));
    end

    // the count moves by exactly one per accepted push or pop
    reg [AW:0] f_count_q;
    reg        f_push_q, f_pop_q;
    always @(posedge clk) begin
        f_count_q <= count;
        f_push_q  <= wr_en && (!full || (rd_en && !empty));
        f_pop_q   <= rd_en && !empty;
    end
    always @(posedge clk)
        if (f_past_valid && rst_n && $past(rst_n))
            assert(count == f_count_q + {{AW{1'b0}}, f_push_q} - {{AW{1'b0}}, f_pop_q});
endmodule

`default_nettype wire
