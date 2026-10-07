`default_nettype none

// UART transmitter, 8 data bits, no parity, 1 stop bit, LSB first.
// div = clock cycles per bit (at least 2). A byte is taken when
// valid && ready; ready is low while a frame is being sent.
module uart_tx #(
    parameter DIV_W = 16
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [DIV_W-1:0] div,

    input  wire             valid,
    output wire             ready,
    input  wire [7:0]       data,

    output wire             txd
);
    logic [9:0]       shift;            // {stop, data, start}, sent from bit 0
    logic [3:0]       bits_left;
    logic [DIV_W-1:0] cnt;

    assign ready = (bits_left == 4'd0);
    assign txd   = ready ? 1'b1 : shift[0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift     <= '1;
            bits_left <= '0;
            cnt       <= '0;
        end else if (ready) begin
            if (valid) begin
                shift     <= {1'b1, data, 1'b0};
                bits_left <= 4'd10;
                cnt       <= div - 1'b1;
            end
        end else if (cnt != '0) begin
            cnt <= cnt - 1'b1;
        end else begin
            shift     <= {1'b1, shift[9:1]};
            bits_left <= bits_left - 1'b1;
            cnt       <= div - 1'b1;
        end
    end

endmodule

`default_nettype wire
