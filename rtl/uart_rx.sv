`default_nettype none

// UART receiver, 8N1, LSB first. div = clock cycles per bit (at least 4).
//
// The input is asynchronous, so it first passes a two-flop synchroniser.
// A falling edge starts a frame; the start bit is re-checked half a bit
// later (a shorter glitch is ignored), and every following bit is sampled
// in its middle. A byte is delivered as a one-cycle valid pulse after a
// good stop bit; a low stop bit raises frame_err instead, and the receiver
// then waits for the line to return high (a "break" holds it low).
module uart_rx #(
    parameter DIV_W = 16
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [DIV_W-1:0] div,

    input  wire             rxd,

    output logic            valid,
    output logic [7:0]      data,
    output logic            frame_err
);
    logic [1:0] sync;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 2'b11;
        else        sync <= {sync[0], rxd};
    end
    wire rx = sync[1];

    localparam [2:0] S_IDLE  = 3'd0,
                     S_START = 3'd1,
                     S_DATA  = 3'd2,
                     S_STOP  = 3'd3,
                     S_BREAK = 3'd4;

    logic [2:0]       state;
    logic [DIV_W-1:0] cnt;
    logic [2:0]       bit_idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            cnt       <= '0;
            bit_idx   <= '0;
            data      <= '0;
            valid     <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            valid     <= 1'b0;
            frame_err <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (!rx) begin
                        state <= S_START;
                        cnt   <= (div >> 1) - 1'b1;
                    end
                end
                S_START: begin
                    if (cnt != '0) cnt <= cnt - 1'b1;
                    else if (rx)   state <= S_IDLE;            // glitch
                    else begin
                        state   <= S_DATA;
                        cnt     <= div - 1'b1;
                        bit_idx <= '0;
                    end
                end
                S_DATA: begin
                    if (cnt != '0) cnt <= cnt - 1'b1;
                    else begin
                        data    <= {rx, data[7:1]};
                        cnt     <= div - 1'b1;
                        bit_idx <= bit_idx + 1'b1;
                        if (bit_idx == 3'd7) state <= S_STOP;
                    end
                end
                S_STOP: begin
                    if (cnt != '0) cnt <= cnt - 1'b1;
                    else if (rx) begin
                        valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        frame_err <= 1'b1;
                        state     <= S_BREAK;
                    end
                end
                default: begin                                 // S_BREAK
                    if (rx) state <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
