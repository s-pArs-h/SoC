`default_nettype none

// UART peripheral: transmitter and receiver with FIFOs, polled by software.
//
// Registers (byte offsets)
//   0x0 TXDATA  W   bits 7:0 are queued for sending (dropped if the FIFO is full)
//               R   bit 31: TX FIFO full
//   0x4 RXDATA  R   bits 7:0: oldest received byte, which is removed;
//                   bit 31: RX FIFO was empty (bits 7:0 are then not valid)
//   0x8 STATUS  R   bit 0 TX full, bit 1 TX idle (FIFO empty, line idle),
//                   bit 2 RX empty, bit 3 RX overrun, bit 4 frame error
//               W   write 1 to bit 3 / bit 4 to clear it
//   0xC DIV     RW  clock cycles per bit (baud rate = clock / DIV), >= 4
//
// Testing "empty" and taking the byte in one load (as SiFive's UART does)
// halves the polling cost per received byte. The read side effect is safe
// with this core: loads are never speculative and each is issued once.
module soc_uart #(
    parameter DEFAULT_DIV = 434,        // 50 MHz / 115200
    parameter TX_DEPTH    = 16,
    parameter RX_DEPTH    = 16
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         sel,
    input  wire         re,
    input  wire [3:0]   we,
    input  wire [3:0]   addr,
    input  wire [31:0]  wdata,
    output logic [31:0] rdata,

    input  wire         rxd,
    output wire         txd
);
    logic [15:0] div_q;
    logic        overrun_q, ferr_q;

    // ------------------------------------------------------------------
    // Transmit path: TXDATA -> FIFO -> uart_tx
    // ------------------------------------------------------------------
    wire       tx_full, tx_empty, tx_ready;
    wire [7:0] tx_byte;
    wire       tx_push = sel && we[0] && addr[3:2] == 2'd0;

    /* verilator lint_off PINCONNECTEMPTY */
    sync_fifo #(.W(8), .DEPTH(TX_DEPTH)) u_txq (
        .clk(clk), .rst_n(rst_n),
        .wr_en(tx_push), .wr_data(wdata[7:0]), .full(tx_full),
        .rd_en(tx_ready), .rd_data(tx_byte), .empty(tx_empty),
        .count()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    uart_tx u_tx (
        .clk(clk), .rst_n(rst_n), .div(div_q),
        .valid(!tx_empty), .ready(tx_ready), .data(tx_byte),
        .txd(txd)
    );

    // ------------------------------------------------------------------
    // Receive path: uart_rx -> FIFO -> RXDATA
    // ------------------------------------------------------------------
    wire       rx_valid, rx_ferr, rx_full, rx_empty;
    wire [7:0] rx_byte, rx_head;
    wire       rx_pop = sel && re && addr[3:2] == 2'd1;

    uart_rx u_rx (
        .clk(clk), .rst_n(rst_n), .div(div_q),
        .rxd(rxd),
        .valid(rx_valid), .data(rx_byte), .frame_err(rx_ferr)
    );

    /* verilator lint_off PINCONNECTEMPTY */
    sync_fifo #(.W(8), .DEPTH(RX_DEPTH)) u_rxq (
        .clk(clk), .rst_n(rst_n),
        .wr_en(rx_valid), .wr_data(rx_byte), .full(rx_full),
        .rd_en(rx_pop), .rd_data(rx_head), .empty(rx_empty),
        .count()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------
    wire tx_idle = tx_empty && tx_ready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_q     <= 16'(DEFAULT_DIV);
            overrun_q <= 1'b0;
            ferr_q    <= 1'b0;
        end else begin
            if (sel && addr[3:2] == 2'd3 && we[1:0] == 2'b11) div_q <= wdata[15:0];
            if (sel && addr[3:2] == 2'd2 && we[0]) begin
                if (wdata[3]) overrun_q <= 1'b0;
                if (wdata[4]) ferr_q    <= 1'b0;
            end
            if (rx_valid && rx_full && !rx_pop) overrun_q <= 1'b1;   // byte lost
            if (rx_ferr)                        ferr_q    <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (sel && re) begin
            case (addr[3:2])
                2'd0:    rdata <= {tx_full, 31'b0};
                2'd1:    rdata <= {rx_empty, 23'b0, rx_empty ? 8'h00 : rx_head};
                2'd2:    rdata <= {27'b0, ferr_q, overrun_q, rx_empty, tx_idle, tx_full};
                default: rdata <= {16'b0, div_q};
            endcase
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire unused = &{1'b0, wdata[31:16], addr[1:0], we[3:2]};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
