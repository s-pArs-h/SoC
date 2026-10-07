`default_nettype none

// Nexys A7-100T top level for the K-means SoC.
//
//   clock   100 MHz board oscillator -> MMCM -> 50 MHz system clock
//   reset   CPU_RESETN button (active low), held until the MMCM locks,
//           released synchronously
//   UART    the board's USB-UART bridge: UART_TXD_IN carries data from the
//           PC to the FPGA, UART_RXD_OUT from the FPGA to the PC
//           (names are from the bridge's point of view), 115200 baud
//   LEDs    LED[15] toggles on every request, LED[8] mode of the last run,
//           LED[7:0] its iteration count; LED16 red: the CPU has stopped
//   SW      readable by software (unused by the default firmware)
//
// Define SIM to replace the MMCM with the board clock.
module top_nexys_a7 #(
    parameter INIT_FILE = "firmware.hex",
    parameter CLK_HZ    = 50_000_000,
    parameter BAUD      = 115_200
) (
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,
    input  wire        UART_TXD_IN,
    output wire        UART_RXD_OUT,
    input  wire [15:0] SW,
    output wire [15:0] LED,
    output wire        LED16_R
);
    // ------------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------------
    wire clk, locked;
`ifdef SIM
    assign clk    = CLK100MHZ;
    assign locked = 1'b1;
`else
    wire clk_mmcm, clk_fb, clk_fb_buf;
    MMCME2_BASE #(
        .CLKIN1_PERIOD    (10.0),
        .CLKFBOUT_MULT_F  (10.0),               // VCO = 1000 MHz
        .CLKOUT0_DIVIDE_F (1.0e9 / CLK_HZ)      // 50 MHz
    ) u_mmcm (
        .CLKIN1(CLK100MHZ), .CLKFBIN(clk_fb_buf), .CLKFBOUT(clk_fb),
        .CLKOUT0(clk_mmcm), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0),
        .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(), .CLKFBOUTB()
    );
    BUFG u_bufg_fb  (.I(clk_fb),   .O(clk_fb_buf));
    BUFG u_bufg_clk (.I(clk_mmcm), .O(clk));
`endif

    // ------------------------------------------------------------------
    // Reset: asserted asynchronously, released synchronously
    // ------------------------------------------------------------------
    wire        rst_in_n = CPU_RESETN && locked;
    logic [1:0] rst_sync;
    always_ff @(posedge clk or negedge rst_in_n) begin
        if (!rst_in_n) rst_sync <= 2'b00;
        else           rst_sync <= {rst_sync[0], 1'b1};
    end

    soc #(
        .INIT_FILE (INIT_FILE),
        .CLK_HZ    (CLK_HZ),
        .BAUD      (BAUD)
    ) u_soc (
        .clk      (clk),
        .rst_n    (rst_sync[1]),
        .uart_rxd (UART_TXD_IN),
        .uart_txd (UART_RXD_OUT),
        .sw       (SW),
        .led      (LED),
        .halted   (LED16_R)
    );

endmodule

`default_nettype wire
