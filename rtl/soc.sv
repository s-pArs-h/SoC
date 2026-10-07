`default_nettype none

// K-means SoC: a 5-stage RV32I core, on-chip RAM, the K-means accelerator,
// a UART and system control on one memory-mapped bus.
//
//   0x0000_0000  RAM          code, data and stack (RAM_WORDS x 32 bit)
//   0x1000_0000  K-means      registers, point memory, label memory
//   0x2000_0000  UART         to the host PC
//   0x3000_0000  system       LEDs, switches, cycle counter, clock frequency
//
// Bus protocol: the core's data port, unchanged. In the cycle a load or
// store is in EX the core drives the address, read strobe or byte write
// enables, and write data; read data is due on the next cycle. Every slave
// completes in one cycle (registered read data, writes on the clock edge),
// so the bus needs no wait states: the interconnect decodes address bits
// 31:28 into one select per slave and steers the read data back using the
// region registered with the request. Unmapped addresses read as zero and
// ignore writes. Instructions are fetched from RAM port A only.
module soc #(
    parameter RAM_WORDS = 4096,             // 16 KiB
    parameter INIT_FILE = "firmware.hex",
    parameter CLK_HZ    = 50_000_000,
    parameter BAUD      = 115_200,
    parameter K         = 4,
    parameter PTS_AW    = 12                // 4096 points
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         uart_rxd,
    output wire         uart_txd,
    input  wire [15:0]  sw,
    output wire [15:0]  led,
    output wire         halted
);
    localparam RAM_AW = $clog2(RAM_WORDS);

    // ------------------------------------------------------------------
    // CPU
    // ------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] imem_addr;                  // only the RAM word bits are used
    /* verilator lint_on UNUSEDSIGNAL */
    wire [31:0] imem_rdata;
    wire [31:0] dmem_addr, dmem_wdata;
    logic [31:0] dmem_rdata;
    wire        dmem_re;
    wire [3:0]  dmem_wmask;

    riscv_pipeline u_cpu (
        .clk(clk), .rst_n(rst_n),
        .imem_addr(imem_addr), .imem_rdata(imem_rdata),
        .dmem_addr(dmem_addr), .dmem_re(dmem_re), .dmem_wmask(dmem_wmask),
        .dmem_wdata(dmem_wdata), .dmem_rdata(dmem_rdata),
        .halted(halted)
    );

    // ------------------------------------------------------------------
    // Interconnect: address decode and read-data return
    // ------------------------------------------------------------------
    localparam [3:0] R_RAM = 4'h0, R_ACC = 4'h1, R_UART = 4'h2, R_SYS = 4'h3;

    wire [3:0] region = dmem_addr[31:28];
    wire       access = dmem_re || (dmem_wmask != 4'b0000);
    wire sel_ram  = access && region == R_RAM;
    wire sel_acc  = access && region == R_ACC;
    wire sel_uart = access && region == R_UART;
    wire sel_sys  = access && region == R_SYS;

    logic [3:0] rsel_q;                     // which slave answers this cycle
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)       rsel_q <= 4'hF;
        else if (dmem_re) rsel_q <= region;
    end

    wire [31:0] ram_rdata, acc_rdata, uart_rdata, sys_rdata;
    always @(*) begin
        case (rsel_q)
            R_RAM:   dmem_rdata = ram_rdata;
            R_ACC:   dmem_rdata = acc_rdata;
            R_UART:  dmem_rdata = uart_rdata;
            R_SYS:   dmem_rdata = sys_rdata;
            default: dmem_rdata = 32'h0;
        endcase
    end

    // ------------------------------------------------------------------
    // Slaves
    // ------------------------------------------------------------------
    soc_ram #(.WORDS(RAM_WORDS), .INIT_FILE(INIT_FILE)) u_ram (
        .clk(clk),
        .a_addr(imem_addr[RAM_AW+1:2]), .a_rdata(imem_rdata),
        .b_en(sel_ram), .b_we(sel_ram ? dmem_wmask : 4'b0000),
        .b_addr(dmem_addr[RAM_AW+1:2]), .b_wdata(dmem_wdata), .b_rdata(ram_rdata)
    );

    kmeans_accel #(.K(K), .PTS_AW(PTS_AW)) u_acc (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_acc), .re(dmem_re), .we(sel_acc ? dmem_wmask : 4'b0000),
        .addr(dmem_addr[17:0]), .wdata(dmem_wdata), .rdata(acc_rdata)
    );

    soc_uart #(.DEFAULT_DIV(CLK_HZ / BAUD)) u_uart (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_uart), .re(dmem_re), .we(sel_uart ? dmem_wmask : 4'b0000),
        .addr(dmem_addr[3:0]), .wdata(dmem_wdata), .rdata(uart_rdata),
        .rxd(uart_rxd), .txd(uart_txd)
    );

    soc_sysctl #(.CLK_HZ(CLK_HZ)) u_sys (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_sys), .re(dmem_re), .we(sel_sys ? dmem_wmask : 4'b0000),
        .addr(dmem_addr[4:0]), .wdata(dmem_wdata), .rdata(sys_rdata),
        .sw(sw), .led(led)
    );

    /* verilator lint_off UNUSEDSIGNAL */
    wire unused = &{1'b0, dmem_addr[27:18]};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
