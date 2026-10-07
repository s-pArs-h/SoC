`default_nettype none

// System control: LEDs, switches, a 64-bit cycle counter and build info.
//
// Registers (byte offsets)
//   0x00 LED       RW  LED[15:0]
//   0x04 SW        RO  switches (synchronised)
//   0x08 CYCLE_LO  RO  cycle counter bits 31:0; reading it also captures
//                      bits 63:32 into CYCLE_HI, so LO then HI is one
//                      consistent 64-bit value
//   0x0C CYCLE_HI  RO  bits 63:32 captured by the last CYCLE_LO read
//   0x10 CLK_HZ    RO  core clock frequency, for software timeouts
module soc_sysctl #(
    parameter CLK_HZ = 50_000_000
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         sel,
    input  wire         re,
    input  wire [3:0]   we,
    input  wire [4:0]   addr,
    input  wire [31:0]  wdata,
    output logic [31:0] rdata,

    input  wire [15:0]  sw,
    output logic [15:0] led
);
    logic [63:0] cycle_q;
    logic [31:0] hi_q;
    logic [15:0] sw_m, sw_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_q <= '0;
            led     <= '0;
            sw_m    <= '0;
            sw_q    <= '0;
            hi_q    <= '0;
        end else begin
            cycle_q <= cycle_q + 1'b1;
            {sw_q, sw_m} <= {sw_m, sw};
            if (sel && addr[4:2] == 3'd0 && we[1:0] == 2'b11) led <= wdata[15:0];
            if (sel && re && addr[4:2] == 3'd2) hi_q <= cycle_q[63:32];
        end
    end

    always_ff @(posedge clk) begin
        if (sel && re) begin
            case (addr[4:2])
                3'd0:    rdata <= {16'b0, led};
                3'd1:    rdata <= {16'b0, sw_q};
                3'd2:    rdata <= cycle_q[31:0];
                3'd3:    rdata <= hi_q;
                3'd4:    rdata <= 32'(CLK_HZ);
                default: rdata <= 32'h0;
            endcase
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire unused = &{1'b0, wdata[31:16], addr[1:0], we[3:2]};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
