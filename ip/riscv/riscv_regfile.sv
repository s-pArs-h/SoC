`default_nettype none

// 32 x 32 register file: two asynchronous read ports, one synchronous write
// port. x0 always reads as zero and is never written. Maps to distributed
// RAM (LUTRAM) on Xilinx 7-series. No reset: RISC-V does not define register
// values after reset, and software initialises what it uses.
module riscv_regfile (
    input  wire        clk,
    input  wire [4:0]  rs1,
    input  wire [4:0]  rs2,
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data,
    input  wire        we,
    input  wire [4:0]  rd,
    input  wire [31:0] rd_data
);
    logic [31:0] regs [1:31];

    always_ff @(posedge clk) begin
        if (we && rd != 5'd0) regs[rd] <= rd_data;
    end

    assign rs1_data = (rs1 == 5'd0) ? 32'd0 : regs[rs1];
    assign rs2_data = (rs2 == 5'd0) ? 32'd0 : regs[rs2];

endmodule

`default_nettype wire
