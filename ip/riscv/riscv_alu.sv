`default_nettype none

// RV32I ALU.
module riscv_alu (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [3:0]  op,
    output logic [31:0] y
);
    `include "rv32i_defs.svh"

    wire [4:0] shamt = b[4:0];

    always @(*) begin
        case (op)
            ALU_SUB:  y = a - b;
            ALU_SLL:  y = a << shamt;
            ALU_SLT:  y = {31'b0, $signed(a) < $signed(b)};
            ALU_SLTU: y = {31'b0, a < b};
            ALU_XOR:  y = a ^ b;
            ALU_SRL:  y = a >> shamt;
            ALU_SRA:  y = $unsigned($signed(a) >>> shamt);
            ALU_OR:   y = a | b;
            ALU_AND:  y = a & b;
            default:  y = a + b;                    // ALU_ADD
        endcase
    end

endmodule

`default_nettype wire
