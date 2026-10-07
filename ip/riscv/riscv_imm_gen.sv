`default_nettype none

// Immediate generator for the five RV32I immediate formats.
module riscv_imm_gen (
    input  wire [31:7] instr,      // the opcode bits are not needed
    input  wire [2:0]  imm_type,
    output logic [31:0] imm
);
    `include "rv32i_defs.svh"

    always @(*) begin
        case (imm_type)
            IMM_S:   imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            IMM_B:   imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            IMM_U:   imm = {instr[31:12], 12'b0};
            IMM_J:   imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            default: imm = {{20{instr[31]}}, instr[31:20]};          // IMM_I
        endcase
    end

endmodule

`default_nettype wire
