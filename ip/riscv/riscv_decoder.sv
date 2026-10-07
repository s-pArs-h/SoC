`default_nettype none

// RV32I instruction decoder (combinational).
//
// Decodes all 40 RV32I base instructions. FENCE is executed as a no-op
// (single hart, no caches). ECALL and EBREAK raise a trap, which this core
// uses to stop. Every other encoding, including FENCE.I, CSR instructions,
// compressed instructions and RV32I encodings with reserved funct7 bits, is
// flagged illegal. Control outputs are forced to "do nothing" when illegal.
module riscv_decoder (
    input  wire [31:0] instr,
    output logic [3:0] alu_op,
    output logic [1:0] a_sel,       // A_RS1 / A_PC / A_ZERO
    output logic       b_imm,       // 1: operand B is the immediate
    output logic [2:0] imm_type,
    output logic [1:0] wb_sel,      // WB_ALU / WB_MEM / WB_PC4
    output logic       reg_write,
    output logic       mem_read,
    output logic       mem_write,
    output logic       branch,
    output logic       jal,
    output logic       jalr,
    output logic       env_trap,    // ECALL / EBREAK
    output logic       illegal
);
    `include "rv32i_defs.svh"

    wire [6:0] opcode = instr[6:0];
    wire [2:0] f3     = instr[14:12];
    wire [6:0] f7     = instr[31:25];

    always @(*) begin
        alu_op    = ALU_ADD;
        a_sel     = A_RS1;
        b_imm     = 1'b0;
        imm_type  = IMM_I;
        wb_sel    = WB_ALU;
        reg_write = 1'b0;
        mem_read  = 1'b0;
        mem_write = 1'b0;
        branch    = 1'b0;
        jal       = 1'b0;
        jalr      = 1'b0;
        env_trap  = 1'b0;
        illegal   = 1'b0;

        case (opcode)
            OP_LUI: begin
                reg_write = 1'b1;
                a_sel     = A_ZERO;
                b_imm     = 1'b1;
                imm_type  = IMM_U;
            end
            OP_AUIPC: begin
                reg_write = 1'b1;
                a_sel     = A_PC;
                b_imm     = 1'b1;
                imm_type  = IMM_U;
            end
            OP_JAL: begin
                reg_write = 1'b1;
                jal       = 1'b1;
                wb_sel    = WB_PC4;
                imm_type  = IMM_J;
            end
            OP_JALR: begin
                reg_write = 1'b1;
                jalr      = 1'b1;
                wb_sel    = WB_PC4;
                b_imm     = 1'b1;          // ALU computes rs1 + imm
                illegal   = (f3 != 3'b000);
            end
            OP_BRANCH: begin
                branch    = 1'b1;
                imm_type  = IMM_B;
                illegal   = (f3 == 3'b010) || (f3 == 3'b011);
            end
            OP_LOAD: begin
                reg_write = 1'b1;
                mem_read  = 1'b1;
                wb_sel    = WB_MEM;
                b_imm     = 1'b1;
                // LB LH LW LBU LHU
                illegal   = (f3 == 3'b011) || (f3 == 3'b110) || (f3 == 3'b111);
            end
            OP_STORE: begin
                mem_write = 1'b1;
                b_imm     = 1'b1;
                imm_type  = IMM_S;
                illegal   = (f3[2] == 1'b1) || (f3 == 3'b011);   // SB SH SW only
            end
            OP_IMM: begin
                reg_write = 1'b1;
                b_imm     = 1'b1;
                case (f3)
                    3'b000: alu_op = ALU_ADD;
                    3'b010: alu_op = ALU_SLT;
                    3'b011: alu_op = ALU_SLTU;
                    3'b100: alu_op = ALU_XOR;
                    3'b110: alu_op = ALU_OR;
                    3'b111: alu_op = ALU_AND;
                    3'b001: begin
                        alu_op  = ALU_SLL;
                        illegal = (f7 != 7'b0000000);
                    end
                    default: begin          // 3'b101
                        alu_op  = f7[5] ? ALU_SRA : ALU_SRL;
                        illegal = (f7 != 7'b0000000) && (f7 != 7'b0100000);
                    end
                endcase
            end
            OP_REG: begin
                reg_write = 1'b1;
                case ({f7, f3})
                    {7'b0000000, 3'b000}: alu_op = ALU_ADD;
                    {7'b0100000, 3'b000}: alu_op = ALU_SUB;
                    {7'b0000000, 3'b001}: alu_op = ALU_SLL;
                    {7'b0000000, 3'b010}: alu_op = ALU_SLT;
                    {7'b0000000, 3'b011}: alu_op = ALU_SLTU;
                    {7'b0000000, 3'b100}: alu_op = ALU_XOR;
                    {7'b0000000, 3'b101}: alu_op = ALU_SRL;
                    {7'b0100000, 3'b101}: alu_op = ALU_SRA;
                    {7'b0000000, 3'b110}: alu_op = ALU_OR;
                    {7'b0000000, 3'b111}: alu_op = ALU_AND;
                    default:              illegal = 1'b1;
                endcase
            end
            OP_FENCE: begin
                illegal = (f3 != 3'b000);   // FENCE is a no-op; FENCE.I unsupported
            end
            OP_SYSTEM: begin
                // ECALL = 0x00000073, EBREAK = 0x00100073
                if (instr == 32'h0000_0073 || instr == 32'h0010_0073) env_trap = 1'b1;
                else                                                  illegal  = 1'b1;
            end
            default: illegal = 1'b1;
        endcase

        if (illegal) begin
            reg_write = 1'b0;
            mem_read  = 1'b0;
            mem_write = 1'b0;
            branch    = 1'b0;
            jal       = 1'b0;
            jalr      = 1'b0;
        end
    end

endmodule

`default_nettype wire
