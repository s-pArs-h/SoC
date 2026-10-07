`default_nettype none

// Five-stage pipelined RV32I core: IF, ID, EX, MEM, WB.
//
// Memory timing contract (different from the single-cycle core):
//   * instruction memory: imem_rdata is the word at the address presented on
//     imem_addr in the PREVIOUS cycle (a block RAM with registered output)
//   * data memory: same, for reads (dmem_re); writes (dmem_wmask) happen on
//     the clock edge at the end of the cycle that drives them
// The port list is identical to riscv_core, so the same testbenches,
// riscv-tests and riscv-formal setup apply.
//
// Stages
//   IF   imem_addr = next fetch address (PC+4, or a branch target from EX)
//   ID   instruction arrives from imem; decode, register read, hazard check
//   EX   forwarding, ALU, branch resolution, traps, data address/store issue
//   MEM  load data arrives from dmem; alignment and extension
//   WB   register write; the instruction retires (RVFI record)
//
// Hazards
//   * data:    results forwarded from MEM and WB into EX; a load followed by
//              an instruction that uses its result stalls ID for one cycle
//   * control: predict not-taken; a taken branch or jump in EX redirects
//              fetch the same cycle and squashes the one instruction in ID
//   * traps:   detected in EX; younger instructions are squashed, older ones
//              finish, and the core halts when the trapping one reaches WB
module riscv_pipeline #(
    parameter [31:0] RESET_PC = 32'h0000_0000
) (
    input  wire         clk,
    input  wire         rst_n,

    output wire [31:0]  imem_addr,
    input  wire [31:0]  imem_rdata,

    output wire [31:0]  dmem_addr,
    output wire         dmem_re,
    output wire [3:0]   dmem_wmask,
    output wire [31:0]  dmem_wdata,
    input  wire [31:0]  dmem_rdata,

    output wire         halted
`ifdef RISCV_FORMAL
    ,
    output logic        rvfi_valid,
    output logic [63:0] rvfi_order,
    output logic [31:0] rvfi_insn,
    output logic        rvfi_trap,
    output logic        rvfi_halt,
    output logic        rvfi_intr,
    output logic [1:0]  rvfi_mode,
    output logic [1:0]  rvfi_ixl,
    output logic [4:0]  rvfi_rs1_addr,
    output logic [4:0]  rvfi_rs2_addr,
    output logic [31:0] rvfi_rs1_rdata,
    output logic [31:0] rvfi_rs2_rdata,
    output logic [4:0]  rvfi_rd_addr,
    output logic [31:0] rvfi_rd_wdata,
    output logic [31:0] rvfi_pc_rdata,
    output logic [31:0] rvfi_pc_wdata,
    output logic [31:0] rvfi_mem_addr,
    output logic [3:0]  rvfi_mem_rmask,
    output logic [3:0]  rvfi_mem_wmask,
    output logic [31:0] rvfi_mem_rdata,
    output logic [31:0] rvfi_mem_wdata
`endif
);
    `include "rv32i_defs.svh"

    // Signals that flow backwards between stages
    logic        stall;          // ID must hold (load-use hazard)
    logic        redirect;       // EX: taken branch or jump
    logic [31:0] redirect_pc;
    logic        ex_trap;        // EX: the instruction in EX traps
    logic        stop_q;         // a trap is in flight: fetch no more

    logic        wb_valid, wb_reg_write;
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;
    wire         wb_we = wb_valid && wb_reg_write;

    // ==================================================================
    // IF
    // ==================================================================
    logic [31:0] pc_q;           // address of the instruction ID wants next
    assign imem_addr = redirect ? redirect_pc : pc_q;

    // ==================================================================
    // ID
    // ==================================================================
    logic        id_valid;
    logic [31:0] id_pc;
    logic        id_fresh;       // the ID instruction is the one on imem_rdata
    logic [31:0] id_instr_q;     // copy, used while ID is stalled
    wire  [31:0] id_instr = id_fresh ? imem_rdata : id_instr_q;

    wire [4:0] id_rs1 = id_instr[19:15];
    wire [4:0] id_rs2 = id_instr[24:20];
    wire [6:0] id_op  = id_instr[6:0];

    wire [3:0] id_alu_op;
    wire [1:0] id_a_sel, id_wb_sel;
    wire [2:0] id_imm_type;
    wire       id_b_imm, id_reg_write, id_mem_read, id_mem_write;
    wire       id_branch, id_jal, id_jalr, id_env_trap, id_illegal;

    riscv_decoder u_dec (
        .instr     (id_instr),
        .alu_op    (id_alu_op),
        .a_sel     (id_a_sel),
        .b_imm     (id_b_imm),
        .imm_type  (id_imm_type),
        .wb_sel    (id_wb_sel),
        .reg_write (id_reg_write),
        .mem_read  (id_mem_read),
        .mem_write (id_mem_write),
        .branch    (id_branch),
        .jal       (id_jal),
        .jalr      (id_jalr),
        .env_trap  (id_env_trap),
        .illegal   (id_illegal)
    );

    wire [31:0] id_imm;
    riscv_imm_gen u_imm (.instr(id_instr[31:7]), .imm_type(id_imm_type), .imm(id_imm));

    // Which source registers the instruction really reads. Used for the
    // load-use check (no needless stalls) and for the RVFI record.
    wire id_uses_rs1 = (id_op == OP_JALR) || (id_op == OP_BRANCH) || (id_op == OP_LOAD) ||
                       (id_op == OP_STORE) || (id_op == OP_IMM) || (id_op == OP_REG);
    wire id_uses_rs2 = (id_op == OP_BRANCH) || (id_op == OP_STORE) || (id_op == OP_REG);

    wire [31:0] rf_rs1, rf_rs2;
    riscv_regfile u_rf (
        .clk      (clk),
        .rs1      (id_rs1),
        .rs2      (id_rs2),
        .rs1_data (rf_rs1),
        .rs2_data (rf_rs2),
        .we       (wb_we),
        .rd       (wb_rd),
        .rd_data  (wb_data)
    );

    // WB writes at the end of this cycle; pass its value straight through
    wire [31:0] id_rs1_val = (wb_we && wb_rd != 5'd0 && wb_rd == id_rs1) ? wb_data : rf_rs1;
    wire [31:0] id_rs2_val = (wb_we && wb_rd != 5'd0 && wb_rd == id_rs2) ? wb_data : rf_rs2;

    // Load-use hazard: the load in EX produces its value only in MEM
    logic       ex_valid, ex_mem_read;
    logic [4:0] ex_rd;
    assign stall = id_valid && ex_valid && ex_mem_read && ex_rd != 5'd0 &&
                   ((id_uses_rs1 && ex_rd == id_rs1) || (id_uses_rs2 && ex_rd == id_rs2));

    // IF/ID update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_q     <= RESET_PC;
            id_valid <= 1'b0;
            id_pc    <= 32'd0;
            id_fresh <= 1'b1;
        end else begin
            id_fresh <= !stall;
            if (ex_trap || stop_q) begin
                id_valid <= 1'b0;                       // fetch nothing more
            end else if (!stall) begin
                id_valid <= 1'b1;
                id_pc    <= imem_addr;
                pc_q     <= imem_addr + 32'd4;
            end
        end
    end

    always_ff @(posedge clk) id_instr_q <= id_instr;

    // ==================================================================
    // EX
    // ==================================================================
    logic [31:0] ex_pc, ex_imm, ex_rs1_val, ex_rs2_val;
    logic [3:0]  ex_alu_op;
    logic [1:0]  ex_a_sel, ex_wb_sel;
    logic        ex_b_imm, ex_reg_write, ex_mem_write, ex_branch, ex_jal, ex_jalr;
    logic        ex_env_trap, ex_illegal;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [31:0] ex_instr;                     // these three feed only the RVFI trace
    logic        ex_uses_rs1, ex_uses_rs2;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [4:0]  ex_rs1, ex_rs2;
    logic [2:0]  ex_f3;

    // a bubble enters EX on a stall, a redirect (ID holds a wrong-path
    // instruction) or a trap in EX
    wire id_to_ex = id_valid && !stall && !redirect && !ex_trap;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ex_valid <= 1'b0;
        else        ex_valid <= id_to_ex;
    end

    always_ff @(posedge clk) begin
        ex_pc        <= id_pc;
        ex_instr     <= id_instr;
        ex_imm       <= id_imm;
        ex_rs1       <= id_rs1;
        ex_rs2       <= id_rs2;
        ex_rd        <= id_instr[11:7];
        ex_f3        <= id_instr[14:12];
        ex_rs1_val   <= id_rs1_val;
        ex_rs2_val   <= id_rs2_val;
        ex_alu_op    <= id_alu_op;
        ex_a_sel     <= id_a_sel;
        ex_b_imm     <= id_b_imm;
        ex_wb_sel    <= id_wb_sel;
        ex_reg_write <= id_reg_write;
        ex_mem_read  <= id_mem_read;
        ex_mem_write <= id_mem_write;
        ex_branch    <= id_branch;
        ex_jal       <= id_jal;
        ex_jalr      <= id_jalr;
        ex_env_trap  <= id_env_trap;
        ex_illegal   <= id_illegal;
        ex_uses_rs1  <= id_uses_rs1;
        ex_uses_rs2  <= id_uses_rs2;
    end

    // Forwarding: MEM (one instruction older) wins over WB (two older).
    // A load in MEM never forwards: the load-use stall guarantees its
    // consumer is not in EX yet.
    logic        mem_valid, mem_reg_write, mem_mem_read;
    logic [4:0]  mem_rd;
    logic [31:0] mem_result;
    wire mem_fwd_ok = mem_valid && mem_reg_write && !mem_mem_read && mem_rd != 5'd0;
    wire wb_fwd_ok  = wb_we && wb_rd != 5'd0;

    wire [31:0] ex_a_reg = (mem_fwd_ok && mem_rd == ex_rs1) ? mem_result :
                           (wb_fwd_ok  && wb_rd  == ex_rs1) ? wb_data    : ex_rs1_val;
    wire [31:0] ex_b_reg = (mem_fwd_ok && mem_rd == ex_rs2) ? mem_result :
                           (wb_fwd_ok  && wb_rd  == ex_rs2) ? wb_data    : ex_rs2_val;

    wire [31:0] alu_a = (ex_a_sel == A_PC)   ? ex_pc :
                        (ex_a_sel == A_ZERO) ? 32'd0 : ex_a_reg;
    wire [31:0] alu_b = ex_b_imm ? ex_imm : ex_b_reg;
    wire [31:0] alu_y;
    riscv_alu u_alu (.a(alu_a), .b(alu_b), .op(ex_alu_op), .y(alu_y));

    logic br_taken;
    always @(*) begin
        case (ex_f3)
            3'b000:  br_taken = (ex_a_reg == ex_b_reg);
            3'b001:  br_taken = (ex_a_reg != ex_b_reg);
            3'b100:  br_taken = ($signed(ex_a_reg) <  $signed(ex_b_reg));
            3'b101:  br_taken = ($signed(ex_a_reg) >= $signed(ex_b_reg));
            3'b110:  br_taken = (ex_a_reg <  ex_b_reg);
            3'b111:  br_taken = (ex_a_reg >= ex_b_reg);
            default: br_taken = 1'b0;
        endcase
    end

    wire [31:0] ex_pc4    = ex_pc + 32'd4;
    wire [31:0] ex_target = ex_jalr ? {alu_y[31:1], 1'b0} : (ex_pc + ex_imm);
    wire        ex_take   = ex_jal || ex_jalr || (ex_branch && br_taken);
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] ex_next   = ex_take ? ex_target : ex_pc4;     // RVFI only
    /* verilator lint_on UNUSEDSIGNAL */

    wire [3:0]  ex_mask;
    wire [31:0] ex_bus_addr, ex_bus_wdata;
    wire        ex_misaligned;
    /* verilator lint_off PINCONNECTEMPTY */
    riscv_lsu u_lsu_ex (
        .addr       (alu_y),
        .funct3     (ex_f3),
        .store_data (ex_b_reg),
        .bus_rdata  (32'd0),
        .bus_addr   (ex_bus_addr),
        .byte_mask  (ex_mask),
        .bus_wdata  (ex_bus_wdata),
        .load_data  (),
        .misaligned (ex_misaligned)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    assign ex_trap = ex_valid && (ex_illegal || ex_env_trap ||
                                  ((ex_mem_read || ex_mem_write) && ex_misaligned) ||
                                  (ex_take && ex_target[1:0] != 2'b00));
    wire ex_ok = ex_valid && !ex_trap;

    assign redirect    = ex_ok && ex_take;
    assign redirect_pc = ex_target;

    assign dmem_addr  = ex_bus_addr;
    assign dmem_wdata = ex_bus_wdata;
    assign dmem_re    = ex_ok && ex_mem_read;
    assign dmem_wmask = (ex_ok && ex_mem_write) ? ex_mask : 4'b0000;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)       stop_q <= 1'b0;
        else if (ex_trap) stop_q <= 1'b1;
    end

    // ==================================================================
    // MEM
    // ==================================================================
    logic [2:0]  mem_f3;
    logic [1:0]  mem_off;
    logic        mem_trap;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) mem_valid <= 1'b0;
        else        mem_valid <= ex_valid;
    end

    always_ff @(posedge clk) begin
        mem_trap      <= ex_trap;
        mem_rd        <= ex_rd;
        mem_reg_write <= ex_reg_write && !ex_trap;
        mem_mem_read  <= ex_mem_read && !ex_trap;
        mem_result    <= (ex_wb_sel == WB_PC4) ? ex_pc4 : alu_y;
        mem_f3        <= ex_f3;
        mem_off       <= alu_y[1:0];
    end

    wire [31:0] mem_load;
    /* verilator lint_off PINCONNECTEMPTY */
    riscv_lsu u_lsu_mem (
        .addr       ({30'd0, mem_off}),
        .funct3     (mem_f3),
        .store_data (32'd0),
        .bus_rdata  (dmem_rdata),
        .bus_addr   (),
        .byte_mask  (),
        .bus_wdata  (),
        .load_data  (mem_load),
        .misaligned ()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ==================================================================
    // WB
    // ==================================================================
    logic        wb_trap;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) wb_valid <= 1'b0;
        else        wb_valid <= mem_valid;
    end

    always_ff @(posedge clk) begin
        wb_rd        <= mem_rd;
        wb_reg_write <= mem_reg_write;
        wb_data      <= mem_mem_read ? mem_load : mem_result;
        wb_trap      <= mem_trap;
    end

    logic halt_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                   halt_q <= 1'b0;
        else if (wb_valid && wb_trap) halt_q <= 1'b1;
    end
    assign halted = halt_q;

    // ==================================================================
    // RVFI: the instruction in WB retires this cycle
    // ==================================================================
`ifdef RISCV_FORMAL
    // Trace-only copies of each instruction's details, carried MEM -> WB.
    // Only the operands an instruction really reads are reported: the
    // forwarding network does not cover unused fields, so their values
    // could be stale.
    logic [31:0] mem_pc, mem_next, mem_instr, mem_addr, mem_wdata_q, mem_a, mem_b;
    logic [3:0]  mem_rmask, mem_wmask;
    logic [4:0]  mem_rs1, mem_rs2;
    logic [31:0] wb_pc, wb_next, wb_instr, wb_addr, wb_mem_rdata, wb_mem_wdata, wb_a, wb_b;
    logic [3:0]  wb_rmask, wb_wmask;
    logic [4:0]  wb_rs1, wb_rs2;

    always_ff @(posedge clk) begin
        mem_pc       <= ex_pc;
        mem_next     <= ex_trap ? ex_pc : ex_next;
        mem_instr    <= ex_instr;
        mem_addr     <= ex_bus_addr;
        mem_rmask    <= dmem_re ? ex_mask : 4'b0000;
        mem_wmask    <= dmem_wmask;
        mem_wdata_q  <= ex_bus_wdata;
        mem_rs1      <= ex_uses_rs1 ? ex_rs1 : 5'd0;
        mem_rs2      <= ex_uses_rs2 ? ex_rs2 : 5'd0;
        mem_a        <= ex_uses_rs1 ? ex_a_reg : 32'd0;
        mem_b        <= ex_uses_rs2 ? ex_b_reg : 32'd0;

        wb_pc        <= mem_pc;
        wb_next      <= mem_next;
        wb_instr     <= mem_instr;
        wb_addr      <= mem_addr;
        wb_rmask     <= mem_rmask;
        wb_wmask     <= mem_wmask;
        wb_mem_rdata <= dmem_rdata;
        wb_mem_wdata <= mem_wdata_q;
        wb_rs1       <= mem_rs1;
        wb_rs2       <= mem_rs2;
        wb_a         <= mem_a;
        wb_b         <= mem_b;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)        rvfi_order <= 64'd0;
        else if (wb_valid) rvfi_order <= rvfi_order + 64'd1;
    end

    always @(*) begin
        rvfi_valid     = wb_valid;
        rvfi_insn      = wb_instr;
        rvfi_trap      = wb_trap;
        rvfi_halt      = wb_trap;
        rvfi_intr      = 1'b0;
        rvfi_mode      = 2'd3;
        rvfi_ixl       = 2'd1;
        rvfi_rs1_addr  = wb_rs1;
        rvfi_rs2_addr  = wb_rs2;
        rvfi_rs1_rdata = wb_a;
        rvfi_rs2_rdata = wb_b;
        rvfi_rd_addr   = wb_reg_write ? wb_rd : 5'd0;
        rvfi_rd_wdata  = (wb_reg_write && wb_rd != 5'd0) ? wb_data : 32'd0;
        rvfi_pc_rdata  = wb_pc;
        rvfi_pc_wdata  = wb_next;
        rvfi_mem_addr  = wb_addr;
        rvfi_mem_rmask = wb_rmask;
        rvfi_mem_wmask = wb_wmask;
        rvfi_mem_rdata = wb_mem_rdata;
        rvfi_mem_wdata = wb_mem_wdata;
    end
`endif

endmodule

`default_nettype wire
