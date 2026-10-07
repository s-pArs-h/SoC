`default_nettype none

// Load/store alignment unit (combinational).
//
// The data bus is word-addressed with byte enables. For stores the byte or
// halfword is replicated across the word and the write mask selects the
// lanes. For loads the addressed lanes are shifted down and sign- or
// zero-extended. Misaligned halfword/word accesses are reported so the core
// can trap instead of performing them.
module riscv_lsu (
    input  wire [31:0]  addr,        // byte address from the ALU
    input  wire [2:0]   funct3,      // size and signedness
    input  wire [31:0]  store_data,  // rs2
    input  wire [31:0]  bus_rdata,   // word read from memory
    output wire [31:0]  bus_addr,    // word-aligned address
    output logic [3:0]  byte_mask,   // lanes touched by this access
    output logic [31:0] bus_wdata,
    output logic [31:0] load_data,
    output logic        misaligned
);
    wire [1:0] off = addr[1:0];
    assign bus_addr = {addr[31:2], 2'b00};

    always @(*) begin
        case (funct3[1:0])
            2'b00: begin                                  // byte
                byte_mask  = 4'b0001 << off;
                bus_wdata  = {4{store_data[7:0]}};
                misaligned = 1'b0;
            end
            2'b01: begin                                  // halfword
                byte_mask  = off[1] ? 4'b1100 : 4'b0011;
                bus_wdata  = {2{store_data[15:0]}};
                misaligned = off[0];
            end
            default: begin                                // word
                byte_mask  = 4'b1111;
                bus_wdata  = store_data;
                misaligned = (off != 2'b00);
            end
        endcase
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] shifted = bus_rdata >> {off, 3'b000};   // only bits [15:0] are used
    /* verilator lint_on UNUSEDSIGNAL */

    always @(*) begin
        case (funct3)
            3'b000:  load_data = {{24{shifted[7]}},  shifted[7:0]};    // LB
            3'b001:  load_data = {{16{shifted[15]}}, shifted[15:0]};   // LH
            3'b100:  load_data = {24'b0, shifted[7:0]};                // LBU
            3'b101:  load_data = {16'b0, shifted[15:0]};               // LHU
            default: load_data = bus_rdata;                            // LW
        endcase
    end

endmodule

`default_nettype wire
