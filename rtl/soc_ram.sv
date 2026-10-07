`default_nettype none

// Dual-port RAM for code and data, written in the form FPGA tools map onto
// block RAM: port A fetches instructions (read-only, every cycle), port B
// serves loads and stores (byte write enables). Both reads are registered,
// so read data arrives the cycle after the address, as the pipeline expects.
module soc_ram #(
    parameter WORDS     = 4096,
    parameter INIT_FILE = "",
    // derived: do not override
    parameter AW        = $clog2(WORDS)
) (
    input  wire          clk,

    input  wire [AW-1:0] a_addr,
    output logic [31:0]  a_rdata,

    input  wire          b_en,
    input  wire [3:0]    b_we,
    input  wire [AW-1:0] b_addr,
    input  wire [31:0]   b_wdata,
    output logic [31:0]  b_rdata
);
    logic [31:0] mem [0:WORDS-1];

    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always_ff @(posedge clk) a_rdata <= mem[a_addr];

    integer i;
    always_ff @(posedge clk) begin
        if (b_en) begin
            for (i = 0; i < 4; i = i + 1)
                if (b_we[i]) mem[b_addr][8*i +: 8] <= b_wdata[8*i +: 8];
            b_rdata <= mem[b_addr];
        end
    end

endmodule

`default_nettype wire
