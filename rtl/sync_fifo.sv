`default_nettype none

// Synchronous FIFO, first-word-fall-through.
//
// rd_data shows the oldest entry whenever !empty; rd_en pops it. A push
// when full and a pop when empty are ignored. A simultaneous push and pop
// on a full FIFO is allowed: the pop frees the slot the push uses.
//
// DEPTH must be a power of two. The storage is a plain register array, so
// small FIFOs map onto LUT RAM or flip-flops.
module sync_fifo #(
    parameter W     = 8,
    parameter DEPTH = 16,
    // derived: do not override
    parameter AW    = $clog2(DEPTH)
) (
    input  wire          clk,
    input  wire          rst_n,

    input  wire          wr_en,
    input  wire [W-1:0]  wr_data,
    output wire          full,

    input  wire          rd_en,
    output wire [W-1:0]  rd_data,
    output wire          empty,

    output wire [AW:0]   count
);
    logic [W-1:0]  mem [0:DEPTH-1];
    logic [AW:0]   wptr, rptr;          // one extra bit tells full from empty

    assign count = wptr - rptr;
    assign empty = (wptr == rptr);
    assign full  = (count == (AW+1)'(DEPTH));

    wire do_rd = rd_en && !empty;
    wire do_wr = wr_en && (!full || do_rd);

    always_ff @(posedge clk) begin
        if (do_wr) mem[wptr[AW-1:0]] <= wr_data;
    end
    assign rd_data = mem[rptr[AW-1:0]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= '0;
            rptr <= '0;
        end else begin
            if (do_wr) wptr <= wptr + 1'b1;
            if (do_rd) rptr <= rptr + 1'b1;
        end
    end

`ifdef FORMAL
    always @(*) if (rst_n) assert(count <= (AW+1)'(DEPTH));

`ifdef SYNC_FIFO_DATA_CHECK
    // Data integrity for every word (formal/fifo.sby): the solver picks any
    // write position and any value. Once that value has been written at
    // that position, it must stay in the FIFO unchanged and come out on
    // rd_data when the read pointer reaches it. Because the position and
    // value are arbitrary, this covers every word ever written.
    /* verilator lint_off UNDRIVEN */           // chosen by the solver
    (* anyconst *) logic [AW:0]  f_addr;
    (* anyconst *) logic [W-1:0] f_data;
    /* verilator lint_on UNDRIVEN */
    logic f_held;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_held <= 1'b0;
        end else begin
            if (do_rd && rptr == f_addr)                     f_held <= 1'b0;
            if (do_wr && wptr == f_addr && wr_data == f_data) f_held <= 1'b1;
        end
    end

    always @(*) begin
        if (rst_n && f_held) begin
            assert((AW+1)'(f_addr - rptr) < count);         // still queued
            assert(mem[f_addr[AW-1:0]] == f_data);          // not overwritten
            if (do_rd && rptr == f_addr) assert(rd_data == f_data);
        end
    end

    always @(*) if (rst_n) cover(f_held && do_rd && rptr == f_addr && full);
`endif
`endif

endmodule

`default_nettype wire
