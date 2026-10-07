`default_nettype none

// Memory-mapped K-means accelerator: kmeans_core plus its own point and
// label memories, so that a data set crosses the bus once and every Lloyd
// iteration after that runs at one point per clock.
//
//   offset 0x0_0000  control and status registers (below)
//   offset 0x1_0000  point memory: word i = point i, {y[31:16], x[15:0]},
//                    read/write by the CPU
//   offset 0x2_0000  label memory: byte i = cluster of point i after the
//                    last run, four labels per word, read-only for the CPU
//
// Registers (byte offsets)
//   0x000 INFO        RO  {PTS_AW, DATA_W, K, version}
//   0x004 CTRL        W   bit 0: start a run over points 0 .. NUM_POINTS-1
//   0x008 STATUS      R   bit 0 busy, bit 1 done (since the last start),
//                         bit 2 error (start while busy, or NUM_POINTS too big)
//                     W   write 1 to bit 1 / bit 2 to clear it
//   0x00C NUM_POINTS  RW  points in the next run, at most 2^PTS_AW
//   0x010 CYCLES      RO  clock cycles taken by the last run
//   0x014 SSE_LO      RO  sum of squared distances of the last run, bits 31:0
//   0x018 SSE_HI      RO  bits 63:32
//   0x100 + 4k        W   CENTROID[k] = {y[31:16], x[15:0]}, ignored while busy
//   0x200 + 16k       RO  SUM_X[k]; +4 SUM_Y[k]; +8 COUNT[k]
//
// A run: the streamer reads the point memory through its second port and
// feeds kmeans_core; each result is written to the label memory as it
// comes out. The core's result output is always accepted, so a run takes
// NUM_POINTS + pipeline latency + a few cycles of start-up.
//
// Bus: every access completes in one cycle. Read data is registered and is
// valid in the cycle after the request, as the pipelined core expects.
//
// Supported: DATA_W = 16 (two coordinates per word), CNT_W = 16 (so the
// sums are 32 bits), K <= 32, PTS_AW <= 14.
module kmeans_accel #(
    parameter K      = 4,
    parameter DATA_W = 16,
    parameter PTS_AW = 12,              // point capacity 2^PTS_AW
    parameter CNT_W  = 16,
    // derived: do not override
    parameter IDX_W  = $clog2(K)
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         sel,
    input  wire         re,
    input  wire [3:0]   we,
    input  wire [17:0]  addr,           // byte offset within the 256 KiB window
    input  wire [31:0]  wdata,
    output logic [31:0] rdata
);
    localparam DEPTH  = 1 << PTS_AW;
    localparam DIST_W = 2*DATA_W + 1;
    localparam SUM_W  = DATA_W + CNT_W;
    localparam SSE_W  = DIST_W + CNT_W;
    localparam FIFO_D = 4;
    localparam FCW    = $clog2(FIFO_D) + 1;

    wire [1:0]  region = addr[17:16];
    wire        rd     = sel && re;
    wire        wr     = sel && (we != 4'b0000);

    // ------------------------------------------------------------------
    // Control registers
    // ------------------------------------------------------------------
    logic              done_q, err_q;
    logic [PTS_AW:0]   num_q;
    logic [31:0]       cycles_q;

    wire core_busy, core_done;
    wire reg_wr     = wr && region == 2'd0;
    wire ctrl_start = reg_wr && addr[11:0] == 12'h004 && we[0] && wdata[0];
    wire start      = ctrl_start && !core_busy;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_q   <= 1'b0;
            err_q    <= 1'b0;
            num_q    <= '0;
            cycles_q <= '0;
        end else begin
            if (reg_wr && addr[11:0] == 12'h008 && we[0]) begin
                if (wdata[1]) done_q <= 1'b0;
                if (wdata[2]) err_q  <= 1'b0;
            end
            if (reg_wr && addr[11:0] == 12'h00C && we == 4'b1111) begin
                if (wdata > 32'(DEPTH)) err_q <= 1'b1;      // refused
                else                     num_q <= wdata[PTS_AW:0];
            end
            if (ctrl_start && core_busy) err_q <= 1'b1;

            if (start) begin
                done_q   <= 1'b0;
                cycles_q <= '0;
            end else if (core_busy) begin
                cycles_q <= cycles_q + 1'b1;
            end
            if (core_done) done_q <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Point memory (port A: CPU, port B: streamer)
    // ------------------------------------------------------------------
    logic [31:0] pmem [0:DEPTH-1];
    logic [31:0] pmem_a_q, pmem_b_q;
    wire  [PTS_AW-1:0] pa = addr[PTS_AW+1:2];

    integer b;
    always_ff @(posedge clk) begin
        if (sel && region == 2'd1) begin
            for (b = 0; b < 4; b = b + 1)
                if (we[b]) pmem[pa][8*b +: 8] <= wdata[8*b +: 8];
            pmem_a_q <= pmem[pa];
        end
    end

    // Streamer: reads points in order and keeps a small FIFO in front of the
    // core topped up. A read is issued only when the FIFO is sure to have
    // room for its data when it arrives a cycle later (entries + reads in
    // flight < depth): credit-based flow control around the memory latency.
    // In the steady state one point is read, queued and consumed per cycle.
    logic [PTS_AW:0]  rd_idx;           // next point to read
    logic [PTS_AW:0]  run_n;            // points in the current run
    logic             rd_pend;          // a read was issued last cycle
    wire  [FCW-1:0]   f_count;
    wire              f_empty, f_full;
    wire  [31:0]      f_data;
    wire              core_s_ready;

    wire [FCW-1:0] used  = f_count + {{(FCW-1){1'b0}}, rd_pend};
    wire           issue = (rd_idx != run_n) && (used < FCW'(FIFO_D));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_idx  <= '0;
            run_n   <= '0;
            rd_pend <= 1'b0;
        end else begin
            rd_pend <= issue && !start;
            if (start) begin
                rd_idx <= '0;
                run_n  <= num_q;
            end else if (issue) begin
                rd_idx <= rd_idx + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (issue) pmem_b_q <= pmem[rd_idx[PTS_AW-1:0]];
    end

    sync_fifo #(.W(32), .DEPTH(FIFO_D)) u_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_en(rd_pend), .wr_data(pmem_b_q), .full(f_full),
        .rd_en(core_s_ready), .rd_data(f_data), .empty(f_empty),
        .count(f_count)
    );

    // ------------------------------------------------------------------
    // K-means core
    // ------------------------------------------------------------------
    wire              m_valid;
    wire [IDX_W-1:0]  m_cluster;
    wire [DIST_W-1:0] m_dist;
    wire [IDX_W-1:0]  acc_idx = addr[IDX_W+3:4];
    wire signed [SUM_W-1:0] acc_sum_x, acc_sum_y;
    wire [CNT_W-1:0]  acc_count;
    wire [SSE_W-1:0]  sse;

    wire cfg_we = reg_wr && addr[11:8] == 4'h1 && we == 4'b1111;

    kmeans_core #(.DATA_W(DATA_W), .K(K), .CNT_W(CNT_W)) u_core (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_idx(addr[IDX_W+1:2]),
        .cfg_cx(wdata[15:0]), .cfg_cy(wdata[31:16]),
        .start(start), .num_points(CNT_W'(num_q)),
        .busy(core_busy), .done(core_done),
        .s_valid(!f_empty), .s_ready(core_s_ready),
        .s_x(f_data[15:0]), .s_y(f_data[31:16]),
        .m_valid(m_valid), .m_ready(1'b1),
        .m_cluster(m_cluster), .m_dist(m_dist),
        .acc_idx(acc_idx),
        .acc_sum_x(acc_sum_x), .acc_sum_y(acc_sum_y), .acc_count(acc_count),
        .sse(sse)
    );

    // ------------------------------------------------------------------
    // Label memory (port A: CPU reads, port B: results)
    // ------------------------------------------------------------------
    logic [31:0]       lmem [0:DEPTH/4-1];
    logic [31:0]       lmem_a_q;
    logic [PTS_AW-1:0] out_idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)       out_idx <= '0;
        else if (start)   out_idx <= '0;
        else if (m_valid) out_idx <= out_idx + 1'b1;
    end

    always_ff @(posedge clk) begin
        if (m_valid)
            lmem[out_idx[PTS_AW-1:2]][8*out_idx[1:0] +: 8] <= 8'(m_cluster);
    end

    always_ff @(posedge clk) begin
        if (rd && region == 2'd2) lmem_a_q <= lmem[addr[PTS_AW-1:2]];
    end

    // ------------------------------------------------------------------
    // Register read port
    // ------------------------------------------------------------------
    logic [1:0]  region_q;
    logic [31:0] reg_q;

    always_ff @(posedge clk) begin
        if (rd) begin
            region_q <= region;
            reg_q    <= 32'h0;
            if (addr[11:9] == 3'b001) begin                 // 0x200 - 0x3FF
                case (addr[3:2])
                    2'd0:    reg_q <= 32'(acc_sum_x);
                    2'd1:    reg_q <= 32'(acc_sum_y);
                    2'd2:    reg_q <= 32'(acc_count);
                    default: reg_q <= 32'h0;
                endcase
            end else begin
                case (addr[11:0])
                    12'h000: reg_q <= {8'(PTS_AW), 8'(DATA_W), 8'(K), 8'h01};
                    12'h008: reg_q <= {29'b0, err_q, done_q, core_busy};
                    12'h00C: reg_q <= 32'(num_q);
                    12'h010: reg_q <= cycles_q;
                    12'h014: reg_q <= sse[31:0];
                    12'h018: reg_q <= 32'(sse[SSE_W-1:32]);
                    default: reg_q <= 32'h0;
                endcase
            end
        end
    end

    always @(*) begin
        case (region_q)
            2'd0:    rdata = reg_q;
            2'd1:    rdata = pmem_a_q;
            2'd2:    rdata = lmem_a_q;
            default: rdata = 32'h0;
        endcase
    end

    // The winning distance is not stored per point (the SSE already sums
    // it); full is implied by the credit check.
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused = &{1'b0, m_dist, f_full, addr};
    /* verilator lint_on UNUSEDSIGNAL */

`ifdef FORMAL
    // Streamer invariants (formal/accel.sby, task prove):
    //   * the FIFO entries plus the read in flight never exceed its depth,
    //     so the data of every issued read finds room: nothing is dropped
    //   * the streamer never reads past the end of the run
    always @(*) begin
        if (rst_n) begin
            assert({1'b0, used} <= (FCW+1)'(FIFO_D));
            assert(!(rd_pend && f_full && !core_s_ready));
            assert(rd_idx <= run_n);
        end
    end

`ifdef KMEANS_ACCEL_BMC
    // A new run starts with nothing left over from the previous one
    // (bounded check, task bmc): this relies on the core having consumed
    // exactly the points the streamer read, a fact about two blocks that is
    // checked over every run sequence the solver can build within the bound.
    always @(*) if (rst_n && start) assert(f_empty && !rd_pend && rd_idx == run_n);

    // the bound is long enough to start a new run after a finished one
    always @(*) if (rst_n) cover(start && done_q && run_n >= 3);
`endif
`endif

endmodule

`default_nettype wire
