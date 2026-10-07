`default_nettype none

// K-means clustering accelerator core.
//
// For every 2-D point streamed in, the core finds the nearest of K centroids
// (assignment step) and accumulates per-cluster coordinate sums and counts,
// which is everything a host needs for the centroid-update step:
//     new_centroid[k] = sum[k] / count[k]
// so one run of this core = one full Lloyd iteration over the data set.
//
// Interfaces
//   cfg_*      centroid registers, writable only while idle
//   start      pulse with num_points; ignored while busy
//   s_*        point stream in, valid/ready handshake
//   m_*        per-point result stream out, valid/ready handshake
//   acc_*, sse per-cluster statistics, valid when done pulses and until the next start
//   done       one-cycle pulse after the last result has been accepted
//
// Pipeline (latency = 3 + log2(K) cycles, one point per cycle):
//   [diff] -> [square] -> [sum] -> [min tree: log2(K) stages] -> m_*
// The whole pipeline advances together and stalls when the output holds a
// result that is not being accepted (adv = !m_valid || m_ready).
// Note: this makes s_ready depend combinationally on m_ready. That is fine
// at this size; a skid buffer on the output would break the path if needed.
module kmeans_core #(
    parameter DATA_W = 16,              // signed coordinate width
    parameter K      = 4,               // number of clusters: power of two, >= 2
    parameter CNT_W  = 16,              // up to 2^CNT_W - 1 points per run
    // derived widths: do not override
    parameter IDX_W  = $clog2(K),
    parameter DIST_W = 2*DATA_W + 1,
    parameter SUM_W  = DATA_W + CNT_W,
    parameter SSE_W  = DIST_W + CNT_W
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     cfg_we,
    input  wire [IDX_W-1:0]         cfg_idx,
    input  wire signed [DATA_W-1:0] cfg_cx,
    input  wire signed [DATA_W-1:0] cfg_cy,

    input  wire                     start,
    input  wire [CNT_W-1:0]         num_points,
    output wire                     busy,
    output wire                     done,

    input  wire                     s_valid,
    output wire                     s_ready,
    input  wire signed [DATA_W-1:0] s_x,
    input  wire signed [DATA_W-1:0] s_y,

    output wire                     m_valid,
    input  wire                     m_ready,
    output wire [IDX_W-1:0]         m_cluster,
    output wire [DIST_W-1:0]        m_dist,

    input  wire [IDX_W-1:0]         acc_idx,
    output wire signed [SUM_W-1:0]  acc_sum_x,
    output wire signed [SUM_W-1:0]  acc_sum_y,
    output wire [CNT_W-1:0]         acc_count,
    output wire [SSE_W-1:0]         sse
);
    localparam LVLS  = $clog2(K);
    localparam DEPTH = 3 + LVLS;               // pipeline latency in cycles

    // ------------------------------------------------------------------
    // Control
    // ------------------------------------------------------------------
    localparam [1:0] S_IDLE = 2'd0,
                     S_RUN  = 2'd1,
                     S_DONE = 2'd2;

    logic [1:0]       state;
    logic [CNT_W-1:0] n_q;                     // points in this run
    logic [CNT_W-1:0] in_cnt;                  // points accepted
    logic [CNT_W-1:0] out_cnt;                 // results accepted
    logic [DEPTH-1:0] vld;                     // valid bit per pipeline stage

    wire run      = (state == S_RUN);
    wire adv      = !vld[DEPTH-1] || m_ready;
    assign s_ready = run && adv && (in_cnt != n_q);
    wire in_fire  = s_valid && s_ready;

    assign m_valid = vld[DEPTH-1];
    wire out_fire = m_valid && m_ready;
    wire last_out = out_fire && (out_cnt == n_q - 1'b1);

    assign busy = (state != S_IDLE);
    assign done = (state == S_DONE);

    // Stage i loads only when the pipeline advances and its input is valid.
    wire [DEPTH-1:0] st_en = {DEPTH{adv}} & {vld[DEPTH-2:0], in_fire};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            n_q     <= '0;
            in_cnt  <= '0;
            out_cnt <= '0;
            vld     <= '0;
        end else begin
            if (adv) vld <= {vld[DEPTH-2:0], in_fire};

            case (state)
                S_IDLE: begin
                    if (start) begin
                        n_q     <= num_points;
                        in_cnt  <= '0;
                        out_cnt <= '0;
                        state   <= (num_points == '0) ? S_DONE : S_RUN;
                    end
                end
                S_RUN: begin
                    if (in_fire)  in_cnt  <= in_cnt + 1'b1;
                    if (out_fire) out_cnt <= out_cnt + 1'b1;
                    if (last_out) state   <= S_DONE;
                end
                default: state <= S_IDLE;      // S_DONE lasts exactly one cycle
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Centroid registers (no reset: software programs them before a run)
    // ------------------------------------------------------------------
    logic signed [DATA_W-1:0] cen_x [0:K-1];
    logic signed [DATA_W-1:0] cen_y [0:K-1];

    always_ff @(posedge clk) begin
        if (cfg_we && !busy) begin
            cen_x[cfg_idx] <= cfg_cx;
            cen_y[cfg_idx] <= cfg_cy;
        end
    end

    // ------------------------------------------------------------------
    // Distance processing elements, one per centroid
    // ------------------------------------------------------------------
    wire [K*DIST_W-1:0] leaf_dist;

    genvar k;
    generate
        for (k = 0; k < K; k = k + 1) begin : g_pe
            distance_calc #(
                .DATA_W (DATA_W),
                .DIST_W (DIST_W)
            ) u_pe (
                .clk  (clk),
                .en   (st_en[2:0]),
                .px   (s_x),
                .py   (s_y),
                .cx   (cen_x[k]),
                .cy   (cen_y[k]),
                .dist_o (leaf_dist[k*DIST_W +: DIST_W])
            );
        end
    endgenerate

    min_tree #(
        .K      (K),
        .DIST_W (DIST_W),
        .IDX_W  (IDX_W),
        .LVLS   (LVLS)
    ) u_tree (
        .clk       (clk),
        .en        (st_en[DEPTH-1:3]),
        .leaf_dist (leaf_dist),
        .min_dist  (m_dist),
        .min_idx   (m_cluster)
    );

    // ------------------------------------------------------------------
    // Point coordinates travel alongside the pipeline for the update step
    // ------------------------------------------------------------------
    logic signed [DATA_W-1:0] px_q [0:DEPTH-1];
    logic signed [DATA_W-1:0] py_q [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (st_en[0]) begin
            px_q[0] <= s_x;
            py_q[0] <= s_y;
        end
    end

    genvar s;
    generate
        for (s = 1; s < DEPTH; s = s + 1) begin : g_pt
            always_ff @(posedge clk) begin
                if (st_en[s]) begin
                    px_q[s] <= px_q[s-1];
                    py_q[s] <= py_q[s-1];
                end
            end
        end
    endgenerate

    wire signed [DATA_W-1:0] out_x = px_q[DEPTH-1];
    wire signed [DATA_W-1:0] out_y = py_q[DEPTH-1];
    wire signed [SUM_W-1:0]  out_x_ext = $signed({{(SUM_W-DATA_W){out_x[DATA_W-1]}}, out_x});
    wire signed [SUM_W-1:0]  out_y_ext = $signed({{(SUM_W-DATA_W){out_y[DATA_W-1]}}, out_y});

    // ------------------------------------------------------------------
    // Per-cluster accumulators for the centroid-update step
    // ------------------------------------------------------------------
    logic signed [SUM_W-1:0] sum_x [0:K-1];
    logic signed [SUM_W-1:0] sum_y [0:K-1];
    logic        [CNT_W-1:0] cnt   [0:K-1];
    logic        [SSE_W-1:0] sse_q;
    integer j;

    always_ff @(posedge clk) begin
        if (!busy && start) begin
            for (j = 0; j < K; j = j + 1) begin
                sum_x[j] <= '0;
                sum_y[j] <= '0;
                cnt[j]   <= '0;
            end
            sse_q <= '0;
        end else if (out_fire) begin
            sum_x[m_cluster] <= sum_x[m_cluster] + out_x_ext;
            sum_y[m_cluster] <= sum_y[m_cluster] + out_y_ext;
            cnt[m_cluster]   <= cnt[m_cluster] + 1'b1;
            sse_q            <= sse_q + {{CNT_W{1'b0}}, m_dist};
        end
    end

    assign acc_sum_x = sum_x[acc_idx];
    assign acc_sum_y = sum_y[acc_idx];
    assign acc_count = cnt[acc_idx];
    assign sse       = sse_q;

`ifdef FORMAL
    // ------------------------------------------------------------------
    // Inductive invariants (checked by formal/kmeans.sby, task "prove").
    // They describe every reachable state of the control logic, which is
    // what lets k-induction prove the handshake properties for all time
    // and all run lengths, not just for a bounded number of cycles.
    // ------------------------------------------------------------------
    integer f_b, f_inflight;
    always @(*) begin
        f_inflight = 0;
        for (f_b = 0; f_b < DEPTH; f_b = f_b + 1)
            f_inflight = f_inflight + {31'b0, vld[f_b]};
    end

    always @(*) begin
        if (rst_n) begin
            assert(state != 2'd3);
            assert(in_cnt <= n_q);
            assert(out_cnt <= in_cnt);
            assert({{(32-CNT_W){1'b0}}, in_cnt - out_cnt} == f_inflight);
            if (state != S_RUN)  assert(vld == '0);
            if (state == S_RUN)  assert(out_cnt < n_q);
            if (state == S_DONE) assert(in_cnt == n_q && out_cnt == n_q);
        end
    end
`endif

endmodule

`default_nettype wire
