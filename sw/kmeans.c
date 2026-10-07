#include "kmeans.h"
#include "protocol.h"
#include "soc.h"

uint8_t sw_labels[MAX_POINTS];

typedef struct {
    int32_t  sum_x[KMAX], sum_y[KMAX];
    uint32_t count[KMAX];
    uint64_t sse;
} pass_stats;

// ----------------------------------------------------------------------
// Assignment step on the accelerator: program the centroids, start a run
// over the point memory, wait, read back the per-cluster sums.
// ----------------------------------------------------------------------
static void assign_hw(kmeans_run *r, pass_stats *s)
{
    for (unsigned k = 0; k < r->k; k++)
        ACC_CENTROID(k) = ((uint32_t)(uint16_t)r->cy[k] << 16) | (uint16_t)r->cx[k];

    ACC_CTRL = ACC_CTRL_START;
    while (!(ACC_STATUS & (ACC_ST_DONE | ACC_ST_ERR)))
        ;

    for (unsigned k = 0; k < r->k; k++) {
        s->sum_x[k] = (int32_t)ACC_SUM_X(k);
        s->sum_y[k] = (int32_t)ACC_SUM_Y(k);
        s->count[k] = ACC_COUNT(k);
    }
    uint32_t lo = ACC_SSE_LO;
    s->sse = ((uint64_t)ACC_SSE_HI << 32) | lo;
    r->accel_cycles += ACC_CYCLES;
}

// ----------------------------------------------------------------------
// The same step in software, for comparison. Exact for the full 16-bit
// range: |dx| <= 65535, so dx^2 fits in 32 unsigned bits, and the sum of
// two squares needs 33. Ties go to the lowest index, as in the hardware.
// RV32I has no multiply instruction, so each square is a libgcc call.
// ----------------------------------------------------------------------
static void assign_sw(kmeans_run *r, pass_stats *s)
{
    for (unsigned k = 0; k < r->k; k++) {
        s->sum_x[k] = 0;
        s->sum_y[k] = 0;
        s->count[k] = 0;
    }
    s->sse = 0;

    for (unsigned i = 0; i < r->n; i++) {
        uint32_t p = ACC_PMEM[i];
        int32_t  x = (int16_t)(p & 0xFFFF);
        int32_t  y = (int16_t)(p >> 16);

        unsigned best_k = 0;
        uint64_t best_d = ~(uint64_t)0;
        for (unsigned k = 0; k < r->k; k++) {
            int32_t  dx = x - r->cx[k];
            int32_t  dy = y - r->cy[k];
            uint32_t ax = dx < 0 ? -(uint32_t)dx : (uint32_t)dx;
            uint32_t ay = dy < 0 ? -(uint32_t)dy : (uint32_t)dy;
            uint64_t d  = (uint64_t)(ax * ax) + (ay * ay);
            if (d < best_d) {
                best_d = d;
                best_k = k;
            }
        }
        sw_labels[i] = (uint8_t)best_k;
        s->sum_x[best_k] += x;
        s->sum_y[best_k] += y;
        s->count[best_k] += 1;
        s->sse += best_d;
    }
}

// Mean rounded to the nearest integer, halves away from zero.
static int32_t round_div(int32_t sum, uint32_t count)
{
    uint32_t mag = sum < 0 ? -(uint32_t)sum : (uint32_t)sum;
    uint32_t q   = (mag + count / 2) / count;
    return sum < 0 ? -(int32_t)q : (int32_t)q;
}

void kmeans(kmeans_run *r)
{
    pass_stats s;
    uint64_t t0 = cycles();

    r->iterations    = 0;
    r->converged     = 0;
    r->assign_cycles = 0;
    r->accel_cycles  = 0;
    if (r->mode == MODE_HW) {
        ACC_STATUS     = ACC_ST_DONE | ACC_ST_ERR;      // clear stale flags
        ACC_NUM_POINTS = r->n;
    }

    while (r->iterations < r->max_iter) {
        uint64_t a0 = cycles();
        if (r->mode == MODE_HW)
            assign_hw(r, &s);
        else
            assign_sw(r, &s);
        r->assign_cycles += cycles() - a0;
        r->iterations++;

        // update step: move each non-empty cluster's centroid to its mean
        unsigned changed = 0;
        for (unsigned k = 0; k < r->k; k++) {
            r->count[k] = s.count[k];
            if (s.count[k] == 0)
                continue;                       // empty cluster keeps its centroid
            int32_t nx = round_div(s.sum_x[k], s.count[k]);
            int32_t ny = round_div(s.sum_y[k], s.count[k]);
            changed |= (nx != r->cx[k]) | (ny != r->cy[k]);
            r->cx[k] = nx;
            r->cy[k] = ny;
        }
        r->sse = s.sse;
        if (!changed) {
            r->converged = 1;
            break;
        }
    }
    r->total_cycles = cycles() - t0;
}
