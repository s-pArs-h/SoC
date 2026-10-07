// Lloyd's K-means on the SoC, with the assignment step either on the
// accelerator (MODE_HW) or in software (MODE_SW). host/kmeans_ref.py is the
// bit-exact reference model of this file.
#ifndef KMEANS_H
#define KMEANS_H

#include <stdint.h>

#define KMAX        16
#define MAX_POINTS  4096        // software label buffer; the accelerator may hold fewer

typedef struct {
    // in
    unsigned mode;              // MODE_HW or MODE_SW
    unsigned k;                 // number of clusters (the accelerator's K)
    unsigned n;                 // points 0 .. n-1 of the point memory
    unsigned max_iter;
    // in: initial centroids; out: final centroids
    int32_t  cx[KMAX], cy[KMAX];
    // out
    unsigned iterations;        // assignment passes done
    unsigned converged;         // 1: the last update changed no centroid
    uint32_t count[KMAX];       // points per cluster in the last pass
    uint64_t sse;               // sum of squared distances in the last pass
    uint64_t total_cycles;      // whole run
    uint64_t assign_cycles;     // assignment passes, including register traffic
    uint64_t accel_cycles;      // accelerator busy cycles (MODE_HW only)
} kmeans_run;

extern uint8_t sw_labels[MAX_POINTS];

void kmeans(kmeans_run *r);

#endif
