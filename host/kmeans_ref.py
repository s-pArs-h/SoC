"""Bit-exact reference model of the firmware's K-means (sw/kmeans.c).

Integer arithmetic throughout: squared Euclidean distances, ties to the
lowest cluster index (as the accelerator's arg-min tree), new centroid =
mean rounded to nearest with halves away from zero, an empty cluster keeps
its centroid, and the run stops when an update changes no centroid.
"""
from __future__ import annotations

import random


def assign(points, cents):
    """One assignment pass: labels, per-cluster sums and counts, SSE."""
    k = len(cents)
    sx, sy, cnt = [0] * k, [0] * k, [0] * k
    labels, sse = [], 0
    for x, y in points:
        best, best_d = 0, None
        for i, (cx, cy) in enumerate(cents):
            d = (x - cx) ** 2 + (y - cy) ** 2
            if best_d is None or d < best_d:
                best, best_d = i, d
        labels.append(best)
        sx[best] += x
        sy[best] += y
        cnt[best] += 1
        sse += best_d
    return labels, sx, sy, cnt, sse


def round_div(s: int, c: int) -> int:
    q = (abs(s) + c // 2) // c
    return q if s >= 0 else -q


def lloyd(points, init, max_iter):
    cents = [tuple(c) for c in init]
    res = dict(iterations=0, converged=False, labels=[], counts=[0] * len(cents), sse=0)
    while res["iterations"] < max_iter:
        labels, sx, sy, cnt, sse = assign(points, cents)
        res["iterations"] += 1
        new = [(round_div(sx[i], cnt[i]), round_div(sy[i], cnt[i])) if cnt[i] else cents[i]
               for i in range(len(cents))]
        changed = new != cents
        cents = new
        res.update(labels=labels, counts=cnt, sse=sse)
        if not changed:
            res["converged"] = True
            break
    res["centroids"] = cents
    return res


def kmeans_pp(points, k, rng: random.Random):
    """k-means++ initialisation: each new centroid is a data point chosen with
    probability proportional to its squared distance from the nearest centroid
    already chosen. Spreads the starting centroids out, which avoids most of
    the poor local minima a purely random start can fall into."""
    cents = [rng.choice(points)]
    d2 = [(x - cents[0][0]) ** 2 + (y - cents[0][1]) ** 2 for x, y in points]
    while len(cents) < k:
        total = sum(d2)
        if total == 0:                                  # fewer distinct points than k
            cents.append(rng.choice(points))
            continue
        r = rng.randrange(total)
        acc = 0
        for i, d in enumerate(d2):
            acc += d
            if acc > r:
                break
        c = points[i]
        cents.append(c)
        d2 = [min(d, (x - c[0]) ** 2 + (y - c[1]) ** 2) for d, (x, y) in zip(d2, points)]
    return cents


def blobs(n, k, spread, rng: random.Random, lo=-32768, hi=32767):
    """n points around k random centres (Gaussian, clipped to int16)."""
    margin = 3 * spread
    centres = [(rng.randint(lo + margin, hi - margin), rng.randint(lo + margin, hi - margin))
               for _ in range(k)]
    pts = []
    for i in range(n):
        cx, cy = centres[i % k]
        pts.append((max(lo, min(hi, round(rng.gauss(cx, spread)))),
                    max(lo, min(hi, round(rng.gauss(cy, spread))))))
    rng.shuffle(pts)
    return pts
