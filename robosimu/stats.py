"""Statistical helpers used by the reaching experiments.

The conventions follow the usual human-motor-control practice (Soukoreff &
MacKenzie 2004, Fitts 1954) and the pre-registered analysis style of the
project: bootstrap confidence intervals for movement time, Wilson intervals
for success/overshoot rates, Holm step-down correction across a family of
comparisons, and effect sizes on the log scale.
"""
from __future__ import annotations

import math

import numpy as np


def bootstrap_ci(values, n_boot: int = 10000, alpha: float = 0.05,
                 seed: int = 0, stat=np.mean) -> tuple[float, float]:
    """Percentile bootstrap CI for a statistic of ``values``.

    The default statistic is the mean; pass ``np.median`` when the point
    estimate reported alongside the interval is a median, so that interval and
    estimate refer to the same quantity.
    """
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return (float("nan"), float("nan"))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, v.size, size=(n_boot, v.size))
    boots = np.asarray(stat(v[idx], axis=1), float)
    lo = float(np.percentile(boots, 100.0 * alpha / 2.0))
    hi = float(np.percentile(boots, 100.0 * (1.0 - alpha / 2.0)))
    return (lo, hi)


def wilson_ci(successes: int, n: int, alpha: float = 0.05) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion."""
    if n <= 0:
        return (float("nan"), float("nan"))
    p = successes / n
    z = 1.959963984540054 if alpha == 0.05 else float(_z_value(alpha))
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / denom
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def _z_value(alpha: float) -> float:
    """Two-sided standard-normal quantile (Abramowitz-Stegun rational fit)."""
    p = 1.0 - alpha / 2.0
    if p <= 0.0 or p >= 1.0:
        return 1.959963984540054
    t = math.sqrt(-2.0 * math.log(1.0 - p))
    c = [2.515517, 0.802853, 0.010328]
    d = [1.432788, 0.189269, 0.001308]
    return t - (c[0] + c[1] * t + c[2] * t * t) / (
        1.0 + d[0] * t + d[1] * t * t + d[2] * t * t * t)


def holm_bonferroni(p_values, alpha: float = 0.05) -> np.ndarray:
    """Holm step-down adjusted p-values for a family of comparisons."""
    p = np.asarray(p_values, float)
    m = p.size
    order = np.argsort(p)
    adj = np.empty(m)
    running = 0.0
    for rank, idx in enumerate(order):
        running = max(running, (m - rank) * float(p[idx]))
        adj[idx] = min(1.0, running)
    return adj


def cohens_d(a, b) -> float:
    """Cohen's d on log-transformed samples (recommended for movement times)."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    a = a[np.isfinite(a) & (a > 0)]
    b = b[np.isfinite(b) & (b > 0)]
    if a.size < 2 or b.size < 2:
        return float("nan")
    la, lb = np.log(a), np.log(b)
    va, vb = la.var(ddof=1), lb.var(ddof=1)
    pooled = math.sqrt(((a.size - 1) * va + (b.size - 1) * vb) / (a.size + b.size - 2))
    if pooled <= 0:
        return float("nan")
    return float((la.mean() - lb.mean()) / pooled)


def permutation_test(a, b, n_perm: int = 5000, seed: int = 0) -> float:
    """Two-sided permutation test on the difference of log means.

    Used for movement-time comparisons: the log transform makes the test
    scale-free (a ratio of times) and the permutation removes any normality
    assumption.
    """
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    a = a[np.isfinite(a) & (a > 0)]
    b = b[np.isfinite(b) & (b > 0)]
    if a.size < 2 or b.size < 2:
        return float("nan")
    la, lb = np.log(a), np.log(b)
    obs = abs(float(la.mean() - lb.mean()))
    pooled = np.concatenate([la, lb])
    rng = np.random.default_rng(seed)
    hits = 0
    for _ in range(n_perm):
        rng.shuffle(pooled)
        d = abs(float(pooled[:a.size].mean() - pooled[a.size:].mean()))
        if d >= obs:
            hits += 1
    return (hits + 1.0) / (n_perm + 1.0)


def fitts_id(distance: float, effective_width: float) -> float:
    """Shannon form of the index of difficulty: ID = log2(1 + D/We)."""
    if effective_width <= 0:
        return float("inf")
    return math.log2(1.0 + distance / effective_width)


def effective_width(sigma: float) -> float:
    """Soukoreff-MacKenzie effective width for a pointing task: We = 4.133 sigma."""
    return 4.133 * sigma


def throughput(distance: float, effective_width: float, movement_time: float) -> float:
    """Throughput in bits per second: ID / MT."""
    if movement_time <= 0:
        return float("nan")
    return fitts_id(distance, effective_width) / movement_time


def median(values) -> float:
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    return float(np.median(v)) if v.size else float("nan")


def summarise_mt(movement_times, n_boot: int = 10000, seed: int = 0) -> dict:
    """Median movement time with a bootstrap CI."""
    mt = np.asarray(movement_times, float)
    med = median(mt)
    lo, hi = bootstrap_ci(mt, n_boot=n_boot, seed=seed, stat=np.median)
    return {"median_s": med, "ci95_s": [lo, hi], "n": int(np.isfinite(mt).sum())}


def summarise_rate(successes: int, n: int) -> dict:
    """Rate with a Wilson CI."""
    lo, hi = wilson_ci(successes, n)
    return {"rate": successes / n if n else float("nan"),
            "ci95": [lo, hi], "successes": int(successes), "n": int(n)}
