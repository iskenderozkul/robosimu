"""A3 dimensionless delayed-reaching core (fair simulation).

The plant is the classical delayed integrator used in visuomotor control
studies: ``e_dot = -v`` with the gap ``e`` measured after a delay ``delta``.
Controllers are the Cartesian velocity laws from ``robosimu.controllers``
applied to the scalar gap (1-element vectors).

This module is the measurement instrument of the "Doğru Yol" programme
(``docs/dogru_yol_Faz0.md``): it is deliberately tiny, fast and free of the
six-DOF plant, so that delay-scaled laws and tuned baselines can be compared
on exactly the same footing.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, asdict

import numpy as np

from .controllers import (DelayScaledP, FixedCartesian, GovernorCartesian,
                          PidCartesian, make_controller)


@dataclass
class A3Outcome:
    """One dimensionless reach."""

    e0: float
    delay: float
    controller: str
    mt: float            # movement time: first entry into the eps band (s)
    overshoot: float     # max(0, -e_min)/e0 * 100  (%)
    final_err: float     # |e(T)| (m, dimensionless)
    settled: bool
    peak_speed: float

    def to_dict(self) -> dict:
        return asdict(self)


def simulate_reach(e0: float, delay: float, controller,
                   fps: float = 100.0, control_dt: float = 0.01,
                   eps: float = 0.003, duration: float = 3.0,
                   noise: float = 0.0, seed: int = 0,
                   jitter: float = 0.0) -> A3Outcome:
    """Simulate one delayed reach of the dimensionless plant.

    ``controller`` may be built with a *reported* delay that differs from the
    actual ``delay`` — that is exactly the mis-specified-delay condition of
    hypothesis H4.  ``noise`` (m) is additive measurement noise; ``jitter``
    (fraction of delay) makes the lag fluctuate uniformly per frame.
    """
    rng = np.random.default_rng(int(seed))
    lag = max(0, int(round(delay / control_dt)))
    buf: deque = deque([float(e0)] * (lag + 1), maxlen=lag + 1)
    e = float(e0)
    emin = e
    peak = 0.0
    entered = False
    mt = float(duration)
    steps = int(round(duration / control_dt))
    for k in range(steps):
        if jitter > 0.0 and lag > 0:
            spread = max(1, int(round(lag * jitter)))
            idx = int(rng.integers(-spread, spread + 1))
            idx = max(-lag, min(0, idx))
            e_meas = buf[idx]
        else:
            e_meas = buf[0]
        if noise > 0.0:
            e_meas = e_meas + float(rng.normal(0.0, noise))
        v3 = controller(np.array([e_meas, 0.0, 0.0]))
        v = float(np.asarray(v3, float).reshape(-1)[0])
        peak = max(peak, abs(v))
        e -= v * control_dt
        emin = min(emin, e)
        t = k * control_dt
        if not entered and abs(e) <= eps:
            entered = True
            mt = t
        buf.append(e)
    overshoot = max(0.0, -emin) / float(e0) * 100.0
    return A3Outcome(e0=e0, delay=float(delay),
                     controller=getattr(controller, "name", type(controller).__name__),
                     mt=mt, overshoot=overshoot, final_err=abs(e),
                     settled=abs(e) <= eps, peak_speed=peak)


def run_condition(name: str, delay: float, targets, seeds,
                  fps: float = 100.0, control_dt: float = 0.01,
                  eps: float = 0.003, vmax: float = 0.5, k=None, c=None,
                  clip_kappa: bool = False, noise: float = 0.0,
                  jitter: float = 0.0, reported_delay: float | None = None,
                  duration: float = 3.0):
    """Run one controller over a target set; returns a list of A3Outcome."""
    ctrl = make_controller(name, delay=(reported_delay if reported_delay is not None
                                        else delay),
                           fps=fps, control_dt=control_dt, eps=eps,
                           vmax=vmax, k=k, c=c, clip_kappa=clip_kappa)
    out = []
    for e0 in targets.amplitudes:
        for seed in targets.seeds:
            out.append(simulate_reach(e0, delay, ctrl, fps=fps,
                                      control_dt=control_dt, eps=eps,
                                      duration=duration, noise=noise,
                                      seed=seed, jitter=jitter))
    return out


def fitts_slope(outcomes, eps: float = 0.003) -> tuple[float, float, float]:
    """Fit MT = a + b * ID0 with ID0 = log2(1 + e0/eps); return (a, b, r2)."""
    pts = [(o.e0, o.mt) for o in outcomes if o.mt < 3.0]
    if len(pts) < 2:
        return float("nan"), float("nan"), float("nan")
    x = np.array([math.log2(1.0 + e0 / eps) for e0, _ in pts])
    y = np.array([mt for _, mt in pts])
    A = np.vstack([x, np.ones_like(x)]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    b, a = float(coef[0]), float(coef[1])
    yhat = a + b * x
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return a, b, r2
