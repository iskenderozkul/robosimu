"""Point-to-point reference trajectories with configurable ramps.

Two profiles are provided:

* trapezoidal (``jmax = 0``): constant acceleration, exact arrival;
* S-curve (``jmax > 0``): jerk-limited, so motor torque and drive current ramp
  instead of stepping — the setting used to test driver ramping and braking.

The planner returns sampled ``(t, q, qd, qdd)`` arrays for a scalar move; for a
multi-axis arm the same profile can be applied per joint or to a Cartesian
path parameter.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _integrate(q0, v0, a0, segments, dt):
    """Integrate jerk segments analytically.

    A segment is ``(duration, jerk)`` or ``(duration, jerk, accel_start)``;
    the third entry pins the acceleration at the segment start (used for the
    constant-acceleration phases of a trapezoidal profile).
    """
    t = [0.0]
    q = [q0]
    v = [v0]
    a = [a0]
    ct, cq, cv, ca = 0.0, q0, v0, a0
    for seg in segments:
        duration, jerk = seg[0], seg[1]
        if len(seg) > 2:
            ca = float(seg[2])
        n = max(1, int(round(duration / dt)))
        h = duration / n
        for _ in range(n):
            cq = cq + cv * h + ca * h * h / 2.0 + jerk * h ** 3 / 6.0
            cv = cv + ca * h + jerk * h * h / 2.0
            ca = ca + jerk * h
            ct += h
            t.append(ct)
            q.append(cq)
            v.append(cv)
            a.append(ca)
    return np.asarray(t), np.asarray(q), np.asarray(v), np.asarray(a)


def _accel_time(vp, amax, jmax):
    """Time to accelerate from rest to ``vp`` under jerk and acceleration limits."""
    if jmax <= 0:
        return vp / amax
    t1 = amax / jmax
    v1 = 0.5 * jmax * t1 * t1
    if vp >= 2.0 * v1:
        return 2.0 * t1 + (vp - 2.0 * v1) / amax
    return 2.0 * math.sqrt(vp / jmax)


def _accel_distance(vp, amax, jmax):
    """Distance covered while accelerating from rest to ``vp`` (symmetric profile)."""
    return 0.5 * vp * _accel_time(vp, amax, jmax)


def _peak_velocity(distance, vmax, amax, jmax):
    """Largest velocity reachable for a move of ``distance`` (no cruise)."""
    if 2.0 * _accel_distance(vmax, amax, jmax) <= distance:
        return vmax, (distance - 2.0 * _accel_distance(vmax, amax, jmax)) / vmax
    lo, hi = 0.0, vmax
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if 2.0 * _accel_distance(mid, amax, jmax) < distance:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi), 0.0


def _accel_segments(vp, amax, jmax):
    """Jerk segments for a symmetric accel from rest to ``vp``."""
    if jmax <= 0:
        return [(vp / amax, 0.0, amax)], amax
    t1 = amax / jmax
    v1 = 0.5 * jmax * t1 * t1
    if vp >= 2.0 * v1:
        t2 = (vp - 2.0 * v1) / amax
        return [(t1, jmax), (t2, 0.0), (t1, -jmax)], amax
    t1 = math.sqrt(vp / jmax)
    return [(t1, jmax), (t1, -jmax)], jmax * t1


@dataclass
class MoveProfile:
    """Sampled point-to-point move."""
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    distance: float
    peak_velocity: float
    duration: float
    profile: str


def plan_move(q0: float, q1: float, vmax: float, amax: float, jmax: float = 0.0,
              dt: float = 0.001) -> MoveProfile:
    """Plan a single-axis move from ``q0`` to ``q1``."""
    distance = abs(q1 - q0)
    sign = 1.0 if q1 >= q0 else -1.0
    if distance < 1e-12:
        z = np.zeros(1)
        return MoveProfile(z, np.array([q0]), z, z, 0.0, 0.0, 0.0, "zero")

    vp, cruise = _peak_velocity(distance, vmax, amax, jmax)
    seg, ap = _accel_segments(vp, amax, jmax)
    decel = [(s[0], -s[1], -s[2]) if len(s) > 2 else (s[0], -s[1]) for s in seg]

    if cruise > 1e-12:
        segments = seg + [(cruise, 0.0, 0.0)] + decel
        label = "s-curve-cruise" if jmax > 0 else "trapezoid-cruise"
    else:
        segments = seg + decel
        label = "s-curve-tri" if jmax > 0 else "trapezoid-tri"

    t, q, qd, qdd = _integrate(0.0, 0.0, 0.0, segments, dt)
    # normalise the final position exactly (integration round-off)
    if q.size > 1:
        q = q * (distance / abs(q[-1]))
        qd = qd * (distance / abs(q[-1]))
        qdd = qdd * (distance / abs(q[-1]))
    q = q0 + sign * q
    qd = sign * qd
    qdd = sign * qdd
    return MoveProfile(t=t, q=q, qd=qd, qdd=qdd, distance=distance,
                       peak_velocity=vp, duration=float(t[-1]), profile=label)


def plan_joint_move(q0, q1, vmax, amax, jmax=0.0, dt=0.001):
    """Plan a multi-axis move; the slowest axis sets the common time base.

    Each axis is planned with scaled limits so the axes start and stop together
    (a true synchronised multi-axis ramp).
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    vmax = np.asarray(vmax, float)
    amax = np.asarray(amax, float)
    distances = np.abs(q1 - q0)
    scale = float(np.max(distances)) if distances.max() > 0 else 1.0
    profiles = []
    for i in range(q0.size):
        d = distances[i]
        s = d / scale if scale > 0 else 0.0
        p = plan_move(0.0, d, vmax[i] * (s ** 0.5 if s > 0 else 1.0),
                      amax[i] * (s ** 0.5 if s > 0 else 1.0), jmax, dt)
        profiles.append(p)
    # common duration: resample the longest onto a shared time base
    durations = [p.duration for p in profiles]
    T = max(durations) if durations else 0.0
    n = int(round(T / dt)) + 1
    ts = np.linspace(0.0, T, n)
    qs = np.zeros((n, q0.size))
    qds = np.zeros((n, q0.size))
    qdds = np.zeros((n, q0.size))
    for i, p in enumerate(profiles):
        qs[:, i] = np.interp(ts, p.t, p.q)
        qds[:, i] = np.interp(ts, p.t, np.clip(p.qd, -vmax[i], vmax[i]))
        qdds[:, i] = np.interp(ts, p.t, p.qdd)
        qs[:, i] = q0[i] + np.sign(q1[i] - q0[i]) * qs[:, i]
    return ts, qs, qds, qdds
