"""Cartesian velocity controllers used in the reaching experiments."""
from __future__ import annotations

import math
from collections import deque

import numpy as np


class FixedCartesian:
    """v = k e, capped at vmax.

    ``k`` is tuned once at the nominal 30 ms condition (see
    ``scripts/tune_baselines.py``) and then frozen.
    """

    name = "fixed"

    def __init__(self, k: float = 10.0, vmax: float = 0.5, eps: float = 0.003):
        self.k = float(k)
        self.vmax = float(vmax)
        self.eps = float(eps)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        if float(np.linalg.norm(e)) <= self.eps:
            return np.zeros(3)
        v = self.k * e
        n = float(np.linalg.norm(v))
        if n > self.vmax:
            v *= self.vmax / n
        return v


class GovernorCartesian:
    """Delay-derived per-axis speed limit (the Iskender governor).

    v_j = min(k_hi |ef_j|, k_j (eps + |ef_j|)),
    k_j = kappa / delta_eff_j,  delta_eff_j = delta + n_j T / 2,
    n_j = clip(ceil((2 sigma_j / eps)^2), 1, floor(2 delta / T)),  kappa = 1/e.

    The boxcar of ``n_j`` frames is what turns measurement noise into extra
    effective delay; the gain is computed from the measured delay and frame
    period, never tuned per condition.
    """

    name = "governor"

    def __init__(self, delay: float, fps: float, eps: float = 0.003,
                 kappa: float = 1.0 / math.e, k_hi: float = 40.0,
                 vmax: float = 0.5, sigma=0.0005, axes: int = 3):
        self.delay = float(delay)
        self.T = 1.0 / float(fps)
        self.eps = float(eps)
        self.kappa = float(min(kappa, 1.0 / math.e))
        self.k_hi = float(k_hi)
        self.vmax = float(vmax)
        sig = np.broadcast_to(np.atleast_1d(np.asarray(sigma, float)),
                              (axes,)).astype(float)
        self.n_req = np.ceil((2.0 * sig / self.eps) ** 2).astype(int)
        self._buf = None
        self._apply(self.delay)

    def _apply(self, delay: float) -> None:
        self.delay = max(0.0, float(delay))
        n_max = max(1, int(math.floor(2.0 * self.delay / self.T)))
        self.n = np.maximum(1, np.minimum(self.n_req, n_max))
        self.n_max = int(self.n.max())
        self.delta_eff = self.delay + self.n_max * self.T / 2.0
        self.k = self.kappa / self.delta_eff
        if self._buf is not None and self._buf.maxlen != self.n_max:
            self._buf = deque(self._buf, maxlen=self.n_max)

    def set_delay(self, delay: float) -> None:
        """Recompute the gain schedule from the currently measured delay."""
        self._apply(delay)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        if self._buf is None:
            self._buf = deque(maxlen=int(self.n_max))
        self._buf.append(e.copy())
        filt = np.mean(np.asarray(self._buf), axis=0)
        r = float(np.linalg.norm(filt))
        if r <= self.eps:
            return np.zeros(e.size)
        v = min(self.k_hi * r, self.k * (self.eps + r), self.vmax)
        return filt / r * v


def _cap(v: np.ndarray, vmax: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > vmax:
        return v * (vmax / n)
    return v


class AdaptiveCartesian:
    """ViSP-style adaptive gain: lambda = lambda0 / (1 + |e| / e_ref)."""

    name = "adaptive"

    def __init__(self, lam0: float = 14.0, e_ref: float = 0.05, vmax: float = 0.5,
                 eps: float = 0.003):
        self.lam0 = float(lam0)
        self.e_ref = float(e_ref)
        self.vmax = float(vmax)
        self.eps = float(eps)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        r = float(np.linalg.norm(e))
        if r <= self.eps:
            return np.zeros(3)
        lam = self.lam0 / (1.0 + r / self.e_ref)
        return _cap(lam * e, self.vmax)


class SmithCartesian:
    """Smith predictor with an internal delay-free model of the error loop.

    The model is the unit-gain Cartesian plant ``e_dot = -v``.  The delayed
    model output is compared with the delayed measurement and the mismatch is
    added to the current model state:

        xhat(t) = xm(t) + [e_meas(t) - xm(t - delta)]

    The control law acts on ``xhat``, which is the standard Smith structure.
    The model matches the plant here, so this is deliberately a hard baseline.
    """

    name = "smith"

    def __init__(self, delay: float, control_dt: float = 0.01, k: float = 20.0,
                 vmax: float = 0.5, eps: float = 0.003):
        self.delay = float(delay)
        self.dt = float(control_dt)
        self.k = float(k)
        self.vmax = float(vmax)
        self.eps = float(eps)
        self.n_delay = max(1, int(round(self.delay / self.dt)))
        self.xm = None
        self._buf = None

    def set_delay(self, delay: float) -> None:
        """Track the delay reported for the current frame (symmetry with the
        governor: both delay-aware laws receive the same information)."""
        self.delay = max(0.0, float(delay))
        n = max(1, int(round(self.delay / self.dt)))
        if n != self.n_delay:
            self.n_delay = n
            if self._buf is not None:
                self._buf = deque(self._buf, maxlen=n + 1)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e_meas = np.asarray(error, float)
        if self.xm is None:
            self.xm = e_meas.copy()
            self._buf = deque([self.xm.copy() for _ in range(self.n_delay + 1)],
                              maxlen=self.n_delay + 1)
        xhat = self.xm + (e_meas - self._buf[0])
        r = float(np.linalg.norm(xhat))
        v = np.zeros(3) if r <= self.eps else _cap(self.k * xhat, self.vmax)
        self.xm = self.xm - v * self.dt      # advance the delay-free model
        self._buf.append(self.xm.copy())
        return v


class BroydenCartesian:
    """IBVS with an online estimate of the interaction-matrix scale.

    The controller starts with a mis-calibrated depth (``scale = 0.7``) and
    updates the scalar interaction gain from delayed measurements using a
    Broyden-style rank-one update.  Commands are matched to the measurement
    delay through a short command history, which is what the plain Broyden
    update misses in a delayed loop.
    """

    name = "broyden"

    def __init__(self, delay: float, control_dt: float = 0.01, lam: float = 6.0,
                 vmax: float = 0.5, eps: float = 0.003, alpha: float = 0.25,
                 init_scale: float = 0.7):
        self.delay = float(delay)
        self.dt = float(control_dt)
        self.lam = float(lam)
        self.vmax = float(vmax)
        self.eps = float(eps)
        self.alpha = float(alpha)
        self.scale = float(init_scale)
        self.n_delay = max(1, int(round(self.delay / self.dt)))
        self._cmds = deque(maxlen=self.n_delay + 2)
        self.e_prev = None

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        if self.e_prev is not None and len(self._cmds) >= self.n_delay + 1:
            dp = -(e - self.e_prev)          # observed motion (delayed)
            v_applied = self._cmds[-(self.n_delay + 1)]
            denom = float(v_applied @ v_applied)
            if denom > 1e-8:
                ratio = float(dp @ v_applied) / denom
                if 0.2 < ratio < 5.0:        # reject outliers and transients
                    self.scale += self.alpha * (ratio - self.scale)
        self.e_prev = e.copy()
        r = float(np.linalg.norm(e))
        if r <= self.eps:
            v = np.zeros(3)
        else:
            v = _cap(self.lam * e / max(self.scale, 0.2), self.vmax)
        self._cmds.append(v.copy())
        return v


class LqrCartesian:
    """Cartesian LQR for a double integrator, tuned for the nominal loop.

    State ``[e, e_dot]``, input ``v_dot`` (Cartesian acceleration), cost
    ``Q = diag(q_pos, q_vel)`` and ``R = r I``.  The scalar continuous-time
    Riccati equation has the closed form

        k1 = sqrt(q_pos / r)
        k2 = sqrt((2 sqrt(r q_pos) + q_vel) / r)

    The controller has no delay model and no delay-adaptive gain; it is the
    model-based comparator.  Its acceleration command is integrated to the
    velocity command, so it carries the extra phase lag that a real LQR
    velocity servo would have.  The default weights put the gains at
    k1 = 20, k2 = 8, chosen to minimise movement time at the nominal 30 ms
    delay with no overshoot; the gains are then frozen for every condition.
    """

    name = "lqr"

    def __init__(self, control_dt: float, q_pos: float = 9.0,
                 q_vel: float = 0.5, r: float = 0.01, vmax: float = 0.5,
                 eps: float = 0.003, tau_d: float = 0.05):
        self.dt = float(control_dt)
        self.q_pos, self.q_vel, self.r = float(q_pos), float(q_vel), float(r)
        self.k1 = math.sqrt(self.q_pos / self.r)
        self.k2 = math.sqrt((2.0 * math.sqrt(self.r * self.q_pos)
                             + self.q_vel) / self.r)
        self.vmax = float(vmax)
        self.eps = float(eps)
        self.tau_d = float(tau_d)
        self.v = np.zeros(3)
        self.e_prev = None
        self.e_dot = np.zeros(3)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        if self.e_prev is None:
            self.e_prev = e.copy()
        a = self.dt / (self.tau_d + self.dt)
        self.e_dot = (1.0 - a) * self.e_dot + a * (e - self.e_prev) / self.dt
        self.e_prev = e.copy()
        if float(np.linalg.norm(e)) <= self.eps:
            self.v = np.zeros(3)
            return self.v
        acc = self.k1 * e + self.k2 * self.e_dot
        self.v = _cap(self.v + self.dt * acc, self.vmax)
        return self.v


class PidCartesian:
    """Cartesian PID with filtered derivative and conditional anti-windup."""

    name = "pid"

    def __init__(self, control_dt: float, kp: float = 10.0, ki: float = 0.5,
                 kd: float = 0.02, vmax: float = 0.5, eps: float = 0.003,
                 integral_limit: float = 0.02, tau_d: float = 0.05):
        self.dt = float(control_dt)
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.vmax = float(vmax)
        self.eps = float(eps)
        self.integral_limit = float(integral_limit)
        self.tau_d = float(tau_d)
        self.integral = np.zeros(3)
        self.de = np.zeros(3)
        self.e_prev = None

    def __call__(self, error: np.ndarray) -> np.ndarray:
        e = np.asarray(error, float)
        r = float(np.linalg.norm(e))
        if self.e_prev is None:
            self.e_prev = e.copy()
        # first-order filtered derivative, so measurement noise is not amplified
        a = self.dt / (self.tau_d + self.dt)
        self.de = (1.0 - a) * self.de + a * (e - self.e_prev) / self.dt
        self.e_prev = e.copy()
        v = self.kp * e + self.ki * self.integral + self.kd * self.de
        saturated = float(np.linalg.norm(v)) > self.vmax
        if not saturated:                      # conditional integration
            self.integral = np.clip(self.integral + e * self.dt,
                                    -self.integral_limit, self.integral_limit)
            v = self.kp * e + self.ki * self.integral + self.kd * self.de
        if r <= self.eps:
            self.integral = np.zeros(3)
            return np.zeros(3)
        return _cap(v, self.vmax)


def make_controller(name: str, delay: float = 0.06, fps: float = 100.0,
                    control_dt: float = 0.01, eps: float = 0.003,
                    vmax: float = 0.5, k: float | None = None,
                    noise: float = 0.0005):
    """Build a Cartesian velocity controller by name.

    ``k`` overrides the controller's own gain when given; the defaults are the
    parameters selected once at the nominal 30 ms condition (see
    ``scripts/tune_baselines.py``) and are frozen for every other condition.
    """
    name = name.lower()
    if name == "fixed":
        return FixedCartesian(**({} if k is None else {"k": k}),
                              vmax=vmax, eps=eps)
    if name == "governor":
        return GovernorCartesian(delay=delay, fps=fps, eps=eps, vmax=vmax,
                                 sigma=noise)
    if name == "adaptive":
        return AdaptiveCartesian(**({} if k is None else {"lam0": k}),
                                 vmax=vmax, eps=eps)
    if name == "smith":
        return SmithCartesian(delay=delay, control_dt=control_dt,
                              **({} if k is None else {"k": k}),
                              vmax=vmax, eps=eps)
    if name == "broyden":
        return BroydenCartesian(delay=delay, control_dt=control_dt,
                                **({} if k is None else {"lam": k}),
                                vmax=vmax, eps=eps)
    if name == "pid":
        return PidCartesian(control_dt=control_dt,
                            **({} if k is None else {"kp": k}),
                            vmax=vmax, eps=eps)
    if name == "lqr":
        return LqrCartesian(control_dt=control_dt, vmax=vmax, eps=eps)
    raise ValueError(f"unknown controller {name!r}")
