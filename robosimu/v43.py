"""V4.3 zero-oracle controller core (Faz 1 + Faz 2).

Implements, on top of the existing plant, the specification in
``MASTER_SPEC_V4_3_ROBOSIMU`` §3.1–§3.6:

* ``TimestampedObservation`` — the only object a V4.3 controller ever sees:
  capture/arrival/now timestamps and pixel coordinates of hand and target
  (no metric gaps, no joint angles, no analytic Jacobian, no ``set_delay``).
* ``EfferenceRingBuffer`` — linear-time interpolation of the internal motor
  state to the exact frame-capture instants.
* ``OrderPreservingBroyden`` — compensation with the previous neck Jacobian
  *first*, arm/neck Jacobian updates *after*, with a Frobenius trust set.
* ``CorrelationAwareWindow`` — AR(1) calibration, upper variance envelope,
  admissible window set, smallest-window selection, stale flag.
* ``V43Controller`` — ties them together with the double-counting-free
  effective delay, the ``1/(e delta_eff)`` gain, the tanh-squared onset
  envelope and an error-scaled trunk weight.

Nothing here reads simulator internals; the plant only receives the Cartesian
velocity command returned by :meth:`V43Controller.step_obs`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------------------
# Zero-oracle observation packet
# ---------------------------------------------------------------------------
@dataclass
class TimestampedObservation:
    """Timestamped pixel observation — the V4.3 controller's only input."""

    t_cap: float                       # image capture time (s)
    t_arr: float                       # arrival (delivery) time (s)
    t_now: float                       # controller clock when processed (s)
    hand: np.ndarray                  # (2, 2): [[u_L, v_L], [u_R, v_R]]
    target: np.ndarray                # (2, 2): [[u_L, v_L], [u_R, v_R]]

    @property
    def delta_obs(self) -> float:
        """Observed base delay: now - capture."""
        return max(0.0, self.t_now - self.t_cap)

    @property
    def retinal_error(self) -> np.ndarray:
        """e_vis = (mean u, mean v, disparity) of hand minus target."""
        h = self.hand
        t = self.target
        du = 0.5 * (h[0, 0] + h[1, 0]) - 0.5 * (t[0, 0] + t[1, 0])
        dv = 0.5 * (h[0, 1] + h[1, 1]) - 0.5 * (t[0, 1] + t[1, 1])
        dd = (h[0, 0] - h[1, 0]) - (t[0, 0] - t[1, 0])
        return np.array([du, dv, dd], float)


# ---------------------------------------------------------------------------
# Efference ring buffer with linear time interpolation
# ---------------------------------------------------------------------------
class EfferenceRingBuffer:
    """Ring buffer of (t, q_hat) with linear interpolation at capture instants."""

    def __init__(self, dim: int = 5, capacity: int = 4096):
        self.dim = int(dim)
        self.capacity = int(capacity)
        self.t: list[float] = []
        self.q: list[np.ndarray] = []

    def push(self, t: float, q: np.ndarray) -> None:
        q = np.asarray(q, float).reshape(self.dim)
        if self.t and t <= self.t[-1]:
            return
        self.t.append(float(t))
        self.q.append(q.copy())
        if len(self.t) > self.capacity:
            self.t.pop(0)
            self.q.pop(0)

    def state_at(self, t: float) -> np.ndarray:
        """Linear interpolation at ``t``; constant extrapolation outside."""
        if not self.t:
            return np.zeros(self.dim)
        if t <= self.t[0]:
            return self.q[0].copy()
        if t >= self.t[-1]:
            return self.q[-1].copy()
        lo, hi = 0, len(self.t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.t[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = self.t[lo], self.t[hi]
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return (1.0 - a) * self.q[lo] + a * self.q[hi]

    def delta_between(self, t_a: float, t_b: float) -> np.ndarray:
        return self.state_at(t_b) - self.state_at(t_a)


# ---------------------------------------------------------------------------
# Order-preserving Broyden with Frobenius trust set
# ---------------------------------------------------------------------------
class OrderPreservingBroyden:
    """Compensate with the previous neck Jacobian, then update arm/neck."""

    def __init__(self, d_arm: int = 5, d_neck: int = 2,
                 lr: float = 0.25, lambda_broy: float = 1e-3,
                 gamma_leak: float = 1e-4, delta_j_max: float = 0.5,
                 theta_min: float = 1e-4, eta_dom: float = 0.5,
                 omega_sac: float = 0.2, eps_still: float = 1e-4,
                 theta_neck: float = 1e-4):
        self.d_arm = int(d_arm)
        self.d_neck = int(d_neck)
        self.lr = float(lr)
        self.lambda_broy = float(lambda_broy)
        self.gamma_leak = float(gamma_leak)
        self.delta_j_max = float(delta_j_max)
        self.theta_min = float(theta_min)
        self.eta_dom = float(eta_dom)
        self.omega_sac = float(omega_sac)
        self.eps_still = float(eps_still)
        self.theta_neck = float(theta_neck)
        self.J_r = np.zeros((3, self.d_arm))
        self.J_neck = np.zeros((3, self.d_neck))
        self.J_r0 = np.zeros((3, self.d_arm))
        self.J_neck0 = np.zeros((3, self.d_neck))

    def set_initial_jacobians(self, J_r0=None, J_neck0=None) -> None:
        if J_r0 is not None:
            self.J_r0 = np.asarray(J_r0, float).reshape(3, self.d_arm)
        if J_neck0 is not None:
            self.J_neck0 = np.asarray(J_neck0, float).reshape(3, self.d_neck)
        self.J_r = self.J_r0.copy()
        self.J_neck = self.J_neck0.copy()

    def compensate(self, delta_e: np.ndarray, delta_q_neck: np.ndarray) -> np.ndarray:
        """Step 1: remove the neck-induced optic shift with the previous J_neck."""
        return np.asarray(delta_e, float) - self.J_neck @ np.asarray(delta_q_neck, float)

    @staticmethod
    def _trust(J0: np.ndarray, J_tilde: np.ndarray, delta_max: float) -> np.ndarray:
        d = J_tilde - J0
        n = float(np.linalg.norm(d, "fro"))
        if n <= delta_max:
            return J_tilde
        return J0 + d * (delta_max / max(n, 1e-12))

    def update_arm(self, residual: np.ndarray, delta_q_r: np.ndarray,
                   qd_neck_norm: float = 0.0) -> bool:
        """Step 2: arm Jacobian update (gated). Returns True if updated."""
        dq = np.asarray(delta_q_r, float)
        nq = float(np.linalg.norm(dq))
        if nq <= self.theta_min:
            return False
        if qd_neck_norm > self.eta_dom * nq:
            return False
        J = self.J_r
        denom = self.lambda_broy + float(dq @ dq)
        J_tilde = (J + self.lr * np.outer(np.asarray(residual, float) - J @ dq, dq) / denom
                   - self.gamma_leak * (J - self.J_r0))
        self.J_r = self._trust(self.J_r0, J_tilde, self.delta_j_max)
        return True

    def update_neck(self, delta_e: np.ndarray, delta_q_neck: np.ndarray,
                    delta_q_r: np.ndarray) -> bool:
        """Step 3: neck Jacobian update, only while the arm is still."""
        dqn = np.asarray(delta_q_neck, float)
        if float(np.linalg.norm(delta_q_r)) > self.eps_still:
            return False
        if float(np.linalg.norm(dqn)) <= self.theta_neck:
            return False
        J = self.J_neck
        denom = self.lambda_broy + float(dqn @ dqn)
        J_tilde = (J + self.lr * np.outer(np.asarray(delta_e, float) - J @ dqn, dqn) / denom
                   - self.gamma_leak * (J - self.J_neck0))
        self.J_neck = self._trust(self.J_neck0, J_tilde, self.delta_j_max)
        return True

    def frobenius_residual(self) -> float:
        """Distance of the learned Jacobians from their initialisation."""
        return float(np.linalg.norm(self.J_r - self.J_r0, "fro")
                     + np.linalg.norm(self.J_neck - self.J_neck0, "fro"))


# ---------------------------------------------------------------------------
# Correlation-aware window selection
# ---------------------------------------------------------------------------
class CorrelationAwareWindow:
    """AR(1) calibration, upper variance envelope, admissible window set."""

    def __init__(self, alpha: float = 0.05, z: float | None = None,
                 sigma_px: float = 0.5):
        self.alpha = float(alpha)
        self.z = float(z) if z is not None else 1.959963984540054
        # Nominal prior before the calibration burst arrives: white noise of
        # half a pixel per axis, no correlation.
        self.sigma = np.full(3, float(sigma_px))
        self.rho = np.zeros(3)
        self.calibrated = False

    def calibrate(self, series: np.ndarray) -> None:
        """Estimate per-axis sigma and lag-1 correlation from a calibration burst."""
        x = np.asarray(series, float)
        if x.ndim != 2 or x.shape[0] < 8 or x.shape[1] != 3:
            raise ValueError("calibration series must be (N>=8, 3)")
        self.sigma = np.std(x, axis=0, ddof=1)
        xc = x - x.mean(axis=0)
        denom = np.sum(xc[:-1] ** 2, axis=0)
        self.rho = np.where(denom > 1e-12,
                            np.sum(xc[:-1] * xc[1:], axis=0) / np.maximum(denom, 1e-12),
                            0.0)
        self.rho = np.clip(self.rho, -0.999, 0.999)
        self.calibrated = True

    @staticmethod
    def variance(n: int, sigma: float, rho: float) -> float:
        """V(n) = sigma^2/n^2 [n + 2 sum_{l=1}^{n-1} (n-l) rho^l]."""
        n = int(n)
        if n <= 1:
            return float(sigma) ** 2
        ell = np.arange(1, n)
        s = float(np.sum((n - ell) * rho ** ell))
        return float(sigma) ** 2 / (n * n) * (n + 2.0 * s)

    def admissible(self, eps_px: np.ndarray, n_max: int) -> list[int]:
        out = []
        for j in range(3):
            eps = float(np.asarray(eps_px, float).reshape(-1)[j])
            for n in range(1, int(n_max) + 1):
                if self.z * math.sqrt(self.variance(n, self.sigma[j], self.rho[j])) <= eps:
                    out.append(n)
                    break
        return out or []

    def select(self, eps_px: np.ndarray, n_max: int) -> tuple[int, bool]:
        """Smallest admissible window across axes; degraded fallback n_max."""
        per_axis = self.admissible(eps_px, n_max)
        if per_axis:
            return int(min(per_axis)), True
        return int(max(1, n_max)), False

    @staticmethod
    def is_stale(t_now: float, t_arr: float, tau_stale: float) -> bool:
        return (t_now - t_arr) > tau_stale


# ---------------------------------------------------------------------------
# V4.3 controller
# ---------------------------------------------------------------------------
class V43Controller:
    """Zero-oracle visuomotor controller (specification §3.1–§3.6).

    Deliberately **not** a callable that takes a metric error vector: the only
    entry point is :meth:`step_obs`, which accepts a
    :class:`TimestampedObservation`.  The class exposes no ``set_delay`` and no
    attribute holding a metric gap, which is what the firewall test checks.
    """

    name = "v43"
    zero_oracle = True                  # marker read by the simulator
    outputs_joint_velocity = True       # 5 joint rates, not a Cartesian vector

    def __init__(self, neck=None, eps_px=0.35, alpha=0.05, tau_stale=0.05,
                 tau_onset=0.15, k_hi=40.0, vmax=0.5, eps=0.003,
                 lambda_dls=0.02, w_min=0.2, w_max=1.0, sigma_reach=0.10,
                 eps_reg=0.02, k_reg0=0.5, lr=0.25, delta_j_max=0.5,
                 fps=100.0, dt=0.01, gaze_comp=True, delay_align=True,
                 nominal_delay=0.10, fovea=(320.0, 240.0), focal_px=520.0,
                 k_neck=6.0, neck_max_rate=2.0):
        self.neck = neck
        self.eps_px = float(eps_px)
        self.tau_stale = float(tau_stale)
        self.tau_onset = float(tau_onset)
        self.k_hi = float(k_hi)
        self.vmax = float(vmax)
        self.eps = float(eps)
        self.lambda_dls = float(lambda_dls)
        self.w_min = float(w_min)
        self.w_max = float(w_max)
        self.sigma_reach = float(sigma_reach)
        self.eps_reg = float(eps_reg)
        self.k_reg0 = float(k_reg0)
        self.fps = float(fps)
        self.T = 1.0 / self.fps
        self.dt = float(dt)
        self.gaze_comp = bool(gaze_comp)
        self.delay_align = bool(delay_align)
        self.nominal_delay = float(nominal_delay)
        self.fovea = np.asarray(fovea, float)
        self.focal_px = float(focal_px)
        self.k_neck = float(k_neck)
        self.neck_max_rate = float(neck_max_rate)
        self.buffer = EfferenceRingBuffer(dim=5)
        self.neck_buffer = EfferenceRingBuffer(dim=2)
        self.broyden = OrderPreservingBroyden(d_arm=5, d_neck=2, lr=lr,
                                              delta_j_max=delta_j_max)
        self.window = CorrelationAwareWindow(alpha=alpha)
        # Generic (datasheet-level) body-schema prior, in px/rad.  It is the
        # same for every reach and is *not* evaluated at the current q.
        self.broyden.set_initial_jacobians(
            J_r0=np.array([[60.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, -110.0, -110.0, -55.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 40.0]]),
            J_neck0=np.array([[520.0, 0.0], [0.0, -520.0], [0.0, 0.0]]),
        )
        self.q_hat = np.zeros(5)
        self.q_neck = np.zeros(2)
        self.t_prev_cap = None
        self.t_start = None
        self.e0 = None
        self.syn_phase = "approach"
        self.alpha_syn = np.zeros(3)
        self.last_qd = np.zeros(5)
        self.last_dq_neck = np.zeros(2)
        self.info: dict = {}
        self.buffer.push(0.0, self.q_hat)
        self.neck_buffer.push(0.0, self.q_neck)

    # -- efference bookkeeping -------------------------------------------------
    def commit(self, t: float, qd_r: np.ndarray, dq_neck: np.ndarray,
               dt: float | None = None) -> None:
        """Integrate the *issued* command into the efference state.

        Called by the plant after it has applied (and possibly saturated) the
        command; ``t`` is the end-of-interval time.
        """
        dt = self.dt if dt is None else float(dt)
        self.q_hat = self.q_hat + np.asarray(qd_r, float).reshape(5) * dt
        self.q_neck = self.q_neck + np.asarray(dq_neck, float).reshape(2) * dt
        self.buffer.push(t, self.q_hat)
        self.neck_buffer.push(t, self.q_neck)

    def calibrate_windows(self, series: np.ndarray) -> None:
        self.window.calibrate(series)

    def set_nominal_delay(self, delay: float) -> None:
        """Set the *assumed* delay used when ``delay_align`` is False."""
        self.nominal_delay = max(1e-3, float(delay))

    # -- gaze ---------------------------------------------------------------
    def neck_command(self, obs: TimestampedObservation) -> np.ndarray:
        """Foveate the target: drive the retinal target offset to zero."""
        if not self.gaze_comp:
            return np.zeros(2)
        du = float(0.5 * (obs.target[0, 0] + obs.target[1, 0]) - self.fovea[0])
        dv = float(0.5 * (obs.target[0, 1] + obs.target[1, 1]) - self.fovea[1])
        pan = self.k_neck * du / self.focal_px
        tilt = -self.k_neck * dv / self.focal_px
        return np.clip(np.array([pan, tilt], float), -self.neck_max_rate, self.neck_max_rate)

    # -- main step -------------------------------------------------------------
    def step_obs(self, obs: TimestampedObservation) -> np.ndarray:
        """Consume one timestamped observation; return 5 joint rates (rad/s)."""
        if self.t_start is None:
            self.t_start = obs.t_now
            self.buffer.push(obs.t_cap, self.q_hat)
            self.neck_buffer.push(obs.t_cap, self.q_neck)
            self.t_prev_cap = obs.t_cap
        e_vis = obs.retinal_error
        if self.e0 is None:
            self.e0 = max(float(np.linalg.norm(e_vis)), 1e-6)

        # --- temporal bookkeeping: efference moved between capture instants
        dq_r = self.buffer.delta_between(self.t_prev_cap, obs.t_cap)
        dq_neck = self.neck_buffer.delta_between(self.t_prev_cap, obs.t_cap)
        self.t_prev_cap = obs.t_cap
        de = e_vis - self.info.get("e_prev", e_vis)

        # --- order-preserving Broyden: compensate first, update after
        resid = self.broyden.compensate(de, dq_neck)
        if float(np.linalg.norm(dq_r)) > 0.0:
            self.broyden.update_arm(resid, dq_r,
                                    qd_neck_norm=float(np.linalg.norm(dq_neck)))
        self.broyden.update_neck(de, dq_neck, dq_r)

        # --- delay / window / gain (spec §3.3)
        delta_obs = max(obs.delta_obs, 1e-3)
        delay_used = delta_obs if self.delay_align else self.nominal_delay
        n_max = max(1, int(math.floor(2.0 * delay_used / self.T)) + 1)
        n_star, admissible = self.window.select(np.array([self.eps_px] * 3), n_max)
        stale = CorrelationAwareWindow.is_stale(obs.t_now, obs.t_arr, self.tau_stale)
        delta_eff = delay_used + (n_star - 1) * self.T / 2.0
        k = 1.0 / (math.e * delta_eff)

        # --- velocity governor with tanh^2 onset envelope (spec §3.3)
        t_since = max(0.0, obs.t_now - self.t_start)
        psi = math.tanh(t_since / self.tau_onset) ** 2
        r = float(np.linalg.norm(e_vis))
        if stale or r <= self.eps:
            v_gov = np.zeros(3)
        else:
            v_mag = psi * min(self.vmax, self.k_hi * r, k * (self.eps_px + r))
            v_gov = -e_vis / r * v_mag

        # --- error-scaled trunk-arm WDLS (spec §3.5): joint-space command
        e_norm = max(r, 1e-6)
        w_trunk = self.w_min + self.w_max * math.exp(-(e_norm ** 2) / (self.sigma_reach ** 2))
        W_inv = np.diag([w_trunk, 1.0, 1.0, 1.0, 1.0])
        J = self.broyden.J_r
        A = J @ W_inv @ J.T + (self.lambda_dls ** 2) * np.eye(3)
        J_W = W_inv @ J.T @ np.linalg.pinv(A)
        qd = J_W @ v_gov
        # secondary regulation, faded near the target
        reg = -(self.k_reg0 * math.tanh(e_norm / self.eps_reg)) * \
              (np.eye(5) - J_W @ J) @ self.q_hat
        qd = qd + reg

        # --- phased hand synergy state machine (spec §3.6)
        xi = float(np.clip(r / self.e0, 0.0, 1.0))
        if self.syn_phase == "approach" and xi <= 0.25:
            self.syn_phase = "pre_grasp"
        elif self.syn_phase == "pre_grasp" and xi <= 0.08:
            self.syn_phase = "closure"
        elif self.syn_phase == "closure" and xi > 0.12:
            self.syn_phase = "pre_grasp"
        target_alpha = {"approach": np.array([0.2, 0.2, 0.2]),
                        "pre_grasp": np.array([1.0, 1.0, 1.0]),
                        "closure": np.array([0.0, 0.0, 0.0])}[self.syn_phase]
        self.alpha_syn = np.clip(self.alpha_syn + 0.05 * (target_alpha - self.alpha_syn), 0.0, 1.0)

        self.last_qd = np.asarray(qd, float).reshape(5)
        self.last_dq_neck = self.neck_command(obs)
        self.info = {"e_prev": e_vis.copy(), "e_vis": e_vis, "delta_obs": delta_obs,
                     "delay_used": delay_used, "delta_eff": delta_eff,
                     "n_star": n_star, "n_max": n_max, "admissible": admissible,
                     "stale": stale, "k": k, "psi": psi,
                     "w_trunk": w_trunk, "synergy_phase": self.syn_phase,
                     "broyden_fro": self.broyden.frobenius_residual()}
        return self.last_qd
