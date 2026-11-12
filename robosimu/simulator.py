"""Closed-loop reaching simulation with delayed visual feedback."""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

from .dynamics import (bias_torque, friction_torque, gravity_torque, mass_matrix, rnea)
from .kinematics import end_effector_position, jacobian
from .model import ArmModel
from .motors import MotorState, drive


@dataclass
class SimConfig:
    duration: float = 2.0
    dt: float = 0.001           # target inner joint-servo / torque-update step
    control_dt: float = 0.01    # outer (visual) loop period
    delay: float = 0.06
    noise: float = 0.0005        # scalar, or a 3-vector (x, y, z) for anisotropic noise
    delay_jitter: float = 0.0    # std-dev of the visual delay (s)
    delay_schedule: tuple = ()   # ((t0, delay), ...) piecewise-constant delay jumps
    seed: int = 0
    wn: float = 10.0          # joint PD natural frequency (rad/s)
    zeta: float = 1.0         # joint PD damping ratio
    payload: float = 0.0
    eps: float = 0.003
    regen: bool = False
    max_qd: float = 0.6       # reference joint speed limit (rad/s)
    max_qdd: float = 3.0      # reference joint acceleration limit (rad/s^2)


@dataclass
class ReachResult:
    t: np.ndarray
    p: np.ndarray
    error: np.ndarray
    tau: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    power: np.ndarray
    energy: float
    motor: list[list[MotorState]]
    p0: np.ndarray
    target: np.ndarray
    config: SimConfig
    controller: str
    metrics: dict = field(default_factory=dict)


def _overshoot_pct(p, p0, target) -> tuple[float, float]:
    """Return (overshoot % of the commanded step, penetration in metres)."""
    u = target - p0
    d = float(np.linalg.norm(u))
    if d < 1e-9:
        return 0.0, 0.0
    u = u / d
    s = (p - p0) @ u
    peak = float(np.max(s))
    return (max(0.0, 100.0 * (peak - d) / d), max(0.0, peak - d))


def simulate_reach(model: ArmModel, q0, target, controller, cfg: SimConfig | None = None) -> ReachResult:
    cfg = cfg or SimConfig()
    model = model.with_payload(cfg.payload) if cfg.payload > 0 else model
    q0 = np.asarray(q0, float)
    target = np.asarray(target, float)
    p0 = end_effector_position(model, q0)
    rng = np.random.default_rng(cfg.seed)

    n_sub = max(1, int(round(cfg.control_dt / cfg.dt)))
    servo_dt = cfg.control_dt / n_sub      # inner joint-servo / torque-update step
    n_ctrl = int(round(cfg.duration / cfg.control_dt))
    t_arr = np.arange(n_ctrl) * cfg.control_dt

    noise_vec = None if np.isscalar(cfg.noise) else np.asarray(cfg.noise, float)
    if noise_vec is not None and noise_vec.shape != (3,):
        raise ValueError("noise must be a scalar or a 3-vector (x, y, z)")

    delays = np.full(n_ctrl, float(cfg.delay))
    for t0, d in cfg.delay_schedule:
        delays[t_arr >= float(t0)] = float(d)
    if cfg.delay_jitter > 0:
        delays = np.clip(delays + cfg.delay_jitter * rng.standard_normal(n_ctrl), 0.0, None)
    lags = np.rint(delays / servo_dt).astype(int)

    limits = np.array([l.limits for l in model.links])

    q = q0.copy()
    qd = np.zeros(model.n)
    q_ref = q0.copy()
    qd_ref = np.zeros(model.n)
    p_ref = p0.copy()
    buffer = [p0.copy() for _ in range(int(lags.max()) + 1)]

    energy = 0.0
    records = {"t": [], "p": [], "error": [], "tau": [], "q": [], "qd": [], "power": []}
    motors: list[list[MotorState]] = []
    saturations = 0
    prev_tau = np.zeros(model.n)
    diverged = False

    for k in range(n_ctrl):
        t = k * cfg.control_dt

        # ---- plant linearisation at the start of the control interval
        M = mass_matrix(model, q)
        M_inv = np.linalg.inv(M)
        bias = bias_torque(model, q, qd)
        M_diag = np.maximum(np.diag(M), 1e-6)
        kp = cfg.wn ** 2 * M_diag
        kd = 2.0 * cfg.zeta * cfg.wn * M_diag

        # ---- delayed measurement and resolved-rate reference update
        lag_k = int(lags[k])
        p_meas = buffer[-1 - lag_k].copy() if lag_k > 0 else buffer[-1].copy()
        if noise_vec is not None:
            p_meas = p_meas + noise_vec * rng.standard_normal(3)
        elif cfg.noise > 0:
            p_meas = p_meas + cfg.noise * rng.standard_normal(3)
        error = target - p_meas
        set_delay = getattr(controller, "set_delay", None)
        if callable(set_delay):
            # a delay-aware law (the governor) is given the delay of this frame
            set_delay(float(delays[k]))
        v_cmd = controller(error)

        J = jacobian(model, q_ref, model.n - 1)[:3]
        A = J @ J.T + (0.02 ** 2) * np.eye(3)
        qd_new = J.T @ np.linalg.solve(A, v_cmd)
        qd_new = np.clip(qd_new, -cfg.max_qd, cfg.max_qd)
        qdd_new = np.clip((qd_new - qd_ref) / cfg.control_dt,
                          -cfg.max_qdd, cfg.max_qdd)
        q_new = q_ref + qd_new * cfg.control_dt
        q_new = np.clip(q_new, limits[:, 0], limits[:, 1])
        q_ref, qd_ref = q_new, qd_new.copy()

        tau_ff = rnea(model, q, qd, qdd_new)

        p_now = end_effector_position(model, q)
        records["t"].append(t)
        records["p"].append(p_now)
        records["error"].append(float(np.linalg.norm(target - p_now)))
        records["q"].append(q.copy())
        records["qd"].append(qd.copy())

        # ---- inner joint servo: the outer command (q_new, qd_new, qdd_new) is
        # held while the PD torque is re-evaluated at the servo rate, so the
        # vision/outer rate never destabilises the joint loop.
        p_sum = 0.0
        for _ in range(n_sub):
            tau_cmd = tau_ff + kp * (q_new - q) + kd * (qd_new - qd)
            states = [drive(l.motor, float(tau_cmd[i]), float(qd[i]), servo_dt,
                            cfg.regen, float(prev_tau[i]))
                      for i, l in enumerate(model.links)]
            tau_actual = np.array([s.tau_joint for s in states])
            prev_tau = tau_actual.copy()
            saturations += sum(1 for s in states if s.saturated)
            p_elec = sum(s.p_electrical for s in states)
            p_sum += p_elec
            energy += p_elec * servo_dt
            rhs = tau_actual - bias - friction_torque(model, qd)
            qdd = M_inv @ rhs
            qd = qd + servo_dt * qdd
            q = q + servo_dt * qd
            buffer.append(end_effector_position(model, q))

        motors.append(states)
        records["tau"].append(tau_actual.copy())
        records["power"].append(p_sum / n_sub)
        if not (np.isfinite(q).all() and np.isfinite(qd).all()):
            # numerical divergence: stop and report the reach as a failure
            q = np.nan_to_num(q, nan=0.0, posinf=0.0, neginf=0.0)
            qd = np.zeros_like(qd)
            diverged = True
            break

    t_arr = np.asarray(records["t"])
    p_arr = np.asarray(records["p"])
    err = np.asarray(records["error"])
    tau_arr = np.asarray(records["tau"])
    settled = float("nan")
    outside = np.where(err > cfg.eps)[0]
    i = int(outside[-1]) + 1 if outside.size else 0
    if not diverged and i < len(t_arr):
        settled = float(t_arr[i])
    overshoot_pct, penetration_m = _overshoot_pct(p_arr, p0, target)
    metrics = {
        "overshoot_pct": overshoot_pct,
        "penetration_m": penetration_m,
        "settling_time_s": settled,
        "final_error_m": float(err[-1]),
        "max_error_m": float(err.max()),
        "peak_torque_nm": float(np.max(np.abs(tau_arr))),
        "rms_torque_nm": float(np.sqrt(np.mean(tau_arr ** 2))),
        "saturations": int(saturations),
        "energy_j": float(energy),
        "peak_power_w": float(np.max(np.abs(records["power"]))),
        "duration_s": cfg.duration,
        "payload_kg": cfg.payload,
        "delay_s": cfg.delay,
        "delay_jitter_s": float(cfg.delay_jitter),
        "delay_mean_s": float(np.mean(delays)),
    }
    return ReachResult(t=t_arr, p=p_arr, error=err, tau=tau_arr,
                       q=np.asarray(records["q"]), qd=np.asarray(records["qd"]),
                       power=np.asarray(records["power"]), energy=energy,
                       motor=motors, p0=p0, target=target, config=cfg,
                       controller=getattr(controller, "name", "controller"),
                       metrics=metrics)
