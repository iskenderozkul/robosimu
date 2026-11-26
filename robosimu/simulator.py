"""Closed-loop reaching simulation with delayed visual feedback."""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

from .dynamics import (bias_torque, friction_torque, gravity_torque, mass_matrix, rnea)
from .kinematics import end_effector_position, jacobian
from .model import ArmModel
from .motors import MotorState, drive
from .stereo import (NeckKinematics, StereoCameraConfig, StereoCameraRig,
                     StereoObservation, compute_singularity_metrics,
                     singularity_safe_joint_velocities)
from .v43 import TimestampedObservation


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
    stereo: StereoCameraConfig | None = None   # if set, use two-eye stereo visual feedback
    singularity_avoidance: bool = False        # if True, use singularity-aware joint routing
    intermittent_gaze: bool = False            # if True, gate visual sampling with saccadic hold+blind flight
    zero_oracle: bool = False                  # V4.3 firewall: pixel-only timestamped observations
    neck: bool = True                          # attach the 2-DOF neck when zero_oracle
    encoderless: bool = False                  # v6: no joint servo; motors get only a drive command
    drive_damping: float = 40.0                # back-EMF / gearbox damping (x mass-matrix diagonal at q0), 1/s
    brake: bool = True                         # per-joint holding brake engages when its command is zero


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
    manipulability: np.ndarray = field(default_factory=lambda: np.zeros(0))
    condition_number: np.ndarray = field(default_factory=lambda: np.zeros(0))
    damping_lambda: np.ndarray = field(default_factory=lambda: np.zeros(0))
    stereo_obs: list[StereoObservation] = field(default_factory=list)


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
    w_mech = 0.0
    sampled_visual_frames = 0
    fixation_count = 0
    gaze_cycle_s = 0.62
    fixation_window_s = 0.14
    prev_is_fixating = False
    held_v_cmd = np.zeros(3)
    records = {"t": [], "p": [], "error": [], "tau": [], "q": [], "qd": [], "power": [],
               "manip": [], "cond": [], "lam": []}
    motors: list[list[MotorState]] = []
    stereo_obs_list: list[StereoObservation] = []
    stereo_rig = StereoCameraRig(cfg.stereo) if cfg.stereo is not None else None
    # ---- V4.3 zero-oracle path: pixel-only observations + 2-DOF neck
    joint_direct = bool(getattr(controller, "zero_oracle", False)) or bool(cfg.zero_oracle)
    neck_kin = None
    if joint_direct:
        if stereo_rig is None:
            raise ValueError("zero-oracle mode needs cfg.stereo (pixel observations)")
        if bool(cfg.neck):
            neck_kin = NeckKinematics(cam_pos=cfg.stereo.cam_pos,
                                      base_origin=(0.0, 0.0, 0.0))
            stereo_rig.set_neck(neck_kin)
            if getattr(controller, "neck", None) is None:
                controller.neck = neck_kin
        if getattr(controller, "outputs_joint_velocity", False) is not True:
            raise TypeError("zero-oracle controller must emit joint velocities")
    qd_direct = np.zeros(model.n)
    dq_neck_now = np.zeros(2)
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
        p_delayed = buffer[-1 - lag_k].copy() if lag_k > 0 else buffer[-1].copy()
        if not joint_direct:
            set_delay = getattr(controller, "set_delay", None)
            if callable(set_delay):
                # a delay-aware law (the governor) is given the delay of this frame
                set_delay(float(delays[k]))

        visual_active = True
        if cfg.intermittent_gaze:
            # Saccadic cycle: S1 hand fixation + S2 target fixation, then S3 blind flight
            phase_t = t % gaze_cycle_s
            is_fixating = phase_t < (2.0 * fixation_window_s)
            if is_fixating and not prev_is_fixating:
                fixation_count += 1
            prev_is_fixating = is_fixating
            visual_active = is_fixating

        if visual_active or k == 0:
            sampled_visual_frames += 1
            if not cfg.intermittent_gaze and (k == 0 or (k * cfg.control_dt) % gaze_cycle_s < cfg.control_dt):
                fixation_count += 1
            if stereo_rig is not None:
                if neck_kin is not None:
                    # the rig is rigidly mounted on the trunk: it inherits q[0]
                    neck_kin.set_trunk(float(q[0]))
                obs_st = stereo_rig.observe(
                    p_delayed, target, rng=rng, t_sec=t,
                    t_cap=max(0.0, t - float(delays[k])), t_arr=t, t_now=t,
                    neck_pan=neck_kin.pan if neck_kin else 0.0,
                    neck_tilt=neck_kin.tilt if neck_kin else 0.0)
                stereo_obs_list.append(obs_st)
                if joint_direct:
                    # ZERO-ORACLE: pixels + timestamps only
                    pkt = TimestampedObservation(
                        t_cap=max(0.0, t - float(delays[k])), t_arr=t, t_now=t,
                        hand=np.array([obs_st.uv_left_gripper, obs_st.uv_right_gripper], float),
                        target=np.array([obs_st.uv_left_target, obs_st.uv_right_target], float))
                    controller.step_obs(pkt)
                    qd_direct = np.asarray(controller.last_qd, float)
                    dq_neck_now = np.asarray(controller.last_dq_neck, float)
                    v_cmd = np.zeros(3)
                else:
                    set_sigma = getattr(controller, "set_sigma", None)
                    if callable(set_sigma):
                        set_sigma(obs_st.sigma_cam_m)
                    if getattr(controller, "per_axis", False):
                        v_cam = controller(obs_st.rel_cam_gap_m)
                        v_cmd = stereo_rig.cam_to_world_vec(v_cam)
                    else:
                        v_cmd = controller(obs_st.rel_base_gap_m)
            else:
                p_meas = p_delayed
                if noise_vec is not None:
                    p_meas = p_meas + noise_vec * rng.standard_normal(3)
                elif cfg.noise > 0:
                    p_meas = p_meas + cfg.noise * rng.standard_normal(3)
                error = target - p_meas
                v_cmd = controller(error)
            held_v_cmd = np.asarray(v_cmd, float).copy()
        else:
            # Sample-and-hold zero-order hold during blind ballistic flight
            v_cmd = held_v_cmd.copy()
            if joint_direct and callable(getattr(controller, "step_blind", None)):
                # efference-copy forward model: the controller predicts its own hand motion
                qd_direct = np.asarray(controller.step_blind(t), float)

        if joint_direct:
            # the controller already emitted joint rates via its learned Jacobian
            qd_new = np.zeros(model.n)
            qd_new[:5] = np.clip(qd_direct, -cfg.max_qd, cfg.max_qd)
            _, sing_status = compute_singularity_metrics(model, q_ref, qd_ref, prev_tau)
        elif cfg.singularity_avoidance:
            qd_new, sing_status = singularity_safe_joint_velocities(
                model, q_ref, qd_ref, v_cmd, tau_prev=prev_tau, max_qd=cfg.max_qd
            )
            # Triphasic joint sequencing (human-like proximal -> distal
            # activation with terminal co-contraction), if the controller
            # implements it.
            seq_fn = getattr(controller, "sequencing_weights", None)
            if callable(seq_fn):
                wj = np.asarray(seq_fn(), float)
                qd_new = np.clip(qd_new, -cfg.max_qd * wj, cfg.max_qd * wj)
        else:
            _, sing_status = compute_singularity_metrics(model, q_ref, qd_ref, prev_tau)
            J = jacobian(model, q_ref, model.n - 1)[:3]
            A = J @ J.T + (0.02 ** 2) * np.eye(3)
            qd_new = J.T @ np.linalg.solve(A, v_cmd)
            qd_new = np.clip(qd_new, -cfg.max_qd, cfg.max_qd)
            # Triphasic joint sequencing caps (if the controller implements it)
            seq_fn = getattr(controller, "sequencing_weights", None)
            if callable(seq_fn):
                wj = np.asarray(seq_fn(), float)
                qd_new = np.clip(qd_new, -cfg.max_qd * wj, cfg.max_qd * wj)

        records["manip"].append(sing_status.manipulability)
        records["cond"].append(sing_status.condition_number)
        records["lam"].append(sing_status.damping_lambda)
        qdd_new = np.clip((qd_new - qd_ref) / cfg.control_dt,
                          -cfg.max_qdd, cfg.max_qdd)
        q_new = q_ref + qd_new * cfg.control_dt
        q_new = np.clip(q_new, limits[:, 0], limits[:, 1])
        q_ref, qd_ref = q_new, qd_new.copy()

        if joint_direct:
            # efference copy of what the plant actually issued, plus neck move
            dq_neck_applied = np.clip(dq_neck_now, -2.0, 2.0)
            controller.commit(t + cfg.control_dt, qd_new[:5], dq_neck_applied,
                              cfg.control_dt)
            if neck_kin is not None:
                neck_kin.set_neck(neck_kin.pan + dq_neck_applied[0] * cfg.control_dt,
                                  neck_kin.tilt + dq_neck_applied[1] * cfg.control_dt)

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
        if cfg.encoderless and k == 0:
            B_drive = cfg.drive_damping * np.maximum(np.diag(mass_matrix(model, q0)), 1e-6)
        braked = (np.abs(qd_new) < 1e-9) if (cfg.encoderless and cfg.brake) else np.zeros(model.n, bool)
        for _ in range(n_sub):
            if cfg.encoderless:
                # No encoder, no gravity compensation, no position servo: the drive only
                # receives a speed command; back-EMF damping, gravity, friction and inertia
                # decide how far the joint actually moves.  A braked joint is clamped.
                tau_cmd = np.where(braked, 0.0, B_drive * (qd_new - qd))
            else:
                tau_cmd = tau_ff + kp * (q_new - q) + kd * (qd_new - qd)
            states = [drive(l.motor, float(tau_cmd[i]), float(qd[i]), servo_dt,
                            cfg.regen, float(prev_tau[i]))
                      for i, l in enumerate(model.links)]
            tau_actual = np.array([s.tau_joint for s in states])
            prev_tau = tau_actual.copy()
            saturations += sum(1 for s in states if s.saturated)
            p_elec = sum(s.p_electrical for s in states)
            w_mech += float(np.sum(np.abs(tau_actual * qd))) * servo_dt
            p_sum += p_elec
            energy += p_elec * servo_dt
            rhs = tau_actual - bias - friction_torque(model, qd)
            qdd = M_inv @ rhs
            qd = qd + servo_dt * qdd
            if cfg.encoderless:
                qd = np.where(braked, 0.0, qd)
            q = q + servo_dt * qd
            over_ = q > limits[:, 1]
            under_ = q < limits[:, 0]
            q = np.clip(q, limits[:, 0], limits[:, 1])
            if bool((over_ | under_).any()):
                qd = np.where(over_ | under_, 0.0, qd)
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
    manip_arr = np.asarray(records["manip"])
    cond_arr = np.asarray(records["cond"])
    lam_arr = np.asarray(records["lam"])
    if len(p_arr) >= 2:
        vel_mag = np.linalg.norm(np.gradient(p_arr, cfg.control_dt, axis=0), axis=1)
        peak_idx = int(np.argmax(vel_mag))
    else:
        peak_idx = 0
    t_ballistic = float(t_arr[peak_idx]) if len(t_arr) else 0.0
    e_init_m = float(err[peak_idx]) if len(err) else 0.0
    t_total = float(settled) if (np.isfinite(settled) and settled > 0.0) else (float(t_arr[-1]) if len(t_arr) else cfg.duration)
    t_comp_s = max(0.0, t_total - t_ballistic)
    t_comp_pct = 100.0 * t_comp_s / max(t_total, 1e-9)
    sim_dur = float(t_arr[-1] + cfg.control_dt) if len(t_arr) else cfg.duration
    visual_load_hz = float(sampled_visual_frames) / max(sim_dur, 1e-9)
    pulse_efficiency = float(min(1.0, max(0.0, w_mech / max(abs(energy), 1e-9))))
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
        "mechanical_work_j": float(w_mech),
        "peak_power_w": float(np.max(np.abs(records["power"]))),
        "duration_s": cfg.duration,
        "payload_kg": cfg.payload,
        "delay_s": cfg.delay,
        "delay_jitter_s": float(cfg.delay_jitter),
        "delay_mean_s": float(np.mean(delays)),
        "min_manipulability": float(np.min(manip_arr)) if manip_arr.size else 0.0,
        "max_condition_number": float(np.max(cond_arr)) if cond_arr.size else 0.0,
        "e_init_m": e_init_m,
        "t_ballistic_s": t_ballistic,
        "t_comp_pct": t_comp_pct,
        "visual_load_hz": visual_load_hz,
        "pulse_efficiency": pulse_efficiency,
        "sampled_visual_frames": int(sampled_visual_frames),
        "fixation_count": int(fixation_count),
    }
    if joint_direct:
        info = getattr(controller, "info", {}) or {}
        metrics.update({
            "zero_oracle": True,
            "delta_obs_s": float(info.get("delta_obs", 0.0)),
            "delta_eff_s": float(info.get("delta_eff", 0.0)),
            "n_star": int(info.get("n_star", 1)),
            "window_admissible": bool(info.get("admissible", True)),
            "stale": bool(info.get("stale", False)),
            "broyden_fro": float(info.get("broyden_fro", 0.0)),
            "neck_pan_rad": float(neck_kin.pan) if neck_kin is not None else 0.0,
            "neck_tilt_rad": float(neck_kin.tilt) if neck_kin is not None else 0.0,
        })
    return ReachResult(t=t_arr, p=p_arr, error=err, tau=tau_arr,
                       q=np.asarray(records["q"]), qd=np.asarray(records["qd"]),
                       power=np.asarray(records["power"]), energy=energy,
                       motor=motors, p0=p0, target=target, config=cfg,
                       controller=getattr(controller, "name", "controller"),
                       metrics=metrics, manipulability=manip_arr,
                       condition_number=cond_arr, damping_lambda=lam_arr,
                       stereo_obs=stereo_obs_list)


@dataclass
class ReachMetrics:
    """Four orthogonal manuscript metrics (§3.6) plus core trajectory indicators."""
    e_init_m: float
    t_comp_pct: float
    visual_load_hz: float
    pulse_efficiency: float
    settling_time_s: float
    overshoot_pct: float
    energy_j: float


def compute_metrics(result: ReachResult) -> ReachMetrics:
    """Extract the four orthogonal manuscript metrics from a ReachResult."""
    m = result.metrics
    return ReachMetrics(
        e_init_m=float(m.get("e_init_m", 0.0)),
        t_comp_pct=float(m.get("t_comp_pct", 0.0)),
        visual_load_hz=float(m.get("visual_load_hz", 0.0)),
        pulse_efficiency=float(m.get("pulse_efficiency", 0.0)),
        settling_time_s=float(m.get("settling_time_s", float("nan"))),
        overshoot_pct=float(m.get("overshoot_pct", 0.0)),
        energy_j=float(m.get("energy_j", 0.0)),
    )
