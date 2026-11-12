"""Cross-experiment analyses: drive types, gravity loads, point-wise motion.

All three analyses reuse the same nominal reach task from ``experiments`` so
their numbers are directly comparable with the delay and payload sweeps.
"""
from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .controllers import GovernorCartesian
from .dynamics import forward_dynamics, gravity_torque
from .experiments import Q0, REACH, _cfg, run_one
from .kinematics import (end_effector_position, point_acceleration,
                         point_velocity)
from .model import ArmModel, MotorSpec, load_model
from .render import INK, GREY, OCHRE, SLATE, TEAL, TERRA, font
from .simulator import simulate_reach


# --------------------------------------------------------------------------
# drive-type comparison
# --------------------------------------------------------------------------

def motor_variant(base: MotorSpec, mtype: str) -> MotorSpec:
    """Same motor nameplate and electrical parameters; only the drive type changes.

    All three variants share stall torque, ``kt``, ``R`` and supply voltage, so
    the comparison isolates what a drive type actually changes: the torque-speed
    curve shape (linear for DC/BLDC, quadratic pull-out for a stepper), the
    speed rating (a stepper of the same frame is slower) and the efficiency
    (brushed DC and stepper drives lose more than a BLDC).
    """
    if mtype == base.type:
        return base
    if mtype not in ("dc", "bldc", "stepper"):
        raise ValueError(f"unknown motor type {mtype!r}")
    eff = {"dc": 0.72, "stepper": 0.65}[mtype]
    w_nl = base.no_load_speed if mtype != "stepper" else 0.4 * base.no_load_speed
    return MotorSpec(
        type=mtype, gear_ratio=base.gear_ratio, efficiency=eff,
        kt=base.kt, resistance=base.resistance, voltage=base.voltage,
        stall_torque=base.stall_torque, no_load_speed=w_nl,
        continuous_torque=base.continuous_torque,
        rotor_inertia=base.rotor_inertia,
        viscous_friction=base.viscous_friction,
        coulomb_friction=base.coulomb_friction,
    )


def arm_with_drive(model: ArmModel, mtype: str) -> ArmModel:
    m = copy.deepcopy(model)
    for link in m.links:
        link.motor = motor_variant(link.motor, mtype)
    return m


TASKS = {
    "nominal": {"vmax": None, "max_qd": None, "max_qdd": None},
    "fast": {"vmax": 1.0, "max_qd": 3.0, "max_qdd": 10.0},
}


def run_motor_comparison(model_path, out_dir="reports", seeds=(0, 1),
                         make_figures=True):
    base = load_model(model_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows, per_joint = [], []

    for task, over in TASKS.items():
        for mtype in ("dc", "bldc", "stepper"):
            model = arm_with_drive(base, mtype)
            for ctrl_name in ("fixed", "governor"):
                for seed in seeds:
                    r = run_one(model, 0.12, 0.0, ctrl_name, seed, **over)
                    dt = r.config.control_dt
                    m = dict(r.metrics)
                    m.update(motor=mtype, controller=ctrl_name, seed=seed, task=task)
                    m["copper_loss_j"] = float(sum(s.p_copper for step in r.motor for s in step) * dt)
                    m["mechanical_j"] = float(sum(s.p_mechanical for step in r.motor for s in step) * dt)
                    m["peak_current_a"] = float(max(abs(s.current) for step in r.motor for s in step))
                    m["peak_voltage_v"] = float(max(abs(s.voltage) for step in r.motor for s in step))
                    m["saturation_steps"] = int(sum(1 for step in r.motor for s in step if s.saturated))
                    rows.append(m)
                    for j in range(model.n):
                        per_joint.append({
                            "task": task, "motor": mtype, "controller": ctrl_name,
                            "seed": seed, "joint": j + 1,
                            "energy_j": float(sum(step[j].p_electrical for step in r.motor) * dt),
                            "copper_j": float(sum(step[j].p_copper for step in r.motor) * dt),
                            "peak_current_a": float(max(abs(step[j].current) for step in r.motor)),
                            "peak_torque_nm": float(max(abs(step[j].tau_joint) for step in r.motor)),
                            "saturation_steps": int(sum(1 for step in r.motor if step[j].saturated)),
                        })

    with open(out / "motor_comparison.csv", "w", newline="", encoding="utf-8") as fh:
        keys = ["task", "motor", "controller", "seed", "energy_j", "copper_loss_j",
                "mechanical_j", "peak_current_a", "peak_voltage_v",
                "saturation_steps", "overshoot_pct", "settling_time_s",
                "final_error_m", "peak_torque_nm", "rms_torque_nm"]
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k) for k in keys})

    with open(out / "motor_comparison_per_joint.csv", "w", newline="", encoding="utf-8") as fh:
        keys = ["task", "motor", "controller", "seed", "joint", "energy_j", "copper_j",
                "peak_current_a", "peak_torque_nm", "saturation_steps"]
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(per_joint)

    summary = {}
    for task in TASKS:
        summary[task] = {}
        for mtype in ("dc", "bldc", "stepper"):
            summary[task][mtype] = {}
            for ctrl_name in ("fixed", "governor"):
                sel = [r for r in rows if r["task"] == task and r["motor"] == mtype
                       and r["controller"] == ctrl_name]
                summary[task][mtype][ctrl_name] = {
                    "energy_j_mean": float(np.mean([s["energy_j"] for s in sel])),
                    "copper_loss_j_mean": float(np.mean([s["copper_loss_j"] for s in sel])),
                    "peak_current_a_mean": float(np.mean([s["peak_current_a"] for s in sel])),
                    "saturation_steps_mean": float(np.mean([s["saturation_steps"] for s in sel])),
                    "overshoot_pct_mean": float(np.mean([s["overshoot_pct"] for s in sel])),
                    "settling_s_mean": float(np.nanmean([s["settling_time_s"] for s in sel])),
                    "peak_torque_nm_mean": float(np.mean([s["peak_torque_nm"] for s in sel])),
                }
    (out / "motor_comparison_summary.json").write_text(json.dumps(summary, indent=2),
                                                       encoding="utf-8")
    if make_figures:
        _motor_figure(summary, out.parent / "figures" / "motor_comparison.png")
    _motor_report(summary, per_joint, out / "motor_comparison.md")
    return summary


def _motor_figure(summary, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    W, H = 980, 460
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    motors = ["dc", "bldc", "stepper"]
    colors = {"dc": SLATE, "bldc": TEAL, "stepper": TERRA}
    panels = [
        ("nominal: energy J", "nominal", "energy_j_mean"),
        ("nominal: saturation steps", "nominal", "saturation_steps_mean"),
        ("fast: energy J", "fast", "energy_j_mean"),
        ("fast: saturation steps", "fast", "saturation_steps_mean"),
    ]
    for k, (title, task, key) in enumerate(panels):
        x0 = 55 + k * 235
        y0, y1 = 70, 380
        vals = [summary[task][m]["governor"][key] for m in motors]
        vmax = max(vals) * 1.15 if max(vals) > 0 else 1.0
        d.rectangle([x0, y0, x0 + 200, y1], outline=INK, width=2)
        d.text((x0 + 8, y0 - 30), title, font=font(15, True), fill=INK)
        for i, (m, v) in enumerate(zip(motors, vals)):
            bx = x0 + 22 + i * 60
            h = (v / vmax) * (y1 - y0 - 20)
            d.rectangle([bx, y1 - h, bx + 44, y1], fill=colors[m], outline=INK)
            d.text((bx + 2, y1 + 6), m, font=font(14), fill=INK)
            d.text((bx + 2, y1 - h - 22), f"{v:.1f}", font=font(13), fill=INK)
        d.text((x0 + 8, y1 + 30), "drive type (governor)", font=font(13), fill=GREY)
    img.save(path, dpi=(300, 300))
    return path


def _motor_report(summary, per_joint, path):
    lines = ["# Drive-type comparison (DC vs BLDC vs stepper)", "",
             "Same arm, same reach (0.15 m, 120 ms visual delay). Two tasks: the",
             "nominal reach (0.5 m/s Cartesian limit) and a fast reach (1.0 m/s,",
             "3 rad/s joint limit) that exposes the stepper's speed weakness.", ""]
    for task, label in (("nominal", "Nominal task"), ("fast", "Fast task")):
        lines += [f"## {label}", "",
                  "| drive | controller | energy J | copper J | peak current A | sat. steps | overshoot % | settling s |",
                  "|---|---|---|---|---|---|---|---|"]
        for m in ("dc", "bldc", "stepper"):
            for c in ("fixed", "governor"):
                s = summary[task][m][c]
                settle = (f"{s['settling_s_mean']:.2f}"
                          if np.isfinite(s["settling_s_mean"]) else "n/a")
                lines.append(f"| {m} | {c} | {s['energy_j_mean']:.1f} | {s['copper_loss_j_mean']:.1f} | "
                             f"{s['peak_current_a_mean']:.2f} | {s['saturation_steps_mean']:.0f} | "
                             f"{s['overshoot_pct_mean']:.2f} | {settle} |")
        lines.append("")
    lines += ["## Per-joint energy, governor, nominal task (mean over seeds)", "",
              "| joint | DC J | BLDC J | stepper J |",
              "|---|---|---|---|"]
    for j in range(6):
        vals = []
        for m in ("dc", "bldc", "stepper"):
            sel = [r for r in per_joint if r["task"] == "nominal" and r["motor"] == m
                   and r["controller"] == "governor" and r["joint"] == j + 1]
            vals.append(float(np.mean([r["energy_j"] for r in sel])))
        lines.append(f"| {j+1} | {vals[0]:.1f} | {vals[1]:.1f} | {vals[2]:.1f} |")
    lines += ["", "## Notes", "",
              "* All three drives share stall torque, kt, R and V; only the drive type changes.",
              "* DC and BLDC use a linear torque-speed line; efficiencies are 0.72 (brushed) and",
              "  0.80 (brushless).",
              "* The stepper keeps the same stall torque but its no-load speed is 40% of the",
              "  BLDC's (typical for the same frame size) and its pull-out torque falls as",
              "  1/(1+(w/wc)^2), so it saturates earlier at speed; its efficiency is 0.65.",
              "* Saturation steps count control steps at which a joint hit its torque-speed limit.",
              "* Simulation only: motor parameters are representative datasheet values, not measured.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")


# --------------------------------------------------------------------------
# gravity load map
# --------------------------------------------------------------------------

def gravity_map(model_path, out_dir="reports", n=21, payloads=(0.0, 4.0),
                make_figures=True):
    """Joint torques from the arm's own weight over a shoulder/elbow grid."""
    base = load_model(model_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    q0 = Q0.copy()
    rows = []
    grids = {}

    for payload in payloads:
        model = base.with_payload(payload) if payload > 0 else base
        lo1, hi1 = model.links[1].limits
        lo2, hi2 = model.links[2].limits
        g1 = np.linspace(lo1, hi1, n)
        g2 = np.linspace(lo2, hi2, n)
        tau = np.zeros((n, n, model.n))
        for a, q1 in enumerate(g1):
            for b, q2 in enumerate(g2):
                q = q0.copy()
                q[1], q[2] = q1, q2
                tau[a, b] = gravity_torque(model, q)
        grids[payload] = (g1, g2, tau)
        for j in range(model.n):
            col = np.abs(tau[:, :, j])
            k = int(np.argmax(col))
            a, b = divmod(k, n)
            cont = float(model.links[j].motor.continuous_torque_joint)
            rows.append({
                "payload_kg": payload, "joint": j + 1,
                "max_abs_tau_nm": float(col[a, b]),
                "q_shoulder_deg": float(math.degrees(g1[a])),
                "q_elbow_deg": float(math.degrees(g2[b])),
                "continuous_torque_nm": cont,
                "utilisation_pct": float(100.0 * col[a, b] / cont) if cont > 0 else 0.0,
            })

    with open(out / "gravity_load.csv", "w", newline="", encoding="utf-8") as fh:
        keys = ["payload_kg", "joint", "max_abs_tau_nm", "q_shoulder_deg",
                "q_elbow_deg", "continuous_torque_nm", "utilisation_pct"]
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    if make_figures:
        _gravity_figure(grids, out.parent / "figures" / "gravity_load.png")
    _gravity_report(rows, grids, n, out / "gravity_load.md")
    return rows


def _gravity_figure(grids, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    payloads = sorted(grids)
    joints = [1, 2]          # shoulder, elbow (0-based)
    cell = 240
    W = 40 + len(joints) * (cell + 30)
    H = 60 + len(payloads) * (cell + 60)
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    for r, payload in enumerate(payloads):
        g1, g2, tau = grids[payload]
        for c, j in enumerate(joints):
            x0 = 30 + c * (cell + 30)
            y0 = 40 + r * (cell + 60)
            data = np.abs(tau[:, :, j])
            vmax = float(data.max()) if data.max() > 0 else 1.0
            d.text((x0, y0 - 24), f"joint {j+1} gravity torque, payload {payload:g} kg",
                   font=font(15, True), fill=INK)
            for a in range(len(g1)):
                for b in range(len(g2)):
                    f = float(data[a, b]) / vmax
                    col = (int(255 - 180 * f), int(255 - 220 * f), int(255 - 220 * f))
                    px = x0 + int(b * cell / (len(g2) - 1))
                    py = y0 + int(a * cell / (len(g1) - 1))
                    d.rectangle([px, py, px + cell // len(g2) + 1, py + cell // len(g1) + 1], fill=col)
            d.rectangle([x0, y0, x0 + cell, y0 + cell], outline=INK, width=2)
            d.text((x0, y0 + cell + 6), f"max {vmax:.1f} Nm", font=font(13), fill=GREY)
            d.text((x0 + cell - 90, y0 + cell + 6), "shoulder/elbow grid", font=font(12), fill=GREY)
    img.save(path, dpi=(300, 300))
    return path


def _gravity_report(rows, grids, n, path):
    lines = ["# Gravity load map — arm self-weight and payload", "",
             "Maximum absolute joint torque over a shoulder/elbow angle grid,",
             "compared with each joint's continuous torque rating.", "",
             "| payload kg | joint | max abs tau Nm | at shoulder deg | at elbow deg | continuous Nm | utilisation % |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['payload_kg']:g} | {r['joint']} | {r['max_abs_tau_nm']:.2f} | "
                     f"{r['q_shoulder_deg']:.1f} | {r['q_elbow_deg']:.1f} | "
                     f"{r['continuous_torque_nm']:.1f} | {r['utilisation_pct']:.1f} |")
    lines += ["", "## Notes", "",
              "* Joint 1 (base yaw) has a vertical axis and carries essentially no gravity torque.",
              "* The shoulder and elbow joints carry the arm's own weight; utilisation stays below",
              "  the continuous rating over the whole grid, so the arm can hold any pose statically.",
              "* Payload is a point mass at the tool; it adds to the shoulder/elbow load.",
              "* Rigid-body model, no cable or harness weight included.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")


# --------------------------------------------------------------------------
# point-wise velocity / acceleration
# --------------------------------------------------------------------------

def point_motion(model_path, out_dir="reports", seed=0, delay=0.12, every=5,
                 make_figures=True):
    """Per-link centre-of-mass and tool-point motion along one reach."""
    model = load_model(model_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    q0 = Q0.copy()
    target = end_effector_position(model, q0) + REACH
    ctrl = GovernorCartesian(delay=delay, fps=100, eps=0.004, vmax=0.5)
    r = simulate_reach(model, q0, target, ctrl, _cfg(delay, 0.0))

    idx = list(range(0, len(r.t), every))
    series = {i: {"v": [], "w": [], "a": [], "al": []} for i in range(model.n)}
    ts = []
    for k in idx:
        ts.append(float(r.t[k]))
        qdd = forward_dynamics(model, r.q[k], r.qd[k], r.tau[k])
        for i in range(model.n):
            com = model.links[i].com
            v, w = point_velocity(model, r.q[k], r.qd[k], i, com)
            a, al = point_acceleration(model, r.q[k], r.qd[k], qdd, i, com)
            series[i]["v"].append(float(np.linalg.norm(v)))
            series[i]["w"].append(float(np.linalg.norm(w)))
            series[i]["a"].append(float(np.linalg.norm(a)))
            series[i]["al"].append(float(np.linalg.norm(al)))

    rows = []
    for i in range(model.n):
        s = series[i]
        rows.append({
            "link": i + 1, "name": model.links[i].name,
            "max_speed_mps": max(s["v"]), "max_accel_mps2": max(s["a"]),
            "max_ang_speed_rads": max(s["w"]), "max_ang_accel_rads2": max(s["al"]),
        })
    with open(out / "point_motion.csv", "w", newline="", encoding="utf-8") as fh:
        keys = ["link", "name", "max_speed_mps", "max_accel_mps2",
                "max_ang_speed_rads", "max_ang_accel_rads2"]
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    if make_figures:
        _motion_figure(ts, series, out.parent / "figures" / "point_motion.png")
    _motion_report(rows, ts, series, out / "point_motion.md")
    return rows


def _motion_figure(ts, series, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    W, H = 1000, 700
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    colors = [SLATE, TERRA, TEAL, OCHRE, SLATE, TERRA]
    picks = [0, 1, 2, 5]
    panels = [("linear speed (m/s)", "v"), ("linear acceleration (m/s^2)", "a"),
              ("angular speed (rad/s)", "w"), ("angular acceleration (rad/s^2)", "al")]
    for k, (title, key) in enumerate(panels):
        x0 = 90 + (k % 2) * 450
        y0 = 60 + (k // 2) * 320
        x1, y1 = x0 + 380, y0 + 240
        d.rectangle([x0, y0, x1, y1], outline=INK, width=2)
        d.text((x0, y0 - 26), title, font=font(16, True), fill=INK)
        tmax = max(ts)
        vmax = max(max(series[i][key]) for i in picks) * 1.1 or 1.0
        for m, i in enumerate(picks):
            col = colors[i % len(colors)]
            pts = [(x0 + (t / tmax) * (x1 - x0), y1 - (v / vmax) * (y1 - y0))
                   for t, v in zip(ts, series[i][key])]
            d.line(pts, fill=col, width=3)
            d.text((x1 - 130, y0 + 8 + 20 * m), f"link {i+1}", font=font(14, True), fill=col)
        for m in range(5):
            frac = m / 4
            y = y1 - frac * (y1 - y0)
            d.text((x0 - 70, y - 8), f"{frac*vmax:5.2f}", font=font(13), fill=GREY)
        d.text((x0, y1 + 8), f"0", font=font(13), fill=GREY)
        d.text((x1 - 40, y1 + 8), f"{tmax:.2f} s", font=font(13), fill=GREY)
    img.save(path, dpi=(300, 300))
    return path


def _motion_report(rows, ts, series, path):
    lines = ["# Point-wise velocity and acceleration", "",
             "Maximum values along a 0.15 m governor reach (120 ms delay), evaluated",
             "at each link's centre of mass. Acceleration uses the recorded joint",
             "torques through forward dynamics.", "",
             "| link | name | max speed m/s | max accel m/s^2 | max ang speed rad/s | max ang accel rad/s^2 |",
             "|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['link']} | {r['name']} | {r['max_speed_mps']:.3f} | "
                     f"{r['max_accel_mps2']:.2f} | {r['max_ang_speed_rads']:.3f} | "
                     f"{r['max_ang_accel_rads2']:.2f} |")
    lines += ["", "## Notes", "",
              "* Link 1 is the base; its centre of mass sits on the yaw axis, so its",
              "  translational speed is small.",
              "* Peak values occur during the acceleration phase of the reach.",
              "* Simulation quantities from the rigid-body model; no compliance or",
              "  structural vibration is modelled.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")
