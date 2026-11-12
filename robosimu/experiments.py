"""Batch experiments and report generation."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .controllers import make_controller
from .kinematics import end_effector_position
from .model import load_model
from .simulator import SimConfig, simulate_reach

Q0 = np.array([0.0, -0.3, 0.6, 0.0, 0.6, 0.0])
REACH = np.array([0.15, 0.0, 0.0])


def _cfg(delay, payload, duration=3.0):
    return SimConfig(duration=duration, delay=delay, noise=0.0005, wn=12.0,
                     zeta=1.0, eps=0.004, payload=payload, max_qd=1.5, max_qdd=5.0)


def run_one(model, delay, payload, controller_name, seed, gif=None,
            vmax=None, max_qd=None, max_qdd=None, reach=None, eps=0.004,
            noise=0.0005, delay_jitter=0.0, delay_schedule=(),
            control_dt=0.01, duration=3.0):
    q0 = Q0.copy()
    reach = REACH if reach is None else np.asarray(reach, float)
    target = end_effector_position(model, q0) + reach
    ctrl = make_controller(controller_name, delay=delay, fps=1.0 / control_dt,
                           control_dt=control_dt, eps=eps,
                           vmax=0.5 if vmax is None else vmax, noise=noise)
    cfg = _cfg(delay, payload)
    cfg.seed = int(seed)
    cfg.eps = eps
    cfg.noise = noise
    cfg.delay_jitter = delay_jitter
    cfg.delay_schedule = delay_schedule
    cfg.control_dt = control_dt
    cfg.duration = duration
    if max_qd is not None:
        cfg.max_qd = max_qd
    if max_qdd is not None:
        cfg.max_qdd = max_qdd
    result = simulate_reach(model, q0, target, ctrl, cfg)
    if gif:
        from .render import save_gif
        save_gif(model, result, gif)
    return result


def run_suite(model_path, out_dir="reports", seeds=(0, 1, 2), make_figures=True):
    model = load_model(model_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    runs = {}

    for delay in (0.03, 0.06, 0.12, 0.18):
        for name in ("fixed", "governor"):
            for seed in seeds:
                r = run_one(model, delay, 0.0, name, seed)
                m = dict(r.metrics)
                m.update(controller=name, seed=seed)
                rows.append(m)
                runs[(delay, name, 0.0, seed)] = r

    for payload in (0.0, 2.0, 4.0):
        for name in ("fixed", "governor"):
            for seed in seeds:
                r = run_one(model, 0.12, payload, name, seed)
                m = dict(r.metrics)
                m.update(controller=name, seed=seed)
                rows.append(m)
                runs[(0.12, name, payload, seed)] = r

    with open(out / "results.csv", "w", newline="", encoding="utf-8") as fh:
        keys = ["controller", "payload_kg", "delay_s", "seed", "overshoot_pct",
                "settling_time_s", "final_error_m", "max_error_m", "peak_torque_nm",
                "rms_torque_nm", "saturations", "energy_j", "peak_power_w", "duration_s"]
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k) for k in keys})

    summary = {}
    for delay in (0.03, 0.06, 0.12, 0.18):
        summary[f"delay_{int(delay*1000)}"] = {}
        for name in ("fixed", "governor"):
            sel = [r for r in rows if r["delay_s"] == delay and r["payload_kg"] == 0
                   and r["controller"] == name]
            summary[f"delay_{int(delay*1000)}"][name] = {
                "overshoot_pct_mean": float(np.mean([s["overshoot_pct"] for s in sel])),
                "settling_s_mean": float(np.nanmean([s["settling_time_s"] for s in sel])),
                "final_error_mm_mean": float(np.mean([s["final_error_m"] for s in sel]) * 1e3),
                "energy_j_mean": float(np.mean([s["energy_j"] for s in sel])),
                "peak_torque_nm": float(np.mean([s["peak_torque_nm"] for s in sel])),
                "saturation_frac": float(np.mean([s["saturations"] / (s["duration_s"] / 0.01) for s in sel])),
            }
    for payload in (0.0, 2.0, 4.0):
        summary[f"payload_{int(payload)}"] = {}
        for name in ("fixed", "governor"):
            sel = [r for r in rows if r["payload_kg"] == payload and r["delay_s"] == 0.12
                   and r["controller"] == name]
            summary[f"payload_{int(payload)}"][name] = {
                "overshoot_pct_mean": float(np.mean([s["overshoot_pct"] for s in sel])),
                "settling_s_mean": float(np.nanmean([s["settling_time_s"] for s in sel])),
                "energy_j_mean": float(np.mean([s["energy_j"] for s in sel])),
                "peak_torque_nm": float(np.mean([s["peak_torque_nm"] for s in sel])),
            }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    if make_figures:
        _figures(model, runs, out)
    _report(summary, out / "robosimu_validation.md")
    return summary


def _figures(model, runs, out):
    from .render import save_error_plot, save_figure, save_gif
    figdir = out.parent / "figures"
    figdir.mkdir(exist_ok=True)
    key_fixed = (0.18, "fixed", 0.0, 0)
    key_gov = (0.18, "governor", 0.0, 0)
    if key_fixed in runs and key_gov in runs:
        save_figure(model, runs[key_gov], figdir / "arm_reach.png")
        save_error_plot([runs[key_fixed], runs[key_gov]], figdir / "error_compare.png")
        save_gif(model, runs[key_gov], figdir / "arm_reach.gif")


def _report(summary, path: Path):
    lines = ["# robosimu — validation report", "",
             "Six-axis rigid-body simulation, 0.15 m Cartesian reach, "
             "100 Hz control, rigid bodies, BLDC drives with harmonic gears.", ""]
    lines += ["## Delay sweep (no payload)", "",
              "| delay (ms) | controller | overshoot % | settling s | final error mm | energy J | peak torque Nm | saturation |",
              "|---|---|---|---|---|---|---|---|"]
    for d in (30, 60, 120, 180):
        for name in ("fixed", "governor"):
            s = summary[f"delay_{d}"][name]
            lines.append(f"| {d} | {name} | {s['overshoot_pct_mean']:.2f} | "
                         f"{s['settling_s_mean']:.2f} | {s['final_error_mm_mean']:.2f} | "
                         f"{s['energy_j_mean']:.1f} | {s['peak_torque_nm']:.1f} | "
                         f"{100*s['saturation_frac']:.0f} % |")
    lines += ["", "## Payload sweep (120 ms delay)", "",
              "| payload (kg) | controller | overshoot % | settling s | energy J | peak torque Nm |",
              "|---|---|---|---|---|---|"]
    for p in (0, 2, 4):
        for name in ("fixed", "governor"):
            s = summary[f"payload_{p}"][name]
            lines.append(f"| {p} | {name} | {s['overshoot_pct_mean']:.2f} | "
                         f"{s['settling_s_mean']:.2f} | {s['energy_j_mean']:.1f} | "
                         f"{s['peak_torque_nm']:.1f} |")
    lines += ["", "## Notes", "",
              "* All numbers are simulation, not hardware measurements.",
              "* Energy includes motor copper loss and gear loss; the arm is held against gravity.",
              "* The controller uses delayed Cartesian feedback (30–180 ms) and a 0.5 mm noise level.",
              "* Saturation is the fraction of control steps at which at least one joint hit its torque–speed limit.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")
