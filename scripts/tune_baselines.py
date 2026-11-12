"""Tune every baseline once at the nominal condition, then freeze it.

Protocol (used by the paper):

* nominal condition: visual delay 30 ms, outer loop 100 Hz, measurement noise
  sigma = 0.5 mm, no payload, 3 s window;
* candidate parameters are evaluated on the same four reach directions that
  the scenarios use, so tuning and evaluation share the task distribution;
* a candidate is admissible only if every reach settles inside the arrival
  tolerance with no penetration beyond it; among admissible candidates the
  lowest median movement time wins;
* the selected parameters are written to ``reports/baseline_tuning.md`` and
  hard-coded as the defaults in ``robosimu/controllers.py``.  They are never
  changed per condition.

The governor has no free parameters, so it is reported for reference only.

    python scripts/tune_baselines.py --jobs 8
"""
from __future__ import annotations

import argparse
import itertools
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robosimu.controllers import (AdaptiveCartesian, BroydenCartesian,
                                  FixedCartesian, LqrCartesian, PidCartesian,
                                  SmithCartesian, GovernorCartesian)
from robosimu.experiments import Q0, REACH
from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.scenarios import reach_variant
from robosimu.simulator import SimConfig, simulate_reach

NOMINAL_DELAY = 0.03
CONTROL_DT = 0.01
N_TASKS = 4


def candidates():
    out = {}
    out["fixed"] = [{"k": k} for k in (4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 26.0)]
    out["adaptive"] = [{"lam0": lam, "e_ref": er}
                       for lam, er in itertools.product((6.0, 10.0, 14.0),
                                                        (0.02, 0.05))]
    out["pid"] = [{"kp": kp, "ki": ki, "kd": kd}
                  for kp, ki, kd in itertools.product((6.0, 10.0, 14.0, 18.0),
                                                      (0.1, 0.5), (0.02, 0.06))]
    out["smith"] = [{"k": k} for k in (6.0, 10.0, 14.0, 20.0)]
    out["broyden"] = [{"lam": lam} for lam in (6.0, 10.0, 14.0, 20.0)]
    out["lqr"] = [{"q_pos": qp, "q_vel": qv}
                  for qp, qv in ((1.0, 0.05), (2.0, 0.1), (4.0, 0.1),
                                 (4.0, 0.24), (9.0, 0.24), (9.0, 0.5))]
    out["governor"] = [{}]
    return out


def build(name, params):
    if name == "fixed":
        return FixedCartesian(vmax=0.5, eps=0.004, **params)
    if name == "adaptive":
        return AdaptiveCartesian(vmax=0.5, eps=0.004, **params)
    if name == "pid":
        return PidCartesian(CONTROL_DT, vmax=0.5, eps=0.004, **params)
    if name == "smith":
        return SmithCartesian(NOMINAL_DELAY, control_dt=CONTROL_DT,
                              vmax=0.5, eps=0.004, **params)
    if name == "broyden":
        return BroydenCartesian(NOMINAL_DELAY, control_dt=CONTROL_DT,
                                vmax=0.5, eps=0.004, **params)
    if name == "lqr":
        return LqrCartesian(CONTROL_DT, vmax=0.5, eps=0.004, **params)
    if name == "governor":
        return GovernorCartesian(NOMINAL_DELAY, fps=1.0 / CONTROL_DT,
                                 eps=0.004, vmax=0.5, sigma=0.0005)
    raise ValueError(name)


def _run(job):
    (model_path, name, params, task) = job
    m = load_model(model_path)
    target = end_effector_position(m, Q0) + task
    ctrl = build(name, params)
    cfg = SimConfig(duration=3.0, delay=NOMINAL_DELAY, noise=0.0005, wn=12.0,
                    zeta=1.0, eps=0.004, max_qd=1.5, max_qdd=5.0, seed=0,
                    control_dt=CONTROL_DT)
    r = simulate_reach(m, Q0, target, ctrl, cfg)
    return (name, params, r.metrics["settling_time_s"],
            r.metrics["penetration_m"], r.metrics["final_error_m"])


def summarise(runs):
    mt = [r[2] for r in runs if np.isfinite(r[2])]
    pen = [r[3] for r in runs]
    final = [r[4] for r in runs]
    return {
        "n": len(runs),
        "failures": len(runs) - len(mt),
        "mt_median_s": float(np.median(mt)) if mt else float("nan"),
        "max_penetration_mm": 1e3 * float(np.max(pen)),
        "max_final_error_mm": 1e3 * float(np.max(final)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", default=str(ROOT / "models" / "six_axis_arm.json"))
    ap.add_argument("--out", default=str(ROOT / "reports" / "baseline_tuning.md"))
    ap.add_argument("--jobs", type=int, default=8)
    args = ap.parse_args()

    tasks = [reach_variant(REACH, i) for i in range(N_TASKS)]
    grid = candidates()
    jobs = [(args.model, name, params, task)
            for name, plist in grid.items()
            for params in plist
            for task in tasks]
    print(f"tuning {len(jobs)} reaches on {args.jobs} workers ...")
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(_run, jobs))

    by_combo = {}
    for name, params, mt, pen, final in results:
        by_combo.setdefault((name, tuple(sorted(params.items()))), []).append(
            (name, params, mt, pen, final))

    lines = ["# Baseline tuning protocol", "",
             f"Nominal condition: delay {NOMINAL_DELAY*1000:.0f} ms, outer loop "
             f"{1/CONTROL_DT:.0f} Hz, sigma = 0.5 mm, no payload, 3 s window.",
             f"Each candidate is evaluated on the same {N_TASKS} reach "
             "directions used by the scenarios.",
             "Admissible: no failure and no penetration beyond the arrival "
             "tolerance (4 mm). Lowest median MT wins.",
             "", "| controller | parameters | MT median s | failures | max penetration mm | admissible |",
             "|---|---|---|---|---|---|"]
    chosen = {}
    for name in grid:
        rows = []
        for (cname, pkey), runs in by_combo.items():
            if cname != name:
                continue
            s = summarise(runs)
            params = dict(pkey)
            s["params"] = params
            s["admissible"] = (s["failures"] == 0
                               and s["max_penetration_mm"] <= 4.0)
            rows.append(s)
        if not rows:
            continue
        ok = [r for r in rows if r["admissible"] and np.isfinite(r["mt_median_s"])]
        best = min(ok, key=lambda r: r["mt_median_s"]) if ok else min(
            rows, key=lambda r: (r["failures"], r["max_penetration_mm"],
                                 r["mt_median_s"] if np.isfinite(r["mt_median_s"]) else 1e9))
        chosen[name] = best
        for r in sorted(rows, key=lambda r: (not r["admissible"],
                                             r["failures"],
                                             r["mt_median_s"] if np.isfinite(r["mt_median_s"]) else 1e9)):
            mark = "yes" if r["admissible"] else "no"
            if r is best:
                mark += " (chosen)"
            pstr = ", ".join(f"{k}={v:g}" for k, v in sorted(r["params"].items())) or "none"
            lines.append(f"| {name} | {pstr} | {r['mt_median_s']:.3f} | "
                         f"{r['failures']} | {r['max_penetration_mm']:.2f} | {mark} |")
    lines += ["", "## Selected (frozen) parameters", "",
              "| controller | parameters | nominal MT median s |",
              "|---|---|---|"]
    for name, r in chosen.items():
        pstr = ", ".join(f"{k}={v:g}" for k, v in sorted(r["params"].items())) or "none"
        lines.append(f"| {name} | {pstr} | {r['mt_median_s']:.3f} |")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    for name, r in chosen.items():
        pstr = ", ".join(f"{k}={v:g}" for k, v in sorted(r["params"].items())) or "none"
        print(f"{name:9s} {pstr:50s} MT={r['mt_median_s']:.3f} "
              f"pen={r['max_penetration_mm']:.2f}mm")
    print(f"written {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
