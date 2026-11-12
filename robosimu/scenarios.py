"""Pre-defined academic scenarios and the batch runner.

Each scenario is a matrix of reaches over one manipulated variable (visual
delay, control rate, noise, delay jitter, a mid-move delay jump, or target
distance/width for a Fitts task).  Controllers are compared inside the same
matrix, and every cell reports the median movement time with a bootstrap
interval, the overshoot rate with a Wilson interval, and the energy.

The runner is parallelised with :class:`concurrent.futures.ProcessPoolExecutor`
and falls back to serial execution when ``jobs=1``.
"""
from __future__ import annotations

import csv
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .experiments import REACH, run_one
from .model import load_model
from .render import INK, GREY, OCHRE, SLATE, TEAL, TERRA, font
from .stats import (bootstrap_ci, cohens_d, effective_width, fitts_id,
                    holm_bonferroni, median, permutation_test, throughput,
                    wilson_ci)

CONTROLLERS = ("fixed", "adaptive", "smith", "broyden", "pid", "lqr", "governor")
COLORS = {"fixed": SLATE, "governor": TEAL, "adaptive": TERRA, "smith": OCHRE,
          "broyden": (120, 90, 160), "pid": (90, 140, 90),
          "lqr": (170, 70, 120)}


# A fixed, low-discrepancy family of reach directions inside a 18-degree cone
# around the nominal reach.  Every cell sees the same task set, so controllers
# are compared on paired reaches; the random seed only changes measurement
# noise.  Distances are scaled by 0.92-1.08 except for Fitts cells, where the
# amplitude must stay exact.
def _task_table(n: int = 64):
    u = [(0.5 + 0.5 * math.cos(2.0 * math.pi * i * 0.61803398875))
         * (0.5 + 0.5 * math.sin(2.0 * math.pi * i * 0.75487766625))
         for i in range(n)]
    return u


_TASKS = _task_table()


def reach_variant(base, index: int, vary_distance: bool = True):
    base = np.asarray(base, float)
    d = float(np.linalg.norm(base))
    if d < 1e-9:
        return base.copy()
    u = base / d
    a = np.array([0.0, 1.0, 0.0])
    if abs(float(u @ a)) > 0.9:
        a = np.array([1.0, 0.0, 0.0])
    e1 = np.cross(u, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    idx = int(index) % len(_TASKS)
    tilt = 0.32 * math.sqrt((idx + 0.5) / len(_TASKS))
    phi = 2.0 * math.pi * ((idx * 0.61803398875) % 1.0)
    direction = (math.cos(tilt) * u
                 + math.sin(tilt) * (math.cos(phi) * e1 + math.sin(phi) * e2))
    scale = 1.0 if not vary_distance else 0.92 + 0.16 * _TASKS[(idx + 17) % len(_TASKS)]
    return d * scale * direction


@dataclass
class Scenario:
    name: str
    xlabel: str
    jobs: list = field(default_factory=list)
    rows: list = field(default_factory=list)
    summary: dict = field(default_factory=dict)
    cell_keys: dict = field(default_factory=dict)
    cell_rows: dict = field(default_factory=dict)
    comparisons: list = field(default_factory=list)
    schedules: list = field(default_factory=list)
    fitts_cells: list = field(default_factory=list)


def _job(args):
    """Worker: run one reach and return its metrics (top-level for pickling)."""
    (model_path, delay, payload, ctrl, seed, reach_id, reach, eps, vmax, max_qd,
     max_qdd, noise, delay_jitter, delay_schedule, control_dt, duration, label) = args
    model = load_model(model_path)
    r = run_one(model, delay, payload, ctrl, seed, reach=reach, eps=eps, vmax=vmax,
                max_qd=max_qd, max_qdd=max_qdd, noise=noise,
                delay_jitter=delay_jitter, delay_schedule=delay_schedule,
                control_dt=control_dt, duration=duration)
    m = dict(r.metrics)
    m.update(delay_s=delay, payload_kg=payload, controller=ctrl, seed=seed,
             reach_id=reach_id, delay_jitter_s=delay_jitter,
             control_dt_s=control_dt, distance_m=float(np.linalg.norm(reach)),
             eps_m=float(eps), label=label)
    if isinstance(noise, (list, tuple)):
        m["noise"] = "ani"
        m["noise_sigma"] = float(max(noise))
        m["noise_label"] = "sigma xyz = " + ", ".join(f"{1000*n:.2g}" for n in noise) + " mm"
    else:
        m["noise"] = float(noise)
        m["noise_sigma"] = float(noise)
        m["noise_label"] = f"sigma iso = {1000*noise:.2g} mm"
    return m


def _build_jobs(model_path, delays, controllers, seeds, reaches, reach=REACH,
                eps=0.004, vmax=0.5, max_qd=1.5, max_qdd=5.0, noise=0.0005,
                delay_jitter=0.0, delay_schedule=(), control_dt=0.01,
                duration=3.0, payload=0.0, label="", vary_distance=True):
    jobs = []
    base = np.asarray(reach, float)
    for delay in delays:
        for ctrl in controllers:
            for seed in seeds:
                for r in range(reaches):
                    task = reach_variant(base, seed * reaches + r,
                                         vary_distance=vary_distance)
                    jobs.append((model_path, delay, payload, ctrl, seed, r,
                                 task, eps, vmax, max_qd,
                                 max_qdd, noise, delay_jitter, delay_schedule,
                                 control_dt, duration, label))
    return jobs


# --------------------------------------------------------------------------
# scenario definitions
# --------------------------------------------------------------------------

def scenario_delay(model_path, seeds, reaches):
    return Scenario(
        name="delay",
        xlabel="visual delay (ms)",
        jobs=_build_jobs(model_path, [0.0, 0.03, 0.06, 0.12, 0.18, 0.20],
                         CONTROLLERS, seeds, reaches),
    )


def scenario_framerate(model_path, seeds, reaches):
    jobs = []
    for rate in (15.0, 30.0, 60.0, 100.0):
        jobs += _build_jobs(model_path, [0.06], ("fixed", "governor", "smith", "lqr"),
                            seeds, reaches, control_dt=1.0 / rate)
    sc = Scenario(name="framerate", xlabel="control rate (Hz)", jobs=jobs)
    return sc


def scenario_noise(model_path, seeds, reaches):
    jobs = []
    for noise in ((0.0005, 0.0005, 0.0005), (0.001, 0.001, 0.002),
                  (0.0015, 0.0015, 0.003), (0.002, 0.002, 0.004)):
        jobs += _build_jobs(model_path, [0.06], ("fixed", "governor"), seeds,
                            reaches, noise=noise)
    return Scenario(name="noise", xlabel="anisotropic noise (sigma x,y,z mm)", jobs=jobs)


def scenario_jitter(model_path, seeds, reaches):
    jobs = []
    for jitter in (0.0, 0.01, 0.02, 0.04):
        jobs += _build_jobs(model_path, [0.12], ("fixed", "governor", "smith", "lqr"),
                            seeds, reaches, delay_jitter=jitter)
    return Scenario(name="jitter", xlabel="delay jitter (ms, 1 sigma)", jobs=jobs)


def scenario_delay_jump(model_path, seeds, reaches):
    jobs = []
    schedules = [(), ((0.8, 0.18),), ((0.8, 0.0),), ((0.4, 0.18), (1.2, 0.06))]
    labels = ["constant 60 ms", "60 -> 180 ms at 0.8 s", "60 -> 0 ms at 0.8 s",
              "60 -> 180 -> 60 ms"]
    for schedule, label in zip(schedules, labels):
        jobs += _build_jobs(model_path, [0.06], ("fixed", "governor", "smith", "lqr"),
                            seeds, reaches, delay_schedule=schedule, duration=3.0,
                            label=label)
    sc = Scenario(name="delay_jump", xlabel="delay schedule", jobs=jobs)
    sc.schedules = schedules
    return sc


def scenario_fitts(model_path, seeds, reaches):
    """Fitts task: movement amplitude x target width, at a 60 ms visual delay.

    Targets are placed along world -y from the start pose, the direction in
    which this arm has the largest usable workspace (a +x or radial target
    beyond 0.15 m is outside its reach).  ``width`` is the full width of the
    acceptance region; the arrival tolerance is ``width / 2`` around the
    target centre.  The controller gains stay frozen at their 30 ms values, so
    the Fitts condition is a stress test of the same frozen baselines rather
    than a separate tuning.
    """
    jobs = []
    cells = []
    direction = np.array([0.0, -1.0, 0.0])
    for distance in (0.10, 0.15, 0.20, 0.25):
        for width in (0.008, 0.016, 0.032):
            reach = distance * direction
            jobs += _build_jobs(model_path, [0.06], ("fixed", "governor"), seeds,
                                reaches, reach=reach, eps=width / 2.0,
                                vary_distance=False)
            cells.append((distance, width))
    sc = Scenario(name="fitts", xlabel="index of difficulty (bits)", jobs=jobs)
    sc.fitts_cells = cells
    return sc


SCENARIOS = {
    "delay": scenario_delay,
    "framerate": scenario_framerate,
    "noise": scenario_noise,
    "jitter": scenario_jitter,
    "delay_jump": scenario_delay_jump,
    "fitts": scenario_fitts,
}


# --------------------------------------------------------------------------
# execution and reporting
# --------------------------------------------------------------------------

def run_scenario(name, model_path, out_dir="reports", seeds=(0, 1, 2, 3, 4),
                 reaches=6, jobs=None, make_figures=True):
    if name not in SCENARIOS:
        raise ValueError(f"unknown scenario {name!r}; choose from {sorted(SCENARIOS)}")
    sc = SCENARIOS[name](model_path, seeds, reaches)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    if jobs is None or jobs == 1:
        sc.rows = [_job(j) for j in sc.jobs]
    else:
        workers = jobs if jobs and jobs > 0 else (os.cpu_count() or 1)
        with ProcessPoolExecutor(max_workers=workers) as ex:
            sc.rows = list(ex.map(_job, sc.jobs, chunksize=4))

    sc.summary = _summarise(sc)
    sc.comparisons = _comparisons(sc)
    _write_csv(sc, out / f"scenario_{name}.csv")
    (out / f"scenario_{name}_summary.json").write_text(
        json.dumps(sc.summary, indent=2, default=float), encoding="utf-8")
    (out / f"scenario_{name}_comparisons.json").write_text(
        json.dumps(sc.comparisons, indent=2, default=float), encoding="utf-8")
    if make_figures:
        _figure(sc, out.parent / "figures" / f"scenario_{name}.png")
    _report(sc, out / f"scenario_{name}.md")
    return sc


def run_all_scenarios(model_path, out_dir="reports", seeds=(0, 1, 2, 3, 4),
                      reaches=6, jobs=None, scenarios=None, make_figures=True):
    names = list(scenarios) if scenarios else list(SCENARIOS)
    results = {}
    for name in names:
        results[name] = run_scenario(name, model_path, out_dir=out_dir, seeds=seeds,
                                     reaches=reaches, jobs=jobs,
                                     make_figures=make_figures)
    _combined_report(results, Path(out_dir) / "scenarios_combined.md")
    return results


def _group_key(row, name):
    if name == "delay":
        return (round(row["delay_s"], 4), row["controller"])
    if name == "framerate":
        return (round(1.0 / row["control_dt_s"], 1), row["controller"])
    if name == "noise":
        return (round(row["noise_sigma"], 5), row["controller"])
    if name == "jitter":
        return (round(row["delay_jitter_s"], 4), row["controller"])
    if name == "delay_jump":
        return (row["label"], row["controller"])
    if name == "fitts":
        return (round(row["distance_m"], 4), round(2.0 * row["eps_m"], 4),
                row["controller"])
    return (row["controller"],)


def _summarise(sc):
    cells: dict = {}
    for row in sc.rows:
        key = _group_key(row, sc.name)
        cells.setdefault(key, []).append(row)
    out = {}
    keys = {}
    for key, rs in sorted(cells.items(), key=lambda kv: str(kv[0])):
        mt = [r["settling_time_s"] for r in rs if np.isfinite(r["settling_time_s"])]
        failures = len(rs) - len(mt)
        over = sum(1 for r in rs if r["penetration_m"] > r["eps_m"])
        lo, hi = bootstrap_ci(mt, stat=np.median) if mt else (float("nan"), float("nan"))
        wlo, whi = wilson_ci(over, len(rs))
        entry = {
            "n": len(rs),
            "mt_median_s": median(mt),
            "mt_ci95_s": [lo, hi],
            "failures": failures,
            "overshoot_rate": over / len(rs) if rs else float("nan"),
            "overshoot_ci95": [wlo, whi],
            "max_penetration_mm": float(np.max([r["penetration_m"] for r in rs]) * 1e3),
            "energy_mean_j": float(np.mean([r["energy_j"] for r in rs])),
            "final_error_mm": float(np.mean([r["final_error_m"] for r in rs]) * 1e3),
            "peak_torque_nm": float(np.mean([r["peak_torque_nm"] for r in rs])),
        }
        if sc.name == "noise":
            entry["noise_label"] = rs[0].get("noise_label", "")
        if sc.name == "fitts":
            d = float(key[0])
            width = float(key[1])
            entry["id_bits"] = fitts_id(d, width)
            entry["throughput_bps"] = (throughput(d, width, median(mt))
                                       if np.isfinite(median(mt)) else float("nan"))
        k = str(key)
        out[k] = entry
        keys[k] = key
    sc.cell_keys = keys
    sc.cell_rows = cells
    return out


def _cond_label(sc, parts):
    x = parts[0]
    if sc.name == "noise":
        return str(x)
    if isinstance(x, str):
        return x
    if sc.name in ("delay", "jitter"):
        return f"{1000*float(x):.0f} ms"
    if sc.name == "framerate":
        return f"{float(x):.0f} Hz"
    if sc.name == "fitts":
        return f"D={float(parts[0]):.3f} m, W={float(parts[1]):.4f} m"
    return f"{float(x):g}"


def _comparisons(sc):
    """Paired governor-vs-baseline comparisons with Holm correction."""
    cells = getattr(sc, "cell_rows", {})
    out = []
    for key, rs in cells.items():
        parts = sc.cell_keys[str(key)]
        if parts[-1] != "governor":
            continue
        gov = [r["settling_time_s"] for r in rs if np.isfinite(r["settling_time_s"])]
        if len(gov) < 2:
            continue
        for other in sorted({sc.cell_keys[k][-1] for k in sc.summary}):
            if other == "governor":
                continue
            okey = tuple(list(parts[:-1]) + [other])
            if okey not in cells:
                continue
            oth = [r["settling_time_s"] for r in cells[okey]
                   if np.isfinite(r["settling_time_s"])]
            if len(oth) < 2:
                continue
            out.append({
                "condition": _cond_label(sc, parts),
                "baseline": other,
                "n_governor": len(gov),
                "n_baseline": len(oth),
                "mt_governor_s": median(gov),
                "mt_baseline_s": median(oth),
                "delta_mt_pct": 100.0 * (median(gov) / median(oth) - 1.0),
                "cohens_d": cohens_d(gov, oth),
                "p_raw": permutation_test(gov, oth),
            })
    if out:
        adj = holm_bonferroni([o["p_raw"] for o in out])
        for o, a in zip(out, adj):
            o["p_holm"] = float(a)
    return out


def _write_csv(sc, path):
    if not sc.rows:
        return
    keys = list(sc.rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(sc.rows)


def _report(sc, path):
    lines = [f"# Scenario: {sc.name}", "",
             f"Manipulated variable: {sc.xlabel}. Cells contain the reaches of "
             f"all controllers under identical conditions.", ""]
    if sc.name == "delay":
        lines += ["| delay ms | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    elif sc.name == "framerate":
        lines += ["| rate Hz | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    elif sc.name == "noise":
        lines += ["| noise | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    elif sc.name == "jitter":
        lines += ["| jitter ms | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    elif sc.name == "delay_jump":
        lines += ["| delay schedule | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    else:
        lines += ["| distance m | width m | ID bits | controller | n | MT median s | MT 95% CI | failures | overshoot rate | max penetration mm | throughput bits/s |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]

    for key, s in sc.summary.items():
        parts = sc.cell_keys[key]
        if sc.name == "fitts":
            d, width, ctrl = parts
            lines.append(f"| {d:.3f} | {width:.4f} | {s['id_bits']:.2f} | {ctrl} | {s['n']} | "
                         f"{_fmt(s['mt_median_s'])} | [{_fmt(s['mt_ci95_s'][0])}, {_fmt(s['mt_ci95_s'][1])}] | "
                         f"{s['failures']} | {_pct(s['overshoot_rate'])} | "
                         f"{s['max_penetration_mm']:.2f} | {_fmt(s['throughput_bps'])} |")
        else:
            x, ctrl = parts[0], parts[-1]
            if sc.name == "noise":
                x = s.get("noise_label", f"{1000*x:.2g} mm")
            elif isinstance(x, str):
                pass
            elif sc.name in ("delay", "jitter"):
                x = f"{1000*x:.0f}"
            elif sc.name == "framerate":
                x = f"{x:.0f}"
            lines.append(f"| {x} | {ctrl} | {s['n']} | {_fmt(s['mt_median_s'])} | "
                         f"[{_fmt(s['mt_ci95_s'][0])}, {_fmt(s['mt_ci95_s'][1])}] | "
                         f"{s['failures']} | {_pct(s['overshoot_rate'])} | "
                         f"[{_pct(s['overshoot_ci95'][0])}, {_pct(s['overshoot_ci95'][1])}] | "
                         f"{s['max_penetration_mm']:.2f} | {s['energy_mean_j']:.1f} |")
    if sc.comparisons:
        lines += ["", "## Governor vs baselines (paired reaches, Holm-corrected)", "",
                  "ΔMT < 0 means the governor settles faster than that baseline. "
                  "`d` is Cohen's d on log movement time; p-values come from a "
                  "two-sided permutation test and are Holm-corrected across all "
                  "comparisons in this scenario.", "",
                  "| condition | baseline | ΔMT % | d | p | p (Holm) |",
                  "|---|---|---|---|---|---|"]
        for c in sc.comparisons:
            lines.append(
                f"| {c['condition']} | {c['baseline']} | {c['delta_mt_pct']:+.1f} | "
                f"{_fmt(c['cohens_d'])} | {_pval(c['p_raw'])} | {_pval(c['p_holm'])} |")
    lines += ["", "## Notes", "",
              "* MT is the settling time (last control step at which the Cartesian error",
              "  exceeds the arrival threshold); the interval is a percentile",
              "  bootstrap over reaches of the same median.",
              "* Failures are reaches that never settled inside the simulated window.",
              "* Overshoot rate counts reaches whose penetration past the target exceeds",
              "  the arrival tolerance `eps` (a real overshoot beyond the accepted band).",
              "  Max penetration is the largest such excursion across the cell.",
              "* Reach directions are a fixed low-discrepancy cone (paired across",
              "  controllers); the seed changes only the measurement-noise realisation.",
              "* Every baseline was tuned once at the nominal 30 ms condition and then",
              "  frozen (`reports/baseline_tuning.md`); the governor has no tuned",
              "  parameter and reads only the measured delay and frame period.",
              "* The delay-aware laws (governor, Smith) recompute their schedule from",
              "  the delay reported for the current frame; baselines keep their gains.",
              "* Simulation only; motor parameters are representative, not measured.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def _fmt(v):
    return "n/a" if not np.isfinite(v) else f"{v:.2f}"


def _pct(v):
    return "n/a" if not np.isfinite(v) else f"{100*v:.1f} %"


def _pval(v):
    if not np.isfinite(v):
        return "n/a"
    if v < 1e-4:
        return "<1e-4"
    return f"{v:.4f}"


def _figure(sc, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    cells = sc.summary
    ctrls = sorted({sc.cell_keys[k][-1] for k in cells})
    if sc.name == "fitts":
        xs = sorted({s["id_bits"] for s in cells.values()
                     if np.isfinite(s.get("id_bits", float("nan")))})
    else:
        xs = sorted({sc.cell_keys[k][0] for k in cells},
                    key=lambda v: (isinstance(v, str), v))
    if not xs:
        return None
    categorical = any(isinstance(x, str) for x in xs)
    if categorical:
        xpos = {x: i for i, x in enumerate(xs)}
        denom = max(1, len(xs) - 1)
    else:
        xmin, xmax = float(min(xs)), float(max(xs))
        if xmax <= xmin:
            xmax = xmin + 1.0
        denom = xmax - xmin

    def xpix(x):
        if categorical:
            return x0 + xpos[x] / denom * (x1 - x0)
        return x0 + (float(x) - xmin) / denom * (x1 - x0)

    W, H = 980, 520
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    x0, y0, x1, y1 = 90, 60, W - 40, H - 90
    d.rectangle([x0, y0, x1, y1], outline=INK, width=2)
    d.text((x0, y0 - 30), f"{sc.name}: movement time vs {sc.xlabel}",
           font=font(17, True), fill=INK)
    vals = [s["mt_median_s"] for s in cells.values()
            if np.isfinite(s["mt_median_s"])]
    ymax = (max(vals) if vals else 1.0) * 1.2
    for ctrl in ctrls:
        pts = []
        if sc.name == "fitts":
            items = sorted(((s["id_bits"], s) for k, s in cells.items()
                            if sc.cell_keys[k][-1] == ctrl
                            and np.isfinite(s["mt_median_s"])),
                           key=lambda t: t[0])
            pts = [(xpix(i), y1 - (s["mt_median_s"] / ymax) * (y1 - y0), s)
                   for i, s in items]
        else:
            for x in xs:
                key = None
                for k in cells:
                    p = sc.cell_keys[k]
                    if p[-1] == ctrl and p[0] == x:
                        key = k
                        break
                if key is None:
                    continue
                s = cells[key]
                px = xpix(x)
                py = y1 - (s["mt_median_s"] / ymax) * (y1 - y0)
                pts.append((px, py, s))
        if len(pts) < 1:
            continue
        col = COLORS.get(ctrl, SLATE)
        d.line([(a[0], a[1]) for a in pts], fill=col, width=3)
        for px, py, s in pts:
            d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
            lo, hi = s["mt_ci95_s"]
            if np.isfinite(lo) and np.isfinite(hi):
                d.line([(px, y1 - (lo / ymax) * (y1 - y0)),
                        (px, y1 - (hi / ymax) * (y1 - y0))], fill=col, width=2)
        d.text((x1 - 150, y0 + 6 + 20 * ctrls.index(ctrl)), ctrl,
               font=font(15, True), fill=col)
    for k in range(5):
        frac = k / 4
        y = y1 - frac * (y1 - y0)
        d.text((x0 - 70, y - 8), f"{frac*ymax:5.2f}", font=font(13), fill=GREY)
    if categorical:
        for x in xs:
            px = xpix(x)
            label = str(x)
            if len(label) > 18:
                label = label[:17] + "."
            d.text((px - 24, y1 + 12), label, font=font(11), fill=GREY)
    else:
        d.text((x0, y1 + 12), f"{xmin:g}", font=font(13), fill=GREY)
        d.text((x1 - 60, y1 + 12), f"{xmax:g}", font=font(13), fill=GREY)
    d.text((x0, y1 + 40), sc.xlabel, font=font(15), fill=INK)
    d.text((x0 - 70, y0 - 30), "MT (s)", font=font(15), fill=INK)
    img.save(path, dpi=(300, 300))
    return path


def _combined_report(results, path):
    lines = ["# Reaching benchmark — combined scenario report", "",
             "All scenarios share the same arm, task and analysis conventions. "
             "Each table contrasts the governor with the fastest baseline in "
             "that cell; the full per-controller tables and the paired "
             "Holm-corrected statistics are in `scenario_<name>.md`.", ""]
    for name, sc in results.items():
        lines += [f"## {name}", "",
                  f"Manipulated variable: {sc.xlabel}. "
                  f"{len(sc.rows)} reaches in {len(sc.summary)} cells.", ""]
        gov_cells = [k for k in sc.summary if sc.cell_keys[k][-1] == "governor"]
        if gov_cells:
            gov_fail = sum(sc.summary[k]["failures"] for k in gov_cells)
            gov_over = sum(1 for k in gov_cells
                           if sc.summary[k]["overshoot_rate"] > 0)
            lines.append(f"* Governor: {gov_fail} failures, overshoot in "
                         f"{gov_over}/{len(gov_cells)} cells.")
        if sc.comparisons:
            sig = [c for c in sc.comparisons if c["p_holm"] < 0.05]
            faster = [c for c in sig if c["delta_mt_pct"] < 0]
            lines.append(f"* Holm-significant paired comparisons: {len(sig)}"
                         f" of {len(sc.comparisons)}; governor faster in "
                         f"{len(faster)}, slower in {len(sig) - len(faster)}.")
        lines += ["",
                  "| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |",
                  "|---|---|---|---|---|---|---|"]
        for k in gov_cells:
            parts = sc.cell_keys[k]
            g = sc.summary[k]
            best = None
            for ok, os_ in sc.summary.items():
                op = sc.cell_keys[ok]
                if op[-1] == "governor" or list(op[:-1]) != list(parts[:-1]):
                    continue
                if not np.isfinite(os_["mt_median_s"]):
                    continue
                if best is None or os_["mt_median_s"] < best[1]["mt_median_s"]:
                    best = (op[-1], os_)
            if best is None:
                lines.append(f"| {_cond_label(sc, parts)} | "
                             f"{_fmt(g['mt_median_s'])} | — | — | — | "
                             f"{_pct(g['overshoot_rate'])} | — |")
                continue
            bname, b = best
            delta = (100.0 * (g["mt_median_s"] / b["mt_median_s"] - 1.0)
                     if np.isfinite(g["mt_median_s"]) else float("nan"))
            lines.append(
                f"| {_cond_label(sc, parts)} | {_fmt(g['mt_median_s'])} | {bname} | "
                f"{_fmt(b['mt_median_s'])} | {delta:+.1f} | "
                f"{_pct(g['overshoot_rate'])} | {_pct(b['overshoot_rate'])} |")
        lines.append("")
    lines += ["## Conventions", "",
              "* MT: settling time (s); median with percentile bootstrap 95 % CI.",
              "* Overshoot rate: fraction of reaches whose penetration past the",
              "  target exceeds the arrival tolerance `eps`; Wilson 95 % CI.",
              "* Failures: reaches that never settled inside the window.",
              "* Paired comparisons use identical reach tasks across controllers;",
              "  two-sided permutation test on log MT, Cohen's d for effect size,",
              "  Holm correction across all comparisons within a scenario.",
              "* All controllers share the same delayed, noisy measurement and the",
              "  same plant; only the control law differs.",
              "* Simulation only; the arm and the motors are a design study.",
              ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def _fmt_cell(name, key, s, cell_keys=None):
    parts = cell_keys[key] if cell_keys else (key,)
    if name == "fitts":
        return (f"D={parts[0]:.3f} m, W={parts[1]:.4f} m, {parts[2]}: "
                f"MT={_fmt(s['mt_median_s'])} s, overshoot {_pct(s['overshoot_rate'])}, "
                f"TP={_fmt(s.get('throughput_bps', float('nan')))} bits/s")
    x = parts[0]
    if isinstance(x, str):
        pass
    elif name in ("delay", "jitter"):
        x = f"{1000*x:.0f} ms"
    elif name == "framerate":
        x = f"{x:.0f} Hz"
    elif name == "noise":
        x = s.get("noise_label", f"{1000*x:.1f} mm")
    else:
        x = f"{x:g}"
    return (f"{x}, {parts[-1]}: MT={_fmt(s['mt_median_s'])} s "
            f"[{_fmt(s['mt_ci95_s'][0])}, {_fmt(s['mt_ci95_s'][1])}], "
            f"overshoot {_pct(s['overshoot_rate'])}, failures {s['failures']}, "
            f"energy {s['energy_mean_j']:.1f} J")
