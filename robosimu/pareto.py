"""A3 Pareto scan: c_max(delta) for the delay-scaled laws, and frontier distance.

For each law (``dps`` — DelayScaledP, ``gov`` — GovernorCartesian) and each
delay, the dimensionless gain ``c`` is scanned on the TEST targets.  The
largest c whose worst-case overshoot stays <= 0.5 % is reported as
``c_max(delta)``; hypothesis H3 asks whether ``c_max`` coincides with the
classical monotone bound ``1/e``.  The frontier distance measures how far a
law's best safe point lies from the best safe point of either law at the same
delay.

CLI:
    python -m robosimu.pareto --out reports
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from . import a3
from .tasks import test_targets

C_TOL = 0.5   # overshoot tolerance (%) for c_max
KAPPA_STAR = 1.0 / math.e


def scan_c(law: str, delay: float, cs, targets, seeds, clip_kappa=False):
    rows = []
    for c in cs:
        out = a3.run_condition(law, delay, targets, seeds, c=float(c),
                               clip_kappa=clip_kappa)
        rows.append({
            "c": float(c),
            "overshoot_max": max(o.overshoot for o in out),
            "mt_median": float(np.median([o.mt for o in out])),
            "settled_rate": float(np.mean([o.settled for o in out])),
        })
    return rows


def c_max(scan, tol: float = C_TOL):
    safe = [r for r in scan if r["overshoot_max"] <= tol]
    return max((r["c"] for r in safe), default=None)


def frontier(rows_by_law: dict[str, list[dict]]):
    """Best safe point across laws and each law's distance to it (%)."""
    best = None
    for law, rows in rows_by_law.items():
        safe = [r for r in rows if r["overshoot_max"] <= C_TOL]
        if not safe:
            continue
        cand = min(safe, key=lambda r: r["mt_median"])
        if best is None or cand["mt_median"] < best["mt_median"]:
            best = cand
    out = {}
    for law, rows in rows_by_law.items():
        safe = [r for r in rows if r["overshoot_max"] <= C_TOL]
        if not safe or best is None:
            out[law] = None
            continue
        cand = min(safe, key=lambda r: r["mt_median"])
        dist = (cand["mt_median"] - best["mt_median"]) / best["mt_median"] * 100.0
        out[law] = {"c": cand["c"], "mt_median": cand["mt_median"],
                    "frontier_distance_pct": dist}
    return out


def run_pareto(delays, cs, targets=None, seeds=(0, 1, 2), out_dir="reports"):
    targets = targets or test_targets(seeds)
    summary = {"c_tol_pct": C_TOL, "kappa_star": KAPPA_STAR,
               "seeds": list(seeds), "delays": list(delays), "per_delay": {}}
    for d in delays:
        scans = {
            "dps": scan_c("dps", d, cs, targets, seeds, clip_kappa=False),
            "gov": scan_c("governor", d, cs, targets, seeds, clip_kappa=False),
        }
        cm = {law: c_max(scan) for law, scan in scans.items()}
        fr = frontier(scans)
        summary["per_delay"][f"{d:.3f}"] = {
            "c_max": cm, "frontier": fr,
            "h3_ok": {law: (cm[law] is not None and abs(cm[law] - KAPPA_STAR) <= 0.05)
                      for law in scans},
            "scans": scans,
        }
        print(f"delta={d * 1000:.0f} ms  c_max(dps)={cm['dps']}  c_max(gov)={cm['gov']}")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "a3_pareto.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out / "a3_pareto.md").write_text(_markdown(summary), encoding="utf-8")
    return summary


def _markdown(summary: dict) -> str:
    lines = [
        "# A3 Pareto — c_max(delta) ve cepheye uzaklik",
        "",
        f"* Kriter: en yüksek `c` — aşma ≤ {summary['c_tol_pct']}% (TEST hedefleri, "
        f"seeds={summary['seeds']})",
        f"* Klasik sınır: `1/e = {summary['kappa_star']:.4f}` · H3 toleransı: ±0.05",
        "",
        "| Gecikme (ms) | c_max(dps) | c_max(gov) | \\|c_max-1/e\\|(dps) | H3 | dps cephe uzaklığı | gov cephe uzaklığı |",
        "|---|---|---|---|---|---|---|",
    ]
    for key, row in summary["per_delay"].items():
        d_ms = float(key) * 1000.0
        cm = row["c_max"]
        fr = row["frontier"]
        dps = cm["dps"]
        dev = abs(dps - KAPPA_STAR) if dps is not None else float("nan")
        h3 = "PASS" if row["h3_ok"]["dps"] else "FAIL"
        fd = (f"{fr['dps']['frontier_distance_pct']:.1f}%" if fr.get("dps") else "—")
        fg = (f"{fr['gov']['frontier_distance_pct']:.1f}%" if fr.get("gov") else "—")
        lines.append(f"| {d_ms:.0f} | {dps if dps is not None else '—'} | "
                     f"{cm['gov'] if cm['gov'] is not None else '—'} | "
                     f"{dev:.3f} | {h3} | {fd} | {fg} |")
    lines.append("")
    lines.append("> H3: tüm gecikmelerde `|c_max(dps) − 1/e| ≤ 0.05` ise desteklenir; "
                 "aksi hâlde yasa `c(δ)` düzeltmesiyle güncellenir veya iddia daraltılır "
                 "(kapı G1, `TODO_dogru_yol.md`).")
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delays", default="0.03,0.06,0.10,0.15,0.20")
    ap.add_argument("--c-grid", default="0.05,0.60,12")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--out", default="reports")
    args = ap.parse_args()
    delays = [float(v) for v in args.delays.split(",") if v.strip()]
    c0, c1, n = (float(v) for v in args.c_grid.split(","))
    cs = np.linspace(c0, c1, int(n))
    seeds = tuple(int(v) for v in args.seeds.split(",") if v.strip())
    run_pareto(delays, cs, seeds=seeds, out_dir=args.out)
    print(f"wrote {args.out}/a3_pareto.md")


if __name__ == "__main__":
    main()
