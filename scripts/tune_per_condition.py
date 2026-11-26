"""Tune the delay-scaled gain c per delay — TUNE targets only (no leakage).

Pre-registered criterion (``docs/dogru_yol_Faz0.md`` §0.4/0.5): for each
delay, choose the **largest** c whose worst-case overshoot on the TUNE set is
<= 0.5 %; ties are broken by the smaller median movement time.  The TEST and
OOD sets are never consulted here.

Usage:
    python scripts/tune_per_condition.py --out reports/a3_tuning.json
    python scripts/tune_per_condition.py --time-check 100
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robosimu import a3
from robosimu.tasks import tune_targets


def parse_floats(s: str) -> list[float]:
    return [float(v) for v in s.split(",") if v.strip()]


def tune_delay(delay: float, cs, seeds=(0, 1, 2)) -> dict:
    """Scan c at one delay on the TUNE set; return rows + selected c*.

    Criterion (pre-registered): largest c with worst-case overshoot <= 0.5 %;
    ties broken by smaller median MT.  Only ``tune_targets`` amplitudes are
    ever evaluated — the TEST/OOD sets are never visible here (T6).
    """
    tgt = tune_targets(tuple(seeds))
    rows = []
    for c in cs:
        out = a3.run_condition("dps", delay, tgt, tuple(seeds), c=float(c))
        rows.append({"c": float(c),
                     "overshoot_max": max(o.overshoot for o in out),
                     "mt_median": float(np.median([o.mt for o in out]))})
    safe = [r for r in rows if r["overshoot_max"] <= 0.5]
    best = (min(safe, key=lambda r: r["mt_median"]) if safe
            else min(rows, key=lambda r: r["overshoot_max"]))
    return {"c_star": best["c"], "overshoot_max": best["overshoot_max"],
            "mt_median": best["mt_median"], "scan": rows}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delays", default="0.03,0.06,0.10,0.15,0.20")
    ap.add_argument("--c-grid", default="0.05,0.60,12",
                    help="start,stop,num for the c scan")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--out", default="reports/a3_tuning.json")
    ap.add_argument("--time-check", type=int, default=0,
                    help="run N reaches and print seconds/reach, then exit")
    args = ap.parse_args()

    if args.time_check:
        tgt = tune_targets()
        t0 = time.time()
        for _ in range(args.time_check):
            a3.simulate_reach(0.2, 0.06, a3.make_controller("dps", delay=0.06))
        dt = time.time() - t0
        print(f"{args.time_check} reaches in {dt:.2f} s -> {dt / args.time_check * 1000:.2f} ms/reach")
        return

    delays = parse_floats(args.delays)
    c0, c1, n = (float(v) for v in args.c_grid.split(","))
    cs = np.linspace(c0, c1, int(n))
    seeds = tuple(int(v) for v in parse_floats(args.seeds))
    tgt = tune_targets(seeds)

    result = {"criterion": "largest c with worst-case overshoot <= 0.5% on TUNE; "
                          "tie-break: smaller median MT",
              "seeds": list(seeds), "delays": delays, "c_grid": list(cs), "per_delay": {}}
    for d in delays:
        res = tune_delay(d, cs, seeds)
        result["per_delay"][f"{d:.3f}"] = res
        print(f"delta={d * 1000:.0f} ms -> c*={res['c_star']:.3f} "
              f"(os_max={res['overshoot_max']:.2f}%, mt={res['mt_median']:.2f} s)")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
