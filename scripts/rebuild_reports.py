"""Rebuild the scenario Markdown reports, JSON summaries and figures from the
cached per-reach CSV files, without re-running any simulation.

Use this after editing the reporting code so that the published tables and
figures are produced by the current version of the analysis.

    python scripts/rebuild_reports.py --out reports
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robosimu.scenarios import (SCENARIOS, _combined_report, _comparisons,
                                _figure, _report, _summarise)

STRING_FIELDS = {"controller", "label", "noise_label"}


def read_rows(path: Path) -> list[dict]:
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for raw in csv.DictReader(fh):
            row = {}
            for key, value in raw.items():
                if key in STRING_FIELDS:
                    row[key] = value
                    continue
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = value
            rows.append(row)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(ROOT / "reports"))
    ap.add_argument("--model", default=str(ROOT / "models" / "six_axis_arm.json"))
    args = ap.parse_args()
    out = Path(args.out)
    results = {}
    for name, builder in SCENARIOS.items():
        csv_path = out / f"scenario_{name}.csv"
        if not csv_path.exists():
            print(f"skip {name}: no {csv_path.name}")
            continue
        sc = builder(args.model, seeds=(0,), reaches=1)
        sc.jobs = []
        sc.rows = read_rows(csv_path)
        sc.summary = _summarise(sc)
        sc.comparisons = _comparisons(sc)
        _report(sc, out / f"scenario_{name}.md")
        (out / f"scenario_{name}_summary.json").write_text(
            json.dumps(sc.summary, indent=2, default=float), encoding="utf-8")
        (out / f"scenario_{name}_comparisons.json").write_text(
            json.dumps(sc.comparisons, indent=2, default=float), encoding="utf-8")
        _figure(sc, out.parent / "figures" / f"scenario_{name}.png")
        results[name] = sc
        print(f"rebuilt {name}: {len(sc.rows)} reaches, {len(sc.summary)} cells")
    if results:
        _combined_report(results, out / "scenarios_combined.md")
        print("rebuilt scenarios_combined.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
