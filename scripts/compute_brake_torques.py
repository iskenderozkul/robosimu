"""Compute holding-brake (tutma freni) torques for a model.

Holding brakes are zero-power joint brakes that engage the moment a drive
pulse ends (isometric-tone analogue; see docs/teorem_ve_felsefe.md §2.4).
They must be sized against the worst-case gravity load of the distal chain:

    brake_i = factor * max_q |tau_g,i(q)|     (+ small floor for j1 / j6)

Because the full anatomical grid is intractable (40^6 cells), the maximum is
estimated from random uniform samples across the joint limits plus all named
postures in the model, and the factor (default 1.5) covers the residual.

Writes the result into the model JSON itself:
  * each link's motor gains ``brake_torque_nm`` (Nm),
  * the model gains top-level ``"brake": {"auto": true, "factor": ..., ...}``.

Usage:
    python scripts/compute_brake_torques.py [--model models/human_arm.json]
                                            [--factor 1.5] [--samples 15000] [--seed 42]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robosimu.model import load_model, load_postures
from robosimu.dynamics import gravity_torque

# j1 (trunk yaw) and j6 (hand rotation) see ~zero gravity moment along
# near-vertical axes. The floors keep those brakes meaningful against
# back-driving and friction instead of writing a useless 0.0.
FLOOR_NM = {0: 1.0, 5: 0.05}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/human_arm.json")
    ap.add_argument("--factor", type=float, default=1.5)
    ap.add_argument("--samples", type=int, default=15000)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    path = Path(args.model)
    data = json.loads(path.read_text(encoding="utf-8"))
    model = load_model(path)
    limits = np.array([np.deg2rad(l["limits_deg"]) for l in data["links"]])

    rng = np.random.default_rng(args.seed)
    max_tau = np.zeros(len(limits))

    for _ in range(args.samples):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        max_tau = np.maximum(max_tau, np.abs(gravity_torque(model, q)))

    # named postures are the physically most likely operating points
    for name, angles in load_postures(path).items():
        q = np.deg2rad(np.asarray(angles, dtype=float))
        max_tau = np.maximum(max_tau, np.abs(gravity_torque(model, q)))
        print(f"posture {name:16s} -> {np.round(np.abs(gravity_torque(model, q)), 3)}")

    for i, (val, l) in enumerate(zip(max_tau, data["links"])):
        brake = args.factor * float(val)
        brake = max(brake, FLOOR_NM.get(i, 0.0))
        l.setdefault("motor", {})["brake_torque_nm"] = round(brake, 3)
        print(f"{l['name']:28s} max|tau_g|={val:7.3f} Nm  brake={brake:7.3f} Nm")

    data["brake"] = {
        "auto": True,
        "factor": args.factor,
        "samples": args.samples,
        "seed": args.seed,
        "note": "tau_b = factor * max|tau_g(q)| over random anatomical samples "
                "(+ named postures). j1/j6 floors applied. Recompute with "
                "scripts/compute_brake_torques.py after geometry changes.",
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
