"""v5 oracle-free benchmark: Perceptual 1/e Governor vs baselines.

Usage (from the robosimu folder):
    python scripts/run_v5.py            # full run, resumable
Outputs: reports/v5_results.jsonl and reports/v5_summary.md
"""
from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robosimu.controllers import make_controller
from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.perceptual_governor import PerceptualGovernor
from robosimu.simulator import SimConfig, simulate_reach
from robosimu.stereo import StereoCameraConfig

M = load_model("models/six_axis_arm.json")
Q0 = np.array([0.0, -0.3, 0.6, 0.0, 0.6, 0.0])
P0 = end_effector_position(M, Q0)
DELAYS = [0.03, 0.06, 0.10, 0.15, 0.20]
CTLS = ["pg", "gov", "pid30", "fixed30", "pg_mis"]
CTLS = CTLS + ["fixed_tuned"]
ABLATIONS = ["abl_noL", "abl_n1", "abl_broy", "abl_nodelay"]   # run at 60 and 150 ms
SEEDS = [0, 1]
OUT = "reports/v5_results.jsonl"


def make_targets(n, seed, rmin=0.04, rmax=0.10):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        v = rng.normal(size=3)
        v /= np.linalg.norm(v)
        out.append(np.round(v * rng.uniform(rmin, rmax), 4))
    return out


TEST = make_targets(12, 1001)   # disjoint from TUNE = make_targets(6, 2002)


def run(ctl, dx, dl, dur, seed, zo):
    st = StereoCameraConfig()
    st.pixel_noise_px = 0.25
    st.disparity_noise_px = 0.35
    cfg = SimConfig(duration=dur, delay=dl, noise=0.0, eps=0.004, stereo=st,
                    zero_oracle=zo, neck=False, seed=seed)
    return simulate_reach(M, Q0, P0 + dx, ctl, cfg)


def metrics(r, tol=0.005):
    e = r.error
    bad = np.where(e > tol)[0]
    settled = bool(len(bad) == 0 or bad[-1] < len(e) - 1)
    mt = float((bad[-1] + 1) * 0.01) if len(bad) and settled else (0.0 if settled else None)
    return {"final_mm": float(e[-1] * 1000), "os_pct": float(r.metrics.get("overshoot_pct", 0.0)),
            "mt_s": mt, "settled": settled, "energy_J": float(r.energy)}


def calibrate():
    c = PerceptualGovernor()
    run(c, np.zeros(3), 0.06, 3.0, 0, True)
    return c.J.copy(), c.L


def factory(name, dl, J0, L0):
    if name == "pg":
        return PerceptualGovernor(J0=J0, L0=L0), True
    if name == "pg_mis":
        return PerceptualGovernor(J0=J0, L0=L0, assumed_delay=dl / 2), True
    if name == "abl_noL":
        return PerceptualGovernor(J0=J0, L0=None, identify_lag=False), True
    if name == "abl_n1":
        return PerceptualGovernor(J0=J0, L0=L0, nwin=1), True
    if name == "abl_broy":
        return PerceptualGovernor(J0=J0, L0=L0, broyden=True), True
    if name == "abl_nodelay":
        return PerceptualGovernor(J0=J0, L0=L0, assumed_delay=1e-3), True
    if name == "gov":
        return make_controller("governor", delay=dl), False
    if name == "pid30":
        return make_controller("pid", delay=0.03), False
    if name == "fixed30":
        return make_controller("fixed", delay=0.03), False
    if name == "fixed_tuned":   # k chosen per delay on TUNE targets (run_v5_robustness.py)
        return make_controller("fixed", delay=dl, k=1.5), False
    raise KeyError(name)


def main():
    os.makedirs("reports", exist_ok=True)
    done = set()
    if os.path.exists(OUT):
        for line in open(OUT):
            d = json.loads(line)
            done.add((d["ctl"], d["delay"], d["tgt"], d["seed"]))
    J0, L0 = calibrate()
    print("body schema: L =", L0)
    t0 = time.time()
    jobs = []
    for s in SEEDS:
        for dl in DELAYS:
            names = CTLS if s == 0 else ["pg", "pg_mis", "gov"]
            if s == 0 and dl in (0.06, 0.15):
                names = names + ABLATIONS
            jobs += [(s, dl, n) for n in names]
    for s, dl, name in jobs:
        if True:
            if True:
                for i, dx in enumerate(TEST):
                    if (name, dl, i, s) in done:
                        continue
                    c, zo = factory(name, dl, J0, L0)
                    m = metrics(run(c, dx, dl, 5.0, 1000 * s + i, zo))
                    with open(OUT, "a") as f:
                        f.write(json.dumps(dict(ctl=name, delay=dl, tgt=i, seed=s, **m)) + "\n")
    print("done in %.0f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
