"""v8 scenario: does a higher camera frame rate improve oracle-free reaching?

Usage (from the robosimu folder):
    python scripts/run_v8_framerate.py         # resumable
Outputs: reports/v8_framerate.jsonl (one line per reach)
Summary table: reports/v8_framerate.md
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.perceptual_governor import PerceptualGovernor
from robosimu.simulator import SimConfig, simulate_reach
from robosimu.stereo import StereoCameraConfig

M = load_model("models/six_axis_arm.json")
Q0 = np.array([0.0, -0.3, 0.6, 0.0, 0.6, 0.0])
P0 = end_effector_position(M, Q0)
FPS = [15, 30, 60, 100, 200]
DELAYS = [0.06, 0.15]
RULES = ["n8", "w80"]          # n8: 8 frames at every rate; w80: an 80 ms window
SEEDS = [0, 1]
OUT = "reports/v8_framerate.jsonl"


def make_targets(n, seed, rmin=0.04, rmax=0.10):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        v = rng.normal(size=3)
        v /= np.linalg.norm(v)
        out.append(np.round(v * rng.uniform(rmin, rmax), 4))
    return out


TEST = make_targets(12, 1001)    # the test targets of run_v5.py


def run(ctl, dx, dl, dur, seed, cdt):
    st = StereoCameraConfig()
    st.pixel_noise_px = 0.25
    st.disparity_noise_px = 0.35
    cfg = SimConfig(duration=dur, delay=dl, noise=0.0, eps=0.004, stereo=st,
                    zero_oracle=True, neck=False, seed=seed, control_dt=cdt)
    return simulate_reach(M, Q0, P0 + dx, ctl, cfg)


def metrics(r, cdt, tol=0.005):
    e = r.error
    bad = np.where(e > tol)[0]
    settled = bool(len(bad) == 0 or bad[-1] < len(e) - 1)
    mt = float((bad[-1] + 1) * cdt) if len(bad) and settled else (0.0 if settled else None)
    return {"final_mm": float(e[-1] * 1000), "os_pct": float(r.metrics.get("overshoot_pct", 0.0)),
            "mt_s": mt, "settled": settled}


def main():
    os.makedirs("reports", exist_ok=True)
    done = set()
    if os.path.exists(OUT):
        for line in open(OUT):
            d = json.loads(line)
            done.add((d["fps"], d["delay"], d["var"], d["tgt"], d["seed"]))
    for fps in FPS:
        cdt = 1.0 / fps
        cal = PerceptualGovernor(T=cdt)          # body schema re-identified at this frame rate
        run(cal, np.zeros(3), 0.06, 8.0, 0, cdt)
        J0, L0 = cal.J.copy(), cal.L
        print(f"{fps} fps: identified L = {L0}")
        for dl in DELAYS:
            for var in RULES:
                nw = 8 if var == "n8" else max(1, int(round(0.08 * fps)))
                for s in SEEDS:
                    for i, dx in enumerate(TEST):
                        if (fps, dl, var, i, s) in done:
                            continue
                        g = PerceptualGovernor(J0=J0, L0=L0, T=cdt, nwin=nw)
                        r = run(g, dx, dl, 5.0, 1000 * s + i, cdt)
                        d = dict(fps=fps, delay=dl, var=var, nwin=nw, L=L0, tgt=i, seed=s,
                                 k=float(g.info.get("k", 0.0)), deff=float(g.info.get("d_eff", 0.0)),
                                 **metrics(r, cdt))
                        with open(OUT, "a") as f:
                            f.write(json.dumps(d) + "\n")


if __name__ == "__main__":
    main()
