"""v5 robustness experiments (paper Sections 5.4-5.5).

1. Retune the fixed-gain law per delay on TUNE targets  -> reports/v5_tune_fixed.jsonl
2. Plant shift: joint servo wn in {5, 10, 30} rad/s, no retuning of the
   fixed gain; the oracle-free law re-runs babbling      -> reports/v5_plant_shift.jsonl

Usage (from the robosimu folder):  python scripts/run_v5_robustness.py
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_v5 import M, P0, Q0, TEST, make_targets, metrics  # noqa: E402
from robosimu.controllers import make_controller  # noqa: E402
from robosimu.perceptual_governor import PerceptualGovernor  # noqa: E402
from robosimu.simulator import SimConfig, simulate_reach  # noqa: E402
from robosimu.stereo import StereoCameraConfig  # noqa: E402

TUNE = make_targets(6, 2002)
KGRID = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0]
DELAYS = [0.03, 0.06, 0.10, 0.15, 0.20]


def run(ctl, dx, dl, seed, zo, wn=10.0, dur=5.0):
    st = StereoCameraConfig()
    st.pixel_noise_px = 0.25
    st.disparity_noise_px = 0.35
    cfg = SimConfig(duration=dur, delay=dl, noise=0.0, eps=0.004, stereo=st,
                    zero_oracle=zo, neck=False, seed=seed, wn=wn)
    return simulate_reach(M, Q0, P0 + np.asarray(dx), ctl, cfg)


def write(path, rec):
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def main():
    os.makedirs("reports", exist_ok=True)
    out = "reports/v5_tune_fixed.jsonl"
    if not os.path.exists(out):
        for dl in DELAYS:
            for k in KGRID:
                for i, dx in enumerate(TUNE[:4]):
                    m = metrics(run(make_controller("fixed", delay=dl, k=k), dx, dl, 500 + i, False))
                    write(out, dict(delay=dl, k=k, tgt=i, **m))
    out = "reports/v5_plant_shift.jsonl"
    if not os.path.exists(out):
        plan = {5.0: ["fixed_k1.0", "fixed_k1.5", "fixed_k2.0"], 10.0: ["fixed_k1.5"],
                30.0: ["fixed_k1.5", "fixed_k3.0"]}
        for wn, fixed in plan.items():
            cal = PerceptualGovernor(wn=wn)
            run(cal, np.zeros(3), 0.06, 0, True, wn=wn, dur=4.0)
            for dl in (0.06, 0.15):
                for i, dx in enumerate(TEST[:8]):
                    m = metrics(run(PerceptualGovernor(J0=cal.J, L0=cal.L, wn=wn), dx, dl, i, True, wn=wn))
                    write(out, dict(wn=wn, ctl="pg", delay=dl, tgt=i, L=cal.L, **m))
                    for name in fixed:
                        c = make_controller("fixed", delay=dl, k=float(name[7:]))
                        m = metrics(run(c, dx, dl, i, False, wn=wn))
                        write(out, dict(wn=wn, ctl=name, delay=dl, tgt=i, **m))
    out = "reports/v5_human_arm.jsonl"
    if not os.path.exists(out):
        from robosimu.model import load_model, load_postures
        H = load_model("models/human_arm.json")
        post = load_postures("models/human_arm.json")
        st = StereoCameraConfig(baseline_m=0.064, focal_px=520.0, cam_pos=(-0.10, 0.0, 1.12),
                                look_at=(0.35, 0.0, 0.85))
        st.pixel_noise_px = 0.25
        st.disparity_noise_px = 0.35

        def hrun(ctl, q0, dx, dl, seed, zo, dur=5.0):
            from robosimu.kinematics import end_effector_position
            cfg = SimConfig(duration=dur, delay=dl, noise=0.0, eps=0.004, stereo=st,
                            zero_oracle=zo, neck=False, seed=seed)
            return simulate_reach(H, q0, end_effector_position(H, q0) + np.asarray(dx), ctl, cfg)
        for name in ("ready_desk", "low_table_pick", "shelf_pre_reach"):
            q0 = np.radians(post[name])
            cal = PerceptualGovernor()
            hrun(cal, q0, np.zeros(3), 0.06, 0, True, dur=4.0)
            for dl in (0.06, 0.15):
                for i, dx in enumerate(TEST[:8]):
                    for ctl, c, zo in (("pg", PerceptualGovernor(J0=cal.J, L0=cal.L), True),
                                       ("gov", make_controller("governor", delay=dl), False),
                                       ("fixed_k1.5", make_controller("fixed", delay=dl, k=1.5), False)):
                        m = metrics(hrun(c, q0, dx, dl, i, zo))
                        write(out, dict(ctl=ctl, posture=name, delay=dl, tgt=i, **m))
    out = "reports/v5_intermittent_gaze.jsonl"
    if not os.path.exists(out):
        cal = PerceptualGovernor()
        run(cal, np.zeros(3), 0.06, 0, True, dur=3.0)

        class HoldOnly(PerceptualGovernor):
            step_blind = None
        for dl in (0.06, 0.15):
          for s in (0, 1):
            for i, dx in enumerate(TEST):
                for name, c, zo in (("pg_fwd", PerceptualGovernor(J0=cal.J, L0=cal.L), True),
                                    ("pg_hold", HoldOnly(J0=cal.J, L0=cal.L), True),
                                    ("gov", make_controller("governor", delay=dl), False),
                                    ("fixed_k1.5", make_controller("fixed", delay=dl, k=1.5), False)):
                    st = StereoCameraConfig(); st.pixel_noise_px = 0.25; st.disparity_noise_px = 0.35
                    cfg = SimConfig(duration=6.0, delay=dl, noise=0.0, eps=0.004, stereo=st, zero_oracle=zo,
                                    neck=False, seed=1000 * s + i, intermittent_gaze=True)
                    m = metrics(simulate_reach(M, Q0, P0 + np.asarray(dx), c, cfg))
                    write(out, dict(ctl=name, delay=dl, tgt=i, seed=s, gaze=True, **m))
    out = "reports/v5_moving_head.jsonl"
    if not os.path.exists(out):
        cal = PerceptualGovernor()
        run(cal, np.zeros(3), 0.06, 0, True, dur=3.0)

        class Foveating(PerceptualGovernor):
            """Head turns to keep the target at the image centre (pixels only)."""
            def __init__(self, *a, k_neck=3.0, **kw):
                super().__init__(*a, **kw)
                self.k_neck = k_neck

            def step_obs(self, o):
                qd = super().step_obs(o)
                du = 0.5 * (o.target[0, 0] + o.target[1, 0]) - 320.0
                dv = 0.5 * (o.target[0, 1] + o.target[1, 1]) - 240.0
                self.last_dq_neck = (np.clip([-self.k_neck * du / 580.0, self.k_neck * dv / 580.0], -1, 1)
                                     if self.learned else np.zeros(2))
                return qd
        for dl in (0.06, 0.15):
            for i, dx in enumerate(TEST):
                for name, kn in (("pg_neck", 3.0), ("pg_neck_fast", 8.0)):
                    st = StereoCameraConfig(); st.pixel_noise_px = 0.25; st.disparity_noise_px = 0.35
                    cfg = SimConfig(duration=5.0, delay=dl, noise=0.0, eps=0.004, stereo=st, zero_oracle=True,
                                    neck=True, seed=i)
                    m = metrics(simulate_reach(M, Q0, P0 + np.asarray(dx), Foveating(J0=cal.J, L0=cal.L, k_neck=kn), cfg))
                    write(out, dict(ctl=name, delay=dl, tgt=i, **m))
    print("done")


if __name__ == "__main__":
    main()
