"""v6 encoderless experiment (paper Section 5.10).

No joint servo, no encoder, no gravity compensation: the drive gets only a speed
command; per-joint holding brakes clamp a joint whose command is zero.

Usage (from the robosimu folder):  python scripts/run_v6_encoderless.py
Output: reports/v6_encoderless.jsonl
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_v5 import M, P0, Q0, TEST, metrics  # noqa: E402
from robosimu.perceptual_governor import PerceptualGovernor  # noqa: E402
from robosimu.pulse_look import PulseLook  # noqa: E402
from robosimu.simulator import SimConfig, simulate_reach  # noqa: E402
from robosimu.stereo import StereoCameraConfig  # noqa: E402

OUT = "reports/v6_encoderless.jsonl"
BEST = dict(frac=0.8, look=0.2, lb=0.7)      # chosen on the TUNE targets only


def run(ctl, dx, dl, seed, damp, dur=6.0):
    st = StereoCameraConfig()
    st.pixel_noise_px = 0.25
    st.disparity_noise_px = 0.35
    cfg = SimConfig(duration=dur, delay=dl, noise=0.0, eps=0.004, stereo=st, zero_oracle=True,
                    neck=False, seed=seed, encoderless=True, drive_damping=damp)
    return simulate_reach(M, Q0, P0 + np.asarray(dx), ctl, cfg)


def main():
    os.makedirs("reports", exist_ok=True)
    if os.path.exists(OUT):
        os.remove(OUT)
    for damp in (150.0, 40.0):
        cal = PerceptualGovernor()
        run(cal, np.zeros(3), 0.06, 0, damp, dur=4.0)
        J = cal.J.copy()
        for dl in (0.06, 0.15):
            for name in ("pulse", "pulse_ord", "pg_cont", "pulse_nobias"):
                for i, dx in enumerate(TEST):
                    if name == "pulse":
                        c = PulseLook(J0=J, L0=0.0, **BEST)
                    elif name == "pulse_ord":
                        c = PulseLook(J0=J, L0=0.0, ordinal=True, **BEST)
                    elif name == "pulse_nobias":
                        c = PulseLook(J0=J, L0=0.0, frac=0.8, look=0.2, lb=0.0)
                    else:
                        c = PerceptualGovernor(J0=J, L0=0.0)
                    m = metrics(run(c, dx, dl, i, damp))
                    with open(OUT, "a") as f:
                        f.write(json.dumps(dict(ctl=name, damp=damp, delay=dl, tgt=i, **m)) + "\n")
    print("done")


if __name__ == "__main__":
    main()
