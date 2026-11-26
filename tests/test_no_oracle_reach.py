"""End-to-end oracle-free reach test (v5 acceptance gate).

The Perceptual 1/e Governor sees only pixels + timestamps.  At 60 ms delay it
must settle within 5 mm on >= 80 % of independent targets with <= 5 % overshoot.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from run_v5 import TEST, calibrate, metrics, run  # noqa: E402
from robosimu.perceptual_governor import PerceptualGovernor  # noqa: E402
from robosimu.v43 import TimestampedObservation  # noqa: E402


def test_packet_has_no_oracle_fields():
    names = set(TimestampedObservation.__dataclass_fields__)
    assert names == {"t_cap", "t_arr", "t_now", "hand", "target"}
    assert not hasattr(PerceptualGovernor(), "set_delay")


def test_oracle_free_reach_gate():
    J0, L0 = calibrate()
    ok = 0
    os_max = 0.0
    for i, dx in enumerate(TEST[:6]):
        m = metrics(run(PerceptualGovernor(J0=J0, L0=L0), dx, 0.06, 5.0, i, True))
        ok += int(m["settled"] and m["final_mm"] <= 5.0)
        os_max = max(os_max, m["os_pct"])
    assert ok / 6 >= 0.8
    assert os_max <= 5.0
