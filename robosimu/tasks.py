"""Disjoint target sets for the A3 fair-simulation programme.

The A3 core is the dimensionless delayed reaching plant (``robosimu.a3``):
a scalar gap ``e`` driven to zero under measurement delay.  Targets are
therefore initial amplitudes ``e0``.  Three *disjoint* sets are defined so
that no amplitude used for tuning (``TUNE``) is ever used for testing
(``TEST``), and the out-of-distribution set (``OOD``) lies strictly outside
the tune/test envelope (larger amplitudes), per ``docs/dogru_yol_Faz0.md``.
"""
from __future__ import annotations

import json
from dataclasses import dataclass


@dataclass(frozen=True)
class TargetSet:
    """A named set of dimensionless initial gaps with evaluation seeds."""

    name: str
    amplitudes: tuple[float, ...]
    seeds: tuple[int, ...] = (0, 1, 2)

    def dump(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"name": self.name,
                       "amplitudes": list(self.amplitudes),
                       "seeds": list(self.seeds)}, fh, indent=2)

    def __len__(self) -> int:
        return len(self.amplitudes)


# --- pre-registered, disjoint amplitude grids (do not edit after G0) -------
TUNE_AMPLITUDES: tuple[float, ...] = (0.10, 0.15, 0.20, 0.25, 0.30, 0.35)
TEST_AMPLITUDES: tuple[float, ...] = (0.12, 0.18, 0.22, 0.28, 0.32, 0.38, 0.42, 0.45)
OOD_AMPLITUDES: tuple[float, ...] = (0.50, 0.60, 0.75, 0.90)


def tune_targets(seeds: tuple[int, ...] = (0, 1, 2)) -> TargetSet:
    return TargetSet("tune", TUNE_AMPLITUDES, tuple(seeds))


def test_targets(seeds: tuple[int, ...] = (0, 1, 2, 3, 4)) -> TargetSet:
    return TargetSet("test", TEST_AMPLITUDES, tuple(seeds))


def ood_targets(seeds: tuple[int, ...] = (0, 1, 2, 3, 4)) -> TargetSet:
    return TargetSet("ood", OOD_AMPLITUDES, tuple(seeds))


def all_sets() -> tuple[TargetSet, TargetSet, TargetSet]:
    return tune_targets(), test_targets(), ood_targets()


def assert_disjoint() -> None:
    """Fail loudly if any two pre-registered sets share an amplitude."""
    sets = all_sets()
    for i in range(len(sets)):
        for j in range(i + 1, len(sets)):
            overlap = set(sets[i].amplitudes) & set(sets[j].amplitudes)
            if overlap:
                raise ValueError(
                    f"target sets {sets[i].name!r} and {sets[j].name!r} "
                    f"overlap at amplitudes {sorted(overlap)}")


def ood_is_outside_envelope() -> bool:
    """True iff every OOD amplitude exceeds every tune/test amplitude."""
    env = max(max(TEST_AMPLITUDES), max(TUNE_AMPLITUDES))
    return min(OOD_AMPLITUDES) > env


if __name__ == "__main__":
    assert_disjoint()
    for s in all_sets():
        print(f"{s.name:5s} n={len(s)} amplitudes={s.amplitudes} seeds={s.seeds}")
    print("ood outside envelope:", ood_is_outside_envelope())
