# TODO — robosimu

Status: work in progress. Each item is `[ ]`, `[x]` or `[~]` (partial).

## P0 — repository

- [x] clone `iskenderozkul/robosimu`
- [x] MIT licence, `pyproject.toml`, README
- [x] directory layout and this roadmap
- [x] git identity configured
- [x] `CITATION.cff`, `.zenodo.json`

## P1 — model and kinematics

- [x] six-axis DH model with masses, centres of mass and inertia
- [x] forward kinematics, joint frames, joint axes
- [x] geometric Jacobian for any point on any link (vectorised)
- [x] point velocity and acceleration (analytic bias + numerical J-dot)
- [x] JSON model loader and validator
- [x] CLI `validate` command
- [ ] model editor GUI (optional, future)

## P2 — dynamics

- [x] inverse dynamics (link-Jacobian Newton–Euler), gravity, Coriolis
- [x] mass matrix from link Jacobians (single pass, was 7× RNEA)
- [x] validation: gravity torque vs dU/dq, passive energy conservation
- [x] forward dynamics (M^-1) and integrator

## P3 — motors, drives and driver features

- [x] DC / BLDC / stepper parameter blocks, per joint
- [x] gearbox ratio and efficiency, torque–speed envelope
- [x] current, voltage, electrical and mechanical power
- [x] energy accounting, saturation flag, regeneration clamp
- [x] per-joint and total energy report
- [x] **driver current limit** (`current_limit_a`)
- [x] **driver torque/current ramp** (`torque_ramp_nm_s`)
- [x] holding-brake field (`brake_torque_nm`, informational)

## P4 — simulation and control

- [x] joint PD + gravity compensation
- [x] two-rate plant: 1 kHz inner joint servo, 15–100 Hz outer visual loop
- [x] Cartesian resolved-rate control with damped pseudo-inverse
- [x] delayed and noisy visual feedback
- [x] **delay jitter** (`delay_jitter`) and **mid-move delay jumps**
      (`delay_schedule`)
- [x] **anisotropic noise** (scalar or 3-vector)
- [x] fixed gain, adaptive gain, Smith predictor, Broyden, PID, LQR, governor
- [x] metrics: overshoot, penetration, settling, final error, peak torque, energy, failures

## P5 — visualisation

- [x] 3D software renderer (Pillow), arm and trajectory
- [x] animated GIF of an experiment
- [x] **interactive HTML viewer** with live joint sliders and trajectory scrubbing

## P6 — experiments and reports

- [x] nominal reach: governor vs fixed gain
- [x] delay sweep (30/60/120/180 ms)
- [x] payload sweep (0/2/4 kg)
- [x] **drive-type comparison** (DC vs BLDC vs stepper, nominal + fast task)
- [x] **gravity load map** (self-weight over a shoulder/elbow grid)
- [x] **point-wise velocity/acceleration report**
- [x] **academic scenarios**: delay, framerate, noise, jitter, delay jump, Fitts
- [x] **statistics**: bootstrap CI (MT median), Wilson CI (rates), permutation
      tests, Cohen's d, Holm correction, Fitts ID / effective width / throughput
- [x] **baseline tuning protocol**: `scripts/tune_baselines.py`, tuned once at
      30 ms and frozen, grid and selection in `reports/baseline_tuning.md`
- [x] **report rebuild**: `scripts/rebuild_reports.py` regenerates tables and
      figures from the cached CSVs without re-simulating
- [x] CSV + JSON + Markdown reports per experiment

## P7 — tests and docs

- [x] kinematics, dynamics, motor, simulation tests
- [x] **driver-feature tests** (current limit, torque ramp)
- [x] **sim-config tests** (jitter, schedule, anisotropic noise)
- [x] **controller tests** (all seven laws, dead zone, boundedness, LQR gains)
- [x] **trajectory tests** (trapezoid, S-curve, short move, synchronised)
- [x] **stats tests** (bootstrap, Wilson, Holm, Fitts, permutation test)
- [x] **scenario construction tests**
- [x] **HTML viewer test**

## P8 — publication

- [x] commit everything and push to GitHub (`origin/main`, 2026-11-12)
- [x] tag `v0.1.0` (release notes in the commit message and `reports/concept_validation.md`)
- [ ] Zenodo DOI (requires the owner's Zenodo account — metadata is ready)

## P9 — concept validation report (for the article)

- [x] `reports/concept_validation.md` — how the delay-derived governor concept
      was tested on the full 6-DOF arm, with the scenario results, the honest
      limitations and reproduction commands

## Performance log

| change | effect |
|---|---|
| vectorised link Jacobians | 6.35 s → 1.05 s per reach (6×) |
| mass matrix from Jacobians | included above |
| ProcessPoolExecutor (8 workers) | ~8× wall-clock on the scenario matrices |

## Known limitations

* rigid bodies, no compliance, no contact, no friction beyond a simple joint
  viscous/Coulomb term;
* motor parameters are representative datasheet values, not measured;
* the 6-axis arm geometry is a design study, not a commercial robot;
* no real-time controller, no ROS interface.

## Work log

| date | item | note |
|---|---|---|
| 2026-11-10 | audit | 19/19 tests pass; delay+payload sweeps done; HTML viewer and motor comparison were missing |
| 2026-11-11 | expansion | driver limits, jitter/schedule, anisotropic noise, 6 new controllers (7 total incl. LQR), 6 scenarios, paired Holm-corrected comparisons, stats, HTML viewer; 46 tests |
| 2026-11-12 | correctness | two-rate plant (1 kHz servo / 15–100 Hz vision) fixed the spurious 15–30 Hz failures; median bootstrap CI; baselines retuned once at 30 ms and frozen; rebuild script |
| 2026-11-12 | scenarios | per-frame delay tracking (governor + Smith) removed the delay-jump artefact; Fitts redesigned to feasible workspace (0.10–0.25 m, 8–32 mm targets, 60 ms stress); full matrix re-run: 3660 reaches, 48 tests |
