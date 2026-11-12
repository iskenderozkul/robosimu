# robosimu — 6-axis arm reaching simulator

A dependency-light rigid-body simulator for a six-axis serial arm, built to test
a delay-derived visual-reaching controller against conventional baselines under
academically reportable conditions.

For any user-supplied model it computes:

* forward and inverse kinematics (DH parameters);
* point-wise position, velocity and acceleration on any link;
* inverse dynamics (link-Jacobian Newton–Euler): joint torques, gravity load,
  Coriolis/centrifugal terms, mass matrix;
* motor-side quantities for DC, BLDC and stepper drives: gearbox torque, current,
  voltage, torque–speed limit, copper and mechanical power, energy;
* driver-level limits: current limit, torque (current) ramp, regeneration;
* closed-loop reaching with delayed, noisy visual feedback, and a library of
  controllers: fixed gain, **adaptive gain**, **Smith predictor**, **Broyden**
  interaction-matrix estimate, **PID**, **LQR**, and the **delay-derived
  governor**;
* a two-rate plant model: a 1 kHz joint servo tracking the reference produced
  by the visual loop, so the visual rate (15–100 Hz) affects the task without
  destabilising the joint loop;
* pre-defined academic scenarios (delay, control rate, anisotropic noise, delay
  jitter, mid-move delay jumps, Fitts distance/width) with bootstrap and Wilson
  confidence intervals;
* 3D visualisation (PNG, animated GIF, and a self-contained interactive HTML
  viewer with joint sliders).

Only NumPy and Pillow are required. No ROS, no PyBullet, no SciPy.

## Status

This is a **simulation** tool. It is not a hardware validation and the motor
models are datasheet-level, not identified from a real rig. See `TODO.md` for
the roadmap and `reports/` for the experiments that were run.

## Install

```bash
pip install -e .          # from this directory
# or
pip install numpy Pillow
```

## Quick start

```bash
# check a model
python -m robosimu validate models/six_axis_arm.json

# one reach, governor vs fixed gain
python -m robosimu simulate models/six_axis_arm.json --controller governor --gif figures/reach.gif
python -m robosimu simulate models/six_axis_arm.json --controller fixed --figure figures/reach.png

# interactive 3D viewer (open the HTML in a browser)
python -m robosimu html models/six_axis_arm.json --out figures/arm_viewer.html

# all academic scenarios, 5 seeds x 6 reaches per cell, 8 parallel workers
python -m robosimu scenarios models/six_axis_arm.json --out reports --seeds 5 --reaches 6 --jobs 8

# a single scenario
python -m robosimu scenario delay models/six_axis_arm.json --out reports --seeds 5 --reaches 6

# retune every baseline once at the nominal condition, then rebuild the
# reports from the cached CSVs without re-simulating
python scripts/tune_baselines.py --jobs 8
python scripts/rebuild_reports.py --out reports

# drive types, gravity loads, point motion
python -m robosimu motor-compare models/six_axis_arm.json --out reports
python -m robosimu gravity-map models/six_axis_arm.json --out reports
python -m robosimu point-motion models/six_axis_arm.json --out reports
```

## Scenarios

| scenario | manipulated variable | controllers |
|---|---|---|
| `delay` | visual delay 0–200 ms | fixed, adaptive, smith, broyden, pid, lqr, governor |
| `framerate` | control rate 15/30/60/100 Hz | fixed, governor, smith, lqr |
| `noise` | anisotropic pixel/depth noise | fixed, governor |
| `jitter` | delay jitter (1σ) | fixed, governor, smith, lqr |
| `delay_jump` | mid-move delay changes | fixed, governor, smith, lqr |
| `fitts` | target distance × width → ID, throughput (60 ms stress, frozen gains) | fixed, governor |

Each scenario writes `reports/scenario_<name>.csv` (one row per reach),
`scenario_<name>.md` (tables and paired statistics), `..._summary.json`,
`..._comparisons.json` and `figures/scenario_<name>.png`; running all of them
also writes `reports/scenarios_combined.md`.

The reports and figures can be regenerated from the cached CSVs without
re-simulating:

```bash
python scripts/rebuild_reports.py --out reports
```

## Controllers

All controllers share one interface — they receive the Cartesian error
`e = target − p_measured` and return a Cartesian velocity command.

| name | law | role |
|---|---|---|
| `fixed` | `v = k e`, `k = 10` | proportional baseline |
| `adaptive` | `v = λ0/(1+\|e\|/e_ref) e`, `λ0 = 14`, `e_ref = 50 mm` | ViSP-style adaptive gain |
| `smith` | `x̂ = xm + [e − xm(t−δ)]`, `v = k x̂`, `k = 20` | Smith predictor with an internal model |
| `broyden` | `v = λ e / ŝ`, `λ = 6`, `ŝ` updated online | uncertain depth/calibration |
| `pid` | `v = kp e + ki ∫e + kd ė`, `kp=10, ki=0.5, kd=0.02` | classical PID |
| `lqr` | double-integrator LQR, `u = v̇`, `Q = diag(9, 0.5)`, `R = 0.01` | model-based optimal control |
| `governor` | `v = min(k_hi\|e\|, κ/δ_eff (ε+\|e\|))` | delay-derived speed limit |

The baseline gains are **tuned once** at the nominal condition (30 ms delay,
100 Hz, σ = 0.5 mm) by `scripts/tune_baselines.py`, chosen as the fastest
admissible candidate (no failure, no penetration beyond the arrival tolerance),
and then frozen for every other condition. The governor has no tuned parameter:
`κ = 1/e`, `k_hi = 40`, and `δ_eff` is computed from the measured delay and the
frame period. The full grid and the selected values are in
`reports/baseline_tuning.md`.

## Model format

A model is a JSON file with `links` (DH parameters, mass, centre of mass,
inertia, joint limits) and a per-joint `motor` block. **Every joint may have a
different motor type, gear ratio and driver settings** — the model is a design
study, not a catalogue robot.

```json
{
  "name": "my_arm",
  "gravity": [0, 0, -9.81],
  "links": [{
    "name": "shoulder",
    "a": 0.42, "alpha_deg": 0.0, "d": 0.0, "theta_offset_deg": 0.0,
    "mass": 4.0, "com": [0.21, 0, 0], "inertia": [0.08, 0.08, 0.01],
    "limits_deg": [-120, 120],
    "motor": {
      "type": "bldc",
      "gear_ratio": 120, "efficiency": 0.80,
      "kt": 0.09, "resistance": 0.50, "voltage": 48,
      "stall_torque": 2.8, "no_load_speed": 400, "continuous_torque": 1.6,
      "rotor_inertia": 0.0002,
      "viscous_friction": 0.06, "coulomb_friction": 0.18,
      "current_limit_a": 0.0,
      "torque_ramp_nm_s": 0.0,
      "brake_torque_nm": 0.0
    }
  }]
}
```

Motor fields:

| field | meaning |
|---|---|
| `type` | `dc`, `bldc` or `stepper` |
| `gear_ratio`, `efficiency` | reducer ratio and efficiency |
| `kt`, `resistance`, `voltage` | motor constant (Nm/A), winding resistance, supply |
| `stall_torque`, `no_load_speed` | torque-speed envelope (motor side) |
| `continuous_torque` | continuous rating (motor side) |
| `rotor_inertia` | rotor inertia (kg m²) |
| `viscous_friction`, `coulomb_friction` | joint friction |
| `current_limit_a` | driver current limit (0 = unlimited) |
| `torque_ramp_nm_s` | driver torque ramp at the joint (0 = unlimited) |
| `brake_torque_nm` | holding brake capacity (informational) |

## Reports

`reports/` contains, for every experiment, a raw CSV, a JSON summary and a
Markdown report. Statistics follow human-motor-control conventions: median
movement time with a percentile bootstrap 95 % CI, overshoot/success rates with
Wilson CIs, paired permutation tests on log movement time with Cohen's d, and
Holm step-down correction across a family of comparisons.

`reports/concept_validation.md` ties the scenario results back to the
delay-derived governor concept and states the limitations of the evidence.

## Plant model

The plant is integrated with a two-rate structure that mirrors a real arm:

* the **inner joint servo** runs at 1 kHz (`SimConfig.dt`): the PD torque,
  driver limits and friction are evaluated at that rate;
* the **outer visual loop** runs at `SimConfig.control_dt` (10 ms by default,
  15–100 Hz in the framerate scenario): the delayed, noisy measurement updates
  the Cartesian reference, which is held until the next frame.

This separation matters: without it, a 15 Hz outer loop would destabilise the
joint servo and the framerate scenario would measure numerical instability
rather than the effect of the visual rate.

## What it is not

* not a finite-element or contact simulator;
* not a motor identification tool;
* not a claim that the physical arm behaves exactly like this model;
* not real-time and not a ROS node.

## License

MIT. See `LICENSE`.
