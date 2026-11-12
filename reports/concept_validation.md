# Concept validation — delay-derived governor

This document ties the simulation evidence in `reports/` back to the
delay-derived governor concept and states plainly what the evidence does and
does not support.  It is a simulation study: no hardware was built or measured.

## 1. The concept under test

The governor replaces a fixed Cartesian gain with a speed limit computed from
the measured delay, the frame period and the measurement-noise level:

```
v = min(k_hi |e|,  k (ε + |e|)),     k = κ / δ_eff,   κ = 1/e
δ_eff = δ + n_max T / 2,             n = clip(ceil((2σ/ε)²), 1, floor(2δ/T))
```

`e` is the Cartesian position error, `δ` the visual delay of the current frame,
`T` the frame period, `ε` the arrival tolerance and `σ` the measurement-noise
standard deviation.  The average of the last `n` error frames is used as the
command direction.

The claim tested here is **not** that the governor is the fastest controller.
It is that a law whose only inputs are the measured delay, the frame period and
the noise level stays stable and accurate as the delay grows, while baseline
gains that were tuned once at a nominal delay are progressively destabilised.
Every baseline is therefore tuned once at 30 ms / 100 Hz / σ = 0.5 mm and then
frozen (`reports/baseline_tuning.md`).  The governor has no tuned parameter at
all.

## 2. Plant, task and statistics

**Plant.**  Six-axis serial arm, rigid bodies, link-Jacobian Newton–Euler
inverse dynamics, mass matrix, Coulomb/viscous friction and gravity.  Each
joint has its own motor (DC, BLDC or stepper), gearbox and driver: torque–speed
envelope, current limit, torque ramp, regeneration and copper loss.  The plant
is integrated with a two-rate structure: a 1 kHz inner joint servo (PD plus
feed-forward, driver limits and friction) tracks the Cartesian reference
produced by the outer visual loop, which runs at 15–100 Hz.  The outer
reference is held between frames; this mirrors a real arm where the joint
servo is faster than the visual loop.

**Task.**  A fixed low-discrepancy cone of 64 reach directions (tilt ≤ 18°,
distance scaled by 0.92–1.08) around a 0.15 m nominal reach.  Every cell
contains 5 seeds × 6 reaches = 30 paired reaches; the seed changes only the
measurement-noise realisation, so controllers see identical tasks.

**Statistics.**  Movement time (MT) is the settling time (last control step at
which the Cartesian error exceeds `ε`); the point estimate is the median and
the interval a percentile bootstrap 95 % CI over reaches of that same median.
Overshoot is the fraction of reaches whose penetration past the target exceeds
`ε`, with a Wilson 95 % CI.  Paired comparisons use a two-sided permutation
test on log MT, Cohen's d on log MT, and Holm step-down correction across all
comparisons within a scenario.  All failures are reported; a failed reach is
never dropped from the table.

**Tuning fairness.**  `scripts/tune_baselines.py` evaluates each candidate at
the nominal condition on four of the scenario's reach directions, admits only
candidates with no failure and no penetration beyond `ε`, and keeps the fastest
admissible one.  Selected values: fixed k = 10; adaptive λ0 = 14, e_ref = 50 mm;
Smith k = 20; Broyden λ = 6; PID kp = 10, ki = 0.5, kd = 0.02; LQR
Q = diag(9, 0.5), R = 0.01.  These values are never changed afterwards.

## 3. Physical-model checks

These reports validate the plant itself, not the control concept.

* **Gravity load** (`reports/gravity_load.md`): over a shoulder/elbow grid with
  0 and 4 kg payload, the worst joint utilisation is joint 2 at 67.8 % of its
  continuous rating; joints 3–5 stay below 41 %.  No joint is over-rated.
* **Drive comparison** (`reports/motor_comparison.md`): the same reach with DC,
  BLDC and stepper drives.  On the nominal task the governor uses 193.6 J (DC),
  159.7 J (BLDC) and 235.7 J (stepper) versus 206.2 / 170.3 / 250.7 J for the
  fixed baseline, with lower peak current and no torque saturation.  On a fast
  1.0 m/s task the fixed baseline saturates (the stepper for 512 control steps,
  30.7 A, 9.8 % overshoot, no settling) while the governor settles in 0.71 s on
  every drive with zero saturation.  Joint 2 dominates the energy (164 J of
  194 J for DC/governor).
* **Point-wise motion** (`reports/point_motion.md`): along a governor reach the
  link centres of mass reach at most 0.39 m/s and 12.4 m/s², angular rates
  ≤ 2.27 rad/s and angular accelerations ≤ 25.3 rad/s² — plausible for a
  0.15 m-class arm, with no unphysical spikes.
* **Validation sweep** (`reports/robosimu_validation.md`): a single nominal
  reach, two seeds, BLDC.  At 30/60/120/180 ms the governor settles in
  0.96/0.90/1.03/1.11 s with 0 % overshoot; the fixed baseline settles in
  0.96/1.17/2.90 s and fails at 180 ms.  With 0/2/4 kg payload at 120 ms the
  governor holds 1.03/1.02/1.02 s while the fixed baseline stays near 2.9 s.
  These are single-reach numbers and are not directly comparable to the
  30-reach medians in the scenarios.

## 4. Scenario results

Headline view; full tables, confidence intervals and per-comparison statistics
are in `scenario_<name>.md` and `scenarios_combined.md`.

| scenario | governor MT (s) | governor failures | governor overshoot cells | significant comparisons |
|---|---|---|---|---|
| delay (0–200 ms) | 0.65 → 1.03 | 0 / 180 | 0 / 6 | 14 of 25, all in governor's favour |
| framerate (15–100 Hz) | 0.80 → 0.64 | 0 / 120 | 0 / 4 | 6 of 8, all in governor's favour |
| noise (0.5–4 mm) | 0.64 → 0.66 | 0 / 120 | 0 / 4 | 3 of 4, all in governor's favour |
| jitter (0–40 ms) | 0.75 → 0.84 | 0 / 120 | 0 / 4 | 5 of 5, all in governor's favour |
| delay jumps | 0.64 → 0.87 | 0 / 120 | 0 / 4 | 7 of 9, all in governor's favour |
| Fitts (60 ms) | 0.19 → 0.58 | 0 / 360 | 3 / 12 | 10 of 12, 9 in governor's favour, 1 against |

### Delay

From 0 to 200 ms the governor's median MT rises only from 0.65 s to 1.03 s with
no failures and no overshoot in any cell (maximum penetration 3.48 mm at 30 ms,
below the 4 mm tolerance).  The fixed and PID baselines show 36.7 % overshoot at
60 ms and fail at 180–200 ms; the LQR fails from 60 ms; the adaptive and Broyden
baselines fail at 180–200 ms.  At 30 ms — the tuning condition — the adaptive
baseline is actually faster than the governor (0.62 s vs 0.72 s, +16.0 %, but
not significant after Holm correction, p = 0.41).  The Smith predictor, which is
given the exact delay, remains stable everywhere and is statistically
indistinguishable from the governor at 120 ms (0.98 vs 0.75 s, p_holm = 0.22)
and at 200 ms (1.10 vs 1.03 s, p_holm = 1.00).

### Framerate

At 15 Hz the governor still settles in 0.80 s with no failures; the fixed
baseline fails 30/30, the LQR fails 30/30 and Smith fails 13/30 (median 2.07 s).
At 30–100 Hz the governor is 0.64–0.67 s with no overshoot, while the fixed
baseline overshoots in 36.7–46.7 % of reaches.  This scenario depends on the
two-rate plant: with a single-rate plant every controller failed at 15 Hz
because the joint servo itself went unstable, which is a numerical artefact, not
a control result.

### Noise

With anisotropic noise up to σ = 2 mm per axis the governor stays at 0.66–0.98 s
with no overshoot.  The fixed baseline is faster at σ = 1 mm (0.93 vs 0.98 s,
+5.9 %, not significant) but degrades sharply at σ = 1.5–2 mm (2.56–2.92 s,
33–37 % overshoot, 10 failures at 2 mm).  The governor's extra averaging budget
`n` grows with σ, which is exactly the mechanism the theory predicts.

### Delay jitter

At a 120 ms mean delay with Gaussian jitter up to σ = 40 ms the governor stays
at 0.75–0.84 s with zero failures and zero overshoot.  The fixed baseline fails
14/30 at zero jitter and 29–30/30 at 10–40 ms; Smith degrades from 0.98 s to
2.35 s and fails 7/30 at 40 ms.  Both delay-aware laws receive the delay of the
current frame, so this tests tracking a fluctuating delay rather than a frozen
one.

### Delay jumps

Mid-move jumps of 60 → 180 ms, 60 → 0 ms and 60 → 180 → 60 ms are absorbed with
zero failures and zero overshoot; the worst cell is 0.87 s for the double jump,
against 2.09 s and 100 % overshoot for the fixed baseline.  The 60 → 180 ms jump
causes 29/30 fixed failures.  This is the strongest evidence that the governor's
per-frame recomputation of `δ_eff` — not a frozen gain — is doing the work.

### Fitts

The Fitts task is run at the 60 ms delay condition with the frozen 30 ms gains,
on targets along world −y (the direction in which this arm has usable workspace
out to 0.25 m), with amplitudes 0.10–0.25 m and widths 8–32 mm.  The governor
has zero failures and overshoot in only 3 of 12 cells, all at the 0.10 m
amplitude where both controllers overshoot the small targets; the fixed baseline
overshoots in 10 of 12 cells.  The governor is significantly faster in 9 of 12
cells and significantly slower in 1 (D = 0.20 m, W = 32 mm, +2.7 %).  Throughput
is 6.5–10.8 bits/s for the governor against 5.4–7.7 bits/s for the fixed
baseline.

## 5. What the evidence supports

1. **Delay robustness without retuning.**  Across 0–200 ms delay, 15–100 Hz
   frame rate, 0.5–4 mm noise, 40 ms jitter and mid-move delay jumps, the
   governor has zero failures and zero overshoot in every cell (except three
   short-amplitude Fitts cells), while frozen baselines fail or overshoot in
   nearly every stress condition.
2. **The mechanism is the delay-derived gain, not a tuned gain.**  The governor
   has no tuned parameter; its MT varies smoothly with delay (0.65 → 1.03 s)
   rather than collapsing.  The Smith predictor, given the same exact delay,
   matches it at high delay, which confirms that knowledge of the delay is the
   operative ingredient.
3. **Energy is not sacrificed.**  On the drive comparison the governor uses
   6–7 % less energy than the fixed baseline on the nominal task and avoids the
   saturation that the fixed baseline hits on fast tasks.

## 6. What the evidence does not support

* **"Fastest controller".**  At the 30 ms tuning condition the adaptive baseline
  is faster than the governor (0.62 vs 0.72 s).  The claim is stability and
  accuracy without retuning, not raw speed.
* **Uniqueness.**  The Smith predictor is statistically indistinguishable from
  the governor at 120–200 ms constant delay when it is given the exact delay.
  It loses its advantage under jitter and delay jumps, but it remains a strong
  delay-compensation baseline.  The governor's practical advantage is that it
  needs no internal model and no retuning when the delay changes.
* **Short-amplitude accuracy.**  On 0.10 m reaches with 8–16 mm targets the
  governor overshoots in 100 % of reaches (6.9–10.6 mm penetration at 60 ms),
  and it is slower than the fixed baseline in three wide-target cells.  The
  governor's `ε + |e|` speed law is designed for the nominal 0.15 m reach and
  does not reduce speed early enough on very short, precise reaches.
* **Hardware validity.**  All results are simulation.  Motors are representative
  datasheet values, the arm is rigid, friction is a simple Coulomb/viscous model,
  and noise is Gaussian and independent.  No structural compliance, backlash,
  temperature drift, quantisation or actuator delay is modelled.

## 7. Reproduction

```bash
# physical-model reports
python -m robosimu motor-compare models/six_axis_arm.json --out reports
python -m robosimu gravity-map   models/six_axis_arm.json --out reports
python -m robosimu point-motion  models/six_axis_arm.json --out reports
python -m robosimu experiment    models/six_axis_arm.json --out reports

# full scenario matrix (5 seeds x 6 reaches per cell)
python -m robosimu scenarios models/six_axis_arm.json --out reports --seeds 5 --reaches 6 --jobs 8

# retune baselines once at the nominal condition; rebuild reports from cached CSVs
python scripts/tune_baselines.py --jobs 8
python scripts/rebuild_reports.py --out reports
```

The full matrix takes roughly 30–40 minutes on 8 workers.  Every scenario CSV,
JSON summary and Markdown report is committed under `reports/`; the figures are
under `figures/`.
