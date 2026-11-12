# Reaching benchmark — combined scenario report

All scenarios share the same arm, task and analysis conventions. Each table contrasts the governor with the fastest baseline in that cell; the full per-controller tables and the paired Holm-corrected statistics are in `scenario_<name>.md`.

## delay

Manipulated variable: visual delay (ms). 1260 reaches in 42 cells.

* Governor: 0 failures, overshoot in 0/6 cells.
* Holm-significant paired comparisons: 14 of 25; governor faster in 14, slower in 0.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| 0 ms | 0.65 | smith | 0.69 | -5.8 | 0.0 % | 0.0 % |
| 30 ms | 0.72 | adaptive | 0.62 | +16.0 | 0.0 % | 0.0 % |
| 60 ms | 0.64 | smith | 0.95 | -33.2 | 0.0 % | 0.0 % |
| 120 ms | 0.75 | smith | 0.98 | -23.9 | 0.0 % | 0.0 % |
| 180 ms | 0.96 | smith | 1.13 | -15.0 | 0.0 % | 0.0 % |
| 200 ms | 1.03 | smith | 1.10 | -6.8 | 0.0 % | 0.0 % |

## framerate

Manipulated variable: control rate (Hz). 480 reaches in 16 cells.

* Governor: 0 failures, overshoot in 0/4 cells.
* Holm-significant paired comparisons: 6 of 8; governor faster in 6, slower in 0.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| 100 Hz | 0.64 | smith | 0.95 | -33.2 | 0.0 % | 0.0 % |
| 15 Hz | 0.80 | smith | 2.07 | -61.3 | 0.0 % | 6.7 % |
| 30 Hz | 0.67 | smith | 1.03 | -35.5 | 0.0 % | 0.0 % |
| 60 Hz | 0.64 | smith | 0.97 | -33.6 | 0.0 % | 0.0 % |

## noise

Manipulated variable: anisotropic noise (sigma x,y,z mm). 240 reaches in 8 cells.

* Governor: 0 failures, overshoot in 0/4 cells.
* Holm-significant paired comparisons: 3 of 4; governor faster in 3, slower in 0.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| 0.0005 | 0.64 | fixed | 1.02 | -37.7 | 0.0 % | 36.7 % |
| 0.002 | 0.98 | fixed | 0.93 | +5.9 | 0.0 % | 26.7 % |
| 0.003 | 0.67 | fixed | 2.56 | -74.0 | 0.0 % | 33.3 % |
| 0.004 | 0.66 | fixed | 2.92 | -77.4 | 0.0 % | 36.7 % |

## jitter

Manipulated variable: delay jitter (ms, 1 sigma). 480 reaches in 16 cells.

* Governor: 0 failures, overshoot in 0/4 cells.
* Holm-significant paired comparisons: 5 of 5; governor faster in 5, slower in 0.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| 0 ms | 0.75 | smith | 0.98 | -23.9 | 0.0 % | 0.0 % |
| 10 ms | 0.77 | smith | 1.19 | -36.0 | 0.0 % | 10.0 % |
| 20 ms | 0.82 | smith | 1.38 | -40.9 | 0.0 % | 16.7 % |
| 40 ms | 0.84 | smith | 2.35 | -64.3 | 0.0 % | 63.3 % |

## delay_jump

Manipulated variable: delay schedule. 480 reaches in 16 cells.

* Governor: 0 failures, overshoot in 0/4 cells.
* Holm-significant paired comparisons: 7 of 9; governor faster in 7, slower in 0.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| 60 -> 0 ms at 0.8 s | 0.64 | smith | 0.69 | -8.0 | 0.0 % | 0.0 % |
| 60 -> 180 -> 60 ms | 0.87 | smith | 1.22 | -28.7 | 0.0 % | 0.0 % |
| 60 -> 180 ms at 0.8 s | 0.64 | smith | 1.19 | -46.4 | 0.0 % | 0.0 % |
| constant 60 ms | 0.64 | smith | 0.95 | -33.2 | 0.0 % | 0.0 % |

## fitts

Manipulated variable: index of difficulty (bits). 720 reaches in 24 cells.

* Governor: 0 failures, overshoot in 3/12 cells.
* Holm-significant paired comparisons: 10 of 12; governor faster in 9, slower in 1.

| condition | governor MT s | fastest baseline | baseline MT s | ΔMT % | gov. overshoot | base. overshoot |
|---|---|---|---|---|---|---|
| D=0.100 m, W=0.0080 m | 0.43 | fixed | 0.58 | -25.9 | 100.0 % | 100.0 % |
| D=0.100 m, W=0.0160 m | 0.42 | fixed | 0.39 | +7.7 | 100.0 % | 100.0 % |
| D=0.100 m, W=0.0320 m | 0.19 | fixed | 0.38 | -50.0 | 0.0 % | 100.0 % |
| D=0.150 m, W=0.0080 m | 0.35 | fixed | 0.68 | -48.5 | 6.7 % | 100.0 % |
| D=0.150 m, W=0.0160 m | 0.32 | fixed | 0.47 | -31.9 | 0.0 % | 100.0 % |
| D=0.150 m, W=0.0320 m | 0.28 | fixed | 0.46 | -39.1 | 0.0 % | 83.3 % |
| D=0.200 m, W=0.0080 m | 0.46 | fixed | 0.78 | -41.0 | 0.0 % | 100.0 % |
| D=0.200 m, W=0.0160 m | 0.42 | fixed | 0.55 | -23.6 | 0.0 % | 100.0 % |
| D=0.200 m, W=0.0320 m | 0.38 | fixed | 0.37 | +2.7 | 0.0 % | 0.0 % |
| D=0.250 m, W=0.0080 m | 0.58 | fixed | 0.86 | -32.6 | 0.0 % | 90.0 % |
| D=0.250 m, W=0.0160 m | 0.52 | fixed | 0.64 | -18.8 | 0.0 % | 73.3 % |
| D=0.250 m, W=0.0320 m | 0.48 | fixed | 0.47 | +2.1 | 0.0 % | 0.0 % |

## Conventions

* MT: settling time (s); median with percentile bootstrap 95 % CI.
* Overshoot rate: fraction of reaches whose penetration past the
  target exceeds the arrival tolerance `eps`; Wilson 95 % CI.
* Failures: reaches that never settled inside the window.
* Paired comparisons use identical reach tasks across controllers;
  two-sided permutation test on log MT, Cohen's d for effect size,
  Holm correction across all comparisons within a scenario.
* All controllers share the same delayed, noisy measurement and the
  same plant; only the control law differs.
* Simulation only; the arm and the motors are a design study.
