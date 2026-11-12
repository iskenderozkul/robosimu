# Scenario: fitts

Manipulated variable: index of difficulty (bits). Cells contain the reaches of all controllers under identical conditions.

| distance m | width m | ID bits | controller | n | MT median s | MT 95% CI | failures | overshoot rate | max penetration mm | throughput bits/s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.100 | 0.0080 | 3.75 | fixed | 30 | 0.58 | [0.58, 0.59] | 0 | 100.0 % | 19.81 | 6.47 |
| 0.100 | 0.0080 | 3.75 | governor | 30 | 0.43 | [0.43, 0.43] | 0 | 100.0 % | 6.88 | 8.73 |
| 0.100 | 0.0160 | 2.86 | fixed | 30 | 0.39 | [0.39, 0.40] | 0 | 100.0 % | 20.48 | 7.33 |
| 0.100 | 0.0160 | 2.86 | governor | 30 | 0.42 | [0.42, 0.42] | 0 | 100.0 % | 10.62 | 6.80 |
| 0.100 | 0.0320 | 2.04 | fixed | 30 | 0.38 | [0.38, 0.38] | 0 | 100.0 % | 22.18 | 5.38 |
| 0.100 | 0.0320 | 2.04 | governor | 30 | 0.19 | [0.19, 0.20] | 0 | 0.0 % | 15.65 | 10.76 |
| 0.150 | 0.0080 | 4.30 | fixed | 30 | 0.68 | [0.67, 0.69] | 0 | 100.0 % | 16.12 | 6.33 |
| 0.150 | 0.0080 | 4.30 | governor | 30 | 0.35 | [0.35, 0.35] | 0 | 6.7 % | 4.48 | 12.30 |
| 0.150 | 0.0160 | 3.38 | fixed | 30 | 0.47 | [0.47, 0.48] | 0 | 100.0 % | 16.84 | 7.18 |
| 0.150 | 0.0160 | 3.38 | governor | 30 | 0.32 | [0.31, 0.32] | 0 | 0.0 % | 6.95 | 10.55 |
| 0.150 | 0.0320 | 2.51 | fixed | 30 | 0.46 | [0.45, 0.46] | 0 | 83.3 % | 17.54 | 5.45 |
| 0.150 | 0.0320 | 2.51 | governor | 30 | 0.28 | [0.28, 0.28] | 0 | 0.0 % | 11.34 | 8.96 |
| 0.200 | 0.0080 | 4.70 | fixed | 30 | 0.78 | [0.77, 0.79] | 0 | 100.0 % | 13.02 | 6.03 |
| 0.200 | 0.0080 | 4.70 | governor | 30 | 0.46 | [0.46, 0.46] | 0 | 0.0 % | 1.81 | 10.22 |
| 0.200 | 0.0160 | 3.75 | fixed | 30 | 0.55 | [0.55, 0.56] | 0 | 100.0 % | 13.41 | 6.83 |
| 0.200 | 0.0160 | 3.75 | governor | 30 | 0.42 | [0.42, 0.42] | 0 | 0.0 % | 4.03 | 8.94 |
| 0.200 | 0.0320 | 2.86 | fixed | 30 | 0.37 | [0.37, 0.37] | 0 | 0.0 % | 13.68 | 7.72 |
| 0.200 | 0.0320 | 2.86 | governor | 30 | 0.38 | [0.38, 0.38] | 0 | 0.0 % | 7.24 | 7.52 |
| 0.250 | 0.0080 | 5.01 | fixed | 30 | 0.86 | [0.84, 0.87] | 0 | 90.0 % | 11.35 | 5.83 |
| 0.250 | 0.0080 | 5.01 | governor | 30 | 0.58 | [0.58, 0.58] | 0 | 0.0 % | 0.49 | 8.64 |
| 0.250 | 0.0160 | 4.06 | fixed | 30 | 0.64 | [0.62, 0.64] | 0 | 73.3 % | 11.38 | 6.34 |
| 0.250 | 0.0160 | 4.06 | governor | 30 | 0.52 | [0.52, 0.53] | 0 | 0.0 % | 1.78 | 7.80 |
| 0.250 | 0.0320 | 3.14 | fixed | 30 | 0.47 | [0.47, 0.48] | 0 | 0.0 % | 11.62 | 6.68 |
| 0.250 | 0.0320 | 3.14 | governor | 30 | 0.48 | [0.48, 0.48] | 0 | 0.0 % | 4.96 | 6.54 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| D=0.100 m, W=0.0080 m | fixed | -25.9 | -9.27 | 0.0002 | 0.0024 |
| D=0.100 m, W=0.0160 m | fixed | +7.7 | 0.04 | 0.8444 | 1.0000 |
| D=0.100 m, W=0.0320 m | fixed | -50.0 | -6.04 | 0.0002 | 0.0024 |
| D=0.150 m, W=0.0080 m | fixed | -48.5 | -3.78 | 0.0002 | 0.0024 |
| D=0.150 m, W=0.0160 m | fixed | -31.9 | -3.98 | 0.0002 | 0.0024 |
| D=0.150 m, W=0.0320 m | fixed | -39.1 | -3.03 | 0.0002 | 0.0024 |
| D=0.200 m, W=0.0080 m | fixed | -41.0 | -15.16 | 0.0002 | 0.0024 |
| D=0.200 m, W=0.0160 m | fixed | -23.6 | -20.81 | 0.0002 | 0.0024 |
| D=0.200 m, W=0.0320 m | fixed | +2.7 | 1.75 | 0.0002 | 0.0024 |
| D=0.250 m, W=0.0080 m | fixed | -32.6 | -4.30 | 0.0002 | 0.0024 |
| D=0.250 m, W=0.0160 m | fixed | -18.8 | -2.84 | 0.0002 | 0.0024 |
| D=0.250 m, W=0.0320 m | fixed | +2.1 | 0.11 | 0.7075 | 1.0000 |

## Notes

* MT is the settling time (last control step at which the Cartesian error
  exceeds the arrival threshold); the interval is a percentile
  bootstrap over reaches of the same median.
* Failures are reaches that never settled inside the simulated window.
* Overshoot rate counts reaches whose penetration past the target exceeds
  the arrival tolerance `eps` (a real overshoot beyond the accepted band).
  Max penetration is the largest such excursion across the cell.
* Reach directions are a fixed low-discrepancy cone (paired across
  controllers); the seed changes only the measurement-noise realisation.
* Every baseline was tuned once at the nominal 30 ms condition and then
  frozen (`reports/baseline_tuning.md`); the governor has no tuned
  parameter and reads only the measured delay and frame period.
* The delay-aware laws (governor, Smith) recompute their schedule from
  the delay reported for the current frame; baselines keep their gains.
* Simulation only; motor parameters are representative, not measured.
