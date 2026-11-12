# Scenario: delay_jump

Manipulated variable: delay schedule. Cells contain the reaches of all controllers under identical conditions.

| delay schedule | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |
|---|---|---|---|---|---|---|---|---|---|
| 60 -> 0 ms at 0.8 s | fixed | 30 | 0.84 | [0.81, 0.90] | 0 | 36.7 % | [21.9 %, 54.5 %] | 6.78 | 160.7 |
| 60 -> 0 ms at 0.8 s | governor | 30 | 0.64 | [0.60, 0.70] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 160.2 |
| 60 -> 0 ms at 0.8 s | lqr | 30 | 1.39 | [1.18, 1.48] | 0 | 13.3 % | [5.3 %, 29.7 %] | 5.34 | 159.2 |
| 60 -> 0 ms at 0.8 s | smith | 30 | 0.69 | [0.62, 0.72] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.28 | 162.0 |
| 60 -> 180 -> 60 ms | fixed | 30 | 2.09 | [1.86, 2.69] | 0 | 100.0 % | [88.6 %, 100.0 %] | 22.11 | 177.3 |
| 60 -> 180 -> 60 ms | governor | 30 | 0.87 | [0.82, 0.94] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.59 | 157.5 |
| 60 -> 180 -> 60 ms | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 33.16 | 214.5 |
| 60 -> 180 -> 60 ms | smith | 30 | 1.22 | [1.16, 1.25] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.23 | 165.2 |
| 60 -> 180 ms at 0.8 s | fixed | 30 | 1.31 | [1.31, 1.31] | 29 | 73.3 % | [55.6 %, 85.8 %] | 31.61 | 168.7 |
| 60 -> 180 ms at 0.8 s | governor | 30 | 0.64 | [0.60, 0.70] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 159.2 |
| 60 -> 180 ms at 0.8 s | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 34.87 | 209.0 |
| 60 -> 180 ms at 0.8 s | smith | 30 | 1.19 | [1.09, 1.22] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.28 | 163.3 |
| constant 60 ms | fixed | 30 | 1.02 | [1.00, 1.04] | 0 | 36.7 % | [21.9 %, 54.5 %] | 6.78 | 160.9 |
| constant 60 ms | governor | 30 | 0.64 | [0.60, 0.86] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 159.7 |
| constant 60 ms | lqr | 30 | 2.54 | [2.49, 2.97] | 27 | 90.0 % | [74.4 %, 96.5 %] | 28.66 | 172.3 |
| constant 60 ms | smith | 30 | 0.95 | [0.65, 1.05] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.28 | 162.2 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| constant 60 ms | fixed | -37.7 | -1.32 | 0.0002 | 0.0018 |
| constant 60 ms | lqr | -75.0 | -4.21 | 0.0004 | 0.0018 |
| constant 60 ms | smith | -33.2 | -0.35 | 0.1832 | 0.3663 |
| 60 -> 180 ms at 0.8 s | smith | -46.4 | -1.58 | 0.0002 | 0.0018 |
| 60 -> 0 ms at 0.8 s | fixed | -24.4 | -1.47 | 0.0002 | 0.0018 |
| 60 -> 0 ms at 0.8 s | lqr | -54.3 | -3.64 | 0.0002 | 0.0018 |
| 60 -> 0 ms at 0.8 s | smith | -8.0 | -0.17 | 0.5113 | 0.5113 |
| 60 -> 180 -> 60 ms | fixed | -58.4 | -4.38 | 0.0002 | 0.0018 |
| 60 -> 180 -> 60 ms | smith | -28.7 | -1.67 | 0.0002 | 0.0018 |

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
