# Scenario: framerate

Manipulated variable: control rate (Hz). Cells contain the reaches of all controllers under identical conditions.

| rate Hz | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |
|---|---|---|---|---|---|---|---|---|---|
| 100 | fixed | 30 | 1.02 | [1.00, 1.04] | 0 | 36.7 % | [21.9 %, 54.5 %] | 6.78 | 160.9 |
| 100 | governor | 30 | 0.64 | [0.60, 0.86] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 159.7 |
| 100 | lqr | 30 | 2.54 | [2.49, 2.97] | 27 | 90.0 % | [74.4 %, 96.5 %] | 28.66 | 172.3 |
| 100 | smith | 30 | 0.95 | [0.65, 1.05] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.28 | 162.2 |
| 15 | fixed | 30 | n/a | [n/a, n/a] | 30 | 96.7 % | [83.3 %, 99.4 %] | 26.71 | 196.4 |
| 15 | governor | 30 | 0.80 | [0.73, 0.83] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.82 | 158.0 |
| 15 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 29.44 | 235.3 |
| 15 | smith | 30 | 2.07 | [1.60, 2.80] | 13 | 6.7 % | [1.8 %, 21.3 %] | 5.74 | 258.3 |
| 30 | fixed | 30 | 1.15 | [1.12, 1.23] | 0 | 46.7 % | [30.2 %, 63.9 %] | 8.74 | 160.8 |
| 30 | governor | 30 | 0.67 | [0.65, 0.72] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.01 | 158.6 |
| 30 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 29.62 | 206.2 |
| 30 | smith | 30 | 1.03 | [0.97, 1.07] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.72 | 160.5 |
| 60 | fixed | 30 | 1.03 | [1.03, 1.05] | 0 | 40.0 % | [24.6 %, 57.7 %] | 7.56 | 160.5 |
| 60 | governor | 30 | 0.64 | [0.62, 0.88] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.59 | 159.1 |
| 60 | lqr | 30 | 1.60 | [1.60, 1.60] | 29 | 96.7 % | [83.3 %, 99.4 %] | 29.22 | 180.1 |
| 60 | smith | 30 | 0.97 | [0.93, 1.02] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.17 | 161.0 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| 15 Hz | smith | -61.3 | -3.19 | 0.0002 | 0.0016 |
| 30 Hz | fixed | -42.0 | -3.31 | 0.0002 | 0.0016 |
| 30 Hz | smith | -35.5 | -2.03 | 0.0002 | 0.0016 |
| 60 Hz | fixed | -37.9 | -1.19 | 0.0002 | 0.0016 |
| 60 Hz | smith | -33.6 | -0.57 | 0.0280 | 0.0560 |
| 100 Hz | fixed | -37.7 | -1.32 | 0.0002 | 0.0016 |
| 100 Hz | lqr | -75.0 | -4.21 | 0.0004 | 0.0016 |
| 100 Hz | smith | -33.2 | -0.35 | 0.1832 | 0.1832 |

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
