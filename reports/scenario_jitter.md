# Scenario: jitter

Manipulated variable: delay jitter (ms, 1 sigma). Cells contain the reaches of all controllers under identical conditions.

| jitter ms | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |
|---|---|---|---|---|---|---|---|---|---|
| 0 | fixed | 30 | 2.90 | [2.88, 2.91] | 14 | 83.3 % | [66.4 %, 92.7 %] | 19.60 | 168.9 |
| 0 | governor | 30 | 0.75 | [0.72, 0.81] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 157.5 |
| 0 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 44.02 | 213.6 |
| 0 | smith | 30 | 0.98 | [0.76, 1.22] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.99 | 162.3 |
| 10 | fixed | 30 | 2.96 | [2.96, 2.96] | 29 | 90.0 % | [74.4 %, 96.5 %] | 26.94 | 186.4 |
| 10 | governor | 30 | 0.77 | [0.72, 1.00] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 157.8 |
| 10 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 36.96 | 210.1 |
| 10 | smith | 30 | 1.19 | [1.02, 1.36] | 0 | 10.0 % | [3.5 %, 25.6 %] | 5.31 | 165.8 |
| 20 | fixed | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 31.69 | 219.2 |
| 20 | governor | 30 | 0.82 | [0.74, 1.08] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.33 | 158.3 |
| 20 | lqr | 30 | n/a | [n/a, n/a] | 30 | 96.7 % | [83.3 %, 99.4 %] | 43.91 | 208.9 |
| 20 | smith | 30 | 1.38 | [1.31, 1.62] | 0 | 16.7 % | [7.3 %, 33.6 %] | 7.09 | 172.1 |
| 40 | fixed | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 38.20 | 240.2 |
| 40 | governor | 30 | 0.84 | [0.78, 0.98] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.68 | 159.8 |
| 40 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 30.62 | 212.1 |
| 40 | smith | 30 | 2.35 | [2.05, 2.73] | 7 | 63.3 % | [45.5 %, 78.1 %] | 15.25 | 201.0 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| 0 ms | fixed | -74.1 | -9.25 | 0.0002 | 0.0010 |
| 0 ms | smith | -23.9 | -0.63 | 0.0200 | 0.0200 |
| 10 ms | smith | -36.0 | -1.01 | 0.0008 | 0.0016 |
| 20 ms | smith | -40.9 | -1.84 | 0.0002 | 0.0010 |
| 40 ms | smith | -64.3 | -3.88 | 0.0002 | 0.0010 |

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
