# Scenario: delay

Manipulated variable: visual delay (ms). Cells contain the reaches of all controllers under identical conditions.

| delay ms | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |
|---|---|---|---|---|---|---|---|---|---|
| 0 | adaptive | 30 | 0.74 | [0.64, 0.80] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 160.1 |
| 0 | broyden | 30 | 0.92 | [0.81, 0.96] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 159.2 |
| 0 | fixed | 30 | 0.89 | [0.76, 0.95] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 159.9 |
| 0 | governor | 30 | 0.65 | [0.62, 0.69] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.28 | 163.3 |
| 0 | lqr | 30 | 1.26 | [1.21, 1.28] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 156.5 |
| 0 | pid | 30 | 0.90 | [0.72, 0.92] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 159.9 |
| 0 | smith | 30 | 0.69 | [0.63, 0.76] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 161.9 |
| 30 | adaptive | 30 | 0.62 | [0.59, 0.86] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.64 | 160.0 |
| 30 | broyden | 30 | 0.96 | [0.91, 0.98] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.53 | 159.6 |
| 30 | fixed | 30 | 0.70 | [0.64, 0.96] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.66 | 160.2 |
| 30 | governor | 30 | 0.72 | [0.69, 0.89] | 0 | 0.0 % | [0.0 %, 11.4 %] | 3.48 | 161.2 |
| 30 | lqr | 30 | 1.11 | [1.07, 1.13] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.25 | 158.0 |
| 30 | pid | 30 | 0.93 | [0.64, 0.96] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.52 | 160.2 |
| 30 | smith | 30 | 0.64 | [0.60, 0.79] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.95 | 161.8 |
| 60 | adaptive | 30 | 0.96 | [0.91, 0.98] | 0 | 33.3 % | [19.2 %, 51.2 %] | 6.51 | 161.1 |
| 60 | broyden | 30 | 1.02 | [0.85, 1.05] | 0 | 13.3 % | [5.3 %, 29.7 %] | 5.34 | 160.1 |
| 60 | fixed | 30 | 1.02 | [1.00, 1.04] | 0 | 36.7 % | [21.9 %, 54.5 %] | 6.78 | 160.9 |
| 60 | governor | 30 | 0.64 | [0.60, 0.86] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 159.7 |
| 60 | lqr | 30 | 2.54 | [2.49, 2.97] | 27 | 90.0 % | [74.4 %, 96.5 %] | 28.66 | 172.3 |
| 60 | pid | 30 | 1.02 | [1.01, 1.03] | 0 | 33.3 % | [19.2 %, 51.2 %] | 6.73 | 160.9 |
| 60 | smith | 30 | 0.95 | [0.65, 1.05] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.28 | 162.2 |
| 120 | adaptive | 30 | 2.98 | [2.96, 2.98] | 27 | 83.3 % | [66.4 %, 92.7 %] | 18.04 | 169.6 |
| 120 | broyden | 30 | 2.10 | [2.06, 2.25] | 0 | 83.3 % | [66.4 %, 92.7 %] | 15.13 | 163.9 |
| 120 | fixed | 30 | 2.90 | [2.88, 2.91] | 14 | 83.3 % | [66.4 %, 92.7 %] | 19.60 | 168.9 |
| 120 | governor | 30 | 0.75 | [0.72, 0.81] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 157.5 |
| 120 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 44.02 | 213.6 |
| 120 | pid | 30 | 2.88 | [2.85, 2.91] | 13 | 83.3 % | [66.4 %, 92.7 %] | 19.56 | 169.2 |
| 120 | smith | 30 | 0.98 | [0.76, 1.22] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.99 | 162.3 |
| 180 | adaptive | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 39.64 | 178.1 |
| 180 | broyden | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 30.58 | 181.1 |
| 180 | fixed | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 49.95 | 199.0 |
| 180 | governor | 30 | 0.96 | [0.93, 1.02] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 155.7 |
| 180 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 42.37 | 221.7 |
| 180 | pid | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 50.17 | 198.2 |
| 180 | smith | 30 | 1.13 | [1.08, 1.25] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.67 | 162.0 |
| 200 | adaptive | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 46.12 | 193.6 |
| 200 | broyden | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 48.94 | 206.7 |
| 200 | fixed | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 53.54 | 216.4 |
| 200 | governor | 30 | 1.03 | [1.00, 1.12] | 0 | 0.0 % | [0.0 %, 11.4 %] | 0.00 | 155.1 |
| 200 | lqr | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 39.98 | 213.4 |
| 200 | pid | 30 | n/a | [n/a, n/a] | 30 | 100.0 % | [88.6 %, 100.0 %] | 54.20 | 216.0 |
| 200 | smith | 30 | 1.10 | [0.92, 1.21] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.32 | 162.2 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| 0 ms | adaptive | -13.4 | -0.44 | 0.0916 | 0.7327 |
| 0 ms | broyden | -29.5 | -1.29 | 0.0002 | 0.0050 |
| 0 ms | fixed | -27.5 | -1.05 | 0.0010 | 0.0130 |
| 0 ms | lqr | -48.8 | -4.36 | 0.0002 | 0.0050 |
| 0 ms | pid | -27.9 | -0.98 | 0.0010 | 0.0130 |
| 0 ms | smith | -5.8 | -0.16 | 0.5255 | 1.0000 |
| 30 ms | adaptive | +16.0 | 0.54 | 0.0410 | 0.4099 |
| 30 ms | broyden | -24.1 | -0.40 | 0.1264 | 0.8846 |
| 30 ms | fixed | +3.6 | 0.06 | 0.8132 | 1.0000 |
| 30 ms | lqr | -34.7 | -2.21 | 0.0002 | 0.0050 |
| 30 ms | pid | -21.6 | -0.17 | 0.5205 | 1.0000 |
| 30 ms | smith | +14.2 | 0.38 | 0.1468 | 0.8846 |
| 60 ms | adaptive | -33.5 | -1.04 | 0.0002 | 0.0050 |
| 60 ms | broyden | -38.0 | -1.04 | 0.0004 | 0.0064 |
| 60 ms | fixed | -37.7 | -1.32 | 0.0002 | 0.0050 |
| 60 ms | lqr | -75.0 | -4.21 | 0.0004 | 0.0064 |
| 60 ms | pid | -37.7 | -1.39 | 0.0002 | 0.0050 |
| 60 ms | smith | -33.2 | -0.35 | 0.1832 | 0.9158 |
| 120 ms | adaptive | -74.8 | -7.94 | 0.0004 | 0.0064 |
| 120 ms | broyden | -64.4 | -6.62 | 0.0002 | 0.0050 |
| 120 ms | fixed | -74.1 | -9.25 | 0.0002 | 0.0050 |
| 120 ms | pid | -74.0 | -9.24 | 0.0002 | 0.0050 |
| 120 ms | smith | -23.9 | -0.63 | 0.0200 | 0.2200 |
| 180 ms | smith | -15.0 | -0.45 | 0.0794 | 0.7145 |
| 200 ms | smith | -6.8 | 0.20 | 0.4545 | 1.0000 |

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
