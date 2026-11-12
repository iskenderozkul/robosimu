# Scenario: noise

Manipulated variable: anisotropic noise (sigma x,y,z mm). Cells contain the reaches of all controllers under identical conditions.

| noise | controller | n | MT median s | MT 95% CI | failures | overshoot rate | overshoot 95% CI | max penetration mm | energy J |
|---|---|---|---|---|---|---|---|---|---|
| sigma xyz = 0.5, 0.5, 0.5 mm | fixed | 30 | 1.02 | [1.00, 1.04] | 0 | 36.7 % | [21.9 %, 54.5 %] | 6.78 | 160.9 |
| sigma xyz = 0.5, 0.5, 0.5 mm | governor | 30 | 0.64 | [0.60, 0.86] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.16 | 159.7 |
| sigma xyz = 1, 1, 2 mm | fixed | 30 | 0.93 | [0.82, 1.04] | 0 | 26.7 % | [14.2 %, 44.4 %] | 6.33 | 168.5 |
| sigma xyz = 1, 1, 2 mm | governor | 30 | 0.98 | [0.91, 1.03] | 0 | 0.0 % | [0.0 %, 11.4 %] | 2.73 | 166.4 |
| sigma xyz = 1.5, 1.5, 3 mm | fixed | 30 | 2.56 | [2.34, 2.62] | 1 | 33.3 % | [19.2 %, 51.2 %] | 5.37 | 185.0 |
| sigma xyz = 1.5, 1.5, 3 mm | governor | 30 | 0.67 | [0.62, 0.76] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.35 | 161.0 |
| sigma xyz = 2, 2, 4 mm | fixed | 30 | 2.92 | [2.75, 2.96] | 10 | 36.7 % | [21.9 %, 54.5 %] | 6.06 | 198.8 |
| sigma xyz = 2, 2, 4 mm | governor | 30 | 0.66 | [0.64, 0.93] | 0 | 0.0 % | [0.0 %, 11.4 %] | 1.22 | 161.0 |

## Governor vs baselines (paired reaches, Holm-corrected)

ΔMT < 0 means the governor settles faster than that baseline. `d` is Cohen's d on log movement time; p-values come from a two-sided permutation test and are Holm-corrected across all comparisons in this scenario.

| condition | baseline | ΔMT % | d | p | p (Holm) |
|---|---|---|---|---|---|
| 0.0005 | fixed | -37.7 | -1.32 | 0.0002 | 0.0008 |
| 0.002 | fixed | +5.9 | -0.17 | 0.5267 | 0.5267 |
| 0.003 | fixed | -74.0 | -4.19 | 0.0002 | 0.0008 |
| 0.004 | fixed | -77.4 | -4.55 | 0.0002 | 0.0008 |

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
