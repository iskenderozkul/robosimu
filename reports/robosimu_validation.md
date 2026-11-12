# robosimu — validation report

Six-axis rigid-body simulation, 0.15 m Cartesian reach, 100 Hz control, rigid bodies, BLDC drives with harmonic gears.

## Delay sweep (no payload)

| delay (ms) | controller | overshoot % | settling s | final error mm | energy J | peak torque Nm | saturation |
|---|---|---|---|---|---|---|---|
| 30 | fixed | 0.00 | 0.96 | 2.15 | 162.0 | 107.0 | 0 % |
| 30 | governor | 0.06 | 0.96 | 0.91 | 163.5 | 117.1 | 0 % |
| 60 | fixed | 0.30 | 1.17 | 1.01 | 162.2 | 107.0 | 0 % |
| 60 | governor | 0.00 | 0.90 | 1.84 | 161.8 | 107.0 | 0 % |
| 120 | fixed | 4.06 | 2.90 | 3.17 | 170.3 | 116.5 | 0 % |
| 120 | governor | 0.00 | 1.03 | 1.95 | 159.7 | 105.2 | 0 % |
| 180 | fixed | 13.08 | nan | 29.65 | 178.6 | 129.7 | 0 % |
| 180 | governor | 0.00 | 1.11 | 2.16 | 157.5 | 99.6 | 0 % |

## Payload sweep (120 ms delay)

| payload (kg) | controller | overshoot % | settling s | energy J | peak torque Nm |
|---|---|---|---|---|---|
| 0 | fixed | 4.06 | 2.90 | 170.3 | 116.5 |
| 0 | governor | 0.00 | 1.03 | 159.7 | 105.2 |
| 2 | fixed | 4.12 | 2.89 | 266.4 | 142.3 |
| 2 | governor | 0.00 | 1.02 | 250.1 | 121.4 |
| 4 | fixed | 4.16 | 2.88 | 387.5 | 167.3 |
| 4 | governor | 0.00 | 1.02 | 364.6 | 138.1 |

## Notes

* All numbers are simulation, not hardware measurements.
* Energy includes motor copper loss and gear loss; the arm is held against gravity.
* The controller uses delayed Cartesian feedback (30–180 ms) and a 0.5 mm noise level.
* Saturation is the fraction of control steps at which at least one joint hit its torque–speed limit.
