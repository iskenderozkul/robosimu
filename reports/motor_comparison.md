# Drive-type comparison (DC vs BLDC vs stepper)

Same arm, same reach (0.15 m, 120 ms visual delay). Two tasks: the
nominal reach (0.5 m/s Cartesian limit) and a fast reach (1.0 m/s,
3 rad/s joint limit) that exposes the stepper's speed weakness.

## Nominal task

| drive | controller | energy J | copper J | peak current A | sat. steps | overshoot % | settling s |
|---|---|---|---|---|---|---|---|
| dc | fixed | 206.2 | 193.3 | 14.98 | 0 | 4.06 | 2.90 |
| dc | governor | 193.6 | 185.5 | 13.53 | 0 | 0.00 | 1.03 |
| bldc | fixed | 170.3 | 158.3 | 13.48 | 0 | 4.06 | 2.90 |
| bldc | governor | 159.7 | 151.9 | 12.18 | 0 | 0.00 | 1.03 |
| stepper | fixed | 250.7 | 237.2 | 16.59 | 0 | 4.06 | 2.90 |
| stepper | governor | 235.7 | 227.6 | 14.99 | 0 | 0.00 | 1.03 |

## Fast task

| drive | controller | energy J | copper J | peak current A | sat. steps | overshoot % | settling s |
|---|---|---|---|---|---|---|---|
| dc | fixed | 321.7 | 261.0 | 23.40 | 2 | 11.68 | n/a |
| dc | governor | 198.2 | 186.7 | 15.40 | 0 | 0.00 | 0.71 |
| bldc | fixed | 271.4 | 214.8 | 21.02 | 2 | 11.69 | n/a |
| bldc | governor | 163.7 | 153.0 | 13.86 | 0 | 0.00 | 0.71 |
| stepper | fixed | 437.6 | 218.1 | 30.72 | 512 | 9.83 | n/a |
| stepper | governor | 241.2 | 229.1 | 17.06 | 0 | 0.00 | 0.71 |

## Per-joint energy, governor, nominal task (mean over seeds)

| joint | DC J | BLDC J | stepper J |
|---|---|---|---|
| 1 | 0.6 | 0.5 | 0.7 |
| 2 | 164.1 | 134.2 | 199.6 |
| 3 | 22.4 | 19.0 | 27.7 |
| 4 | 4.0 | 3.8 | 4.7 |
| 5 | 1.4 | 1.4 | 1.6 |
| 6 | 0.0 | 0.0 | 0.0 |

## Notes

* All three drives share stall torque, kt, R and V; only the drive type changes.
* DC and BLDC use a linear torque-speed line; efficiencies are 0.72 (brushed) and
  0.80 (brushless).
* The stepper keeps the same stall torque but its no-load speed is 40% of the
  BLDC's (typical for the same frame size) and its pull-out torque falls as
  1/(1+(w/wc)^2), so it saturates earlier at speed; its efficiency is 0.65.
* Saturation steps count control steps at which a joint hit its torque-speed limit.
* Simulation only: motor parameters are representative datasheet values, not measured.
