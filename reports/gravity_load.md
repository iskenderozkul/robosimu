# Gravity load map — arm self-weight and payload

Maximum absolute joint torque over a shoulder/elbow angle grid,
compared with each joint's continuous torque rating.

| payload kg | joint | max abs tau Nm | at shoulder deg | at elbow deg | continuous Nm | utilisation % |
|---|---|---|---|---|---|---|
| 0 | 1 | 0.00 | -120.0 | -150.0 | 112.0 | 0.0 |
| 0 | 2 | 76.02 | 0.0 | 0.0 | 153.6 | 49.5 |
| 0 | 3 | 26.26 | 36.0 | -30.0 | 93.6 | 28.1 |
| 0 | 4 | 2.40 | 0.0 | 105.0 | 22.5 | 10.7 |
| 0 | 5 | 0.99 | 0.0 | -90.0 | 22.5 | 4.4 |
| 0 | 6 | 0.00 | 12.0 | 30.0 | 7.2 | 0.0 |
| 4 | 1 | 0.00 | -120.0 | -150.0 | 112.0 | 0.0 |
| 4 | 2 | 104.08 | 0.0 | 15.0 | 153.6 | 67.8 |
| 4 | 3 | 37.92 | -96.0 | -75.0 | 93.6 | 40.5 |
| 4 | 4 | 7.17 | -120.0 | -120.0 | 22.5 | 31.9 |
| 4 | 5 | 5.20 | -120.0 | -150.0 | 22.5 | 23.1 |
| 4 | 6 | 0.00 | 36.0 | 15.0 | 7.2 | 0.0 |

## Notes

* Joint 1 (base yaw) has a vertical axis and carries essentially no gravity torque.
* The shoulder and elbow joints carry the arm's own weight; utilisation stays below
  the continuous rating over the whole grid, so the arm can hold any pose statically.
* Payload is a point mass at the tool; it adds to the shoulder/elbow load.
* Rigid-body model, no cable or harness weight included.
