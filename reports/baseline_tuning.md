# Baseline tuning protocol

Nominal condition: delay 30 ms, outer loop 100 Hz, sigma = 0.5 mm, no payload, 3 s window.
Each candidate is evaluated on the same 4 reach directions used by the scenarios.
Admissible: no failure and no penetration beyond the arrival tolerance (4 mm). Lowest median MT wins.

| controller | parameters | MT median s | failures | max penetration mm | admissible |
|---|---|---|---|---|---|
| fixed | k=10 | 0.610 | 0 | 0.13 | yes (chosen) |
| fixed | k=12 | 0.700 | 0 | 1.30 | yes |
| fixed | k=16 | 0.775 | 0 | 2.82 | yes |
| fixed | k=20 | 0.955 | 0 | 3.66 | yes |
| fixed | k=6 | 0.960 | 0 | 0.00 | yes |
| fixed | k=8 | 0.960 | 0 | 0.00 | yes |
| fixed | k=4 | 0.970 | 0 | 0.00 | yes |
| fixed | k=26 | 1.010 | 0 | 4.52 | no |
| adaptive | e_ref=0.05, lam0=14 | 0.610 | 0 | 0.55 | yes (chosen) |
| adaptive | e_ref=0.05, lam0=10 | 0.885 | 0 | 0.00 | yes |
| adaptive | e_ref=0.05, lam0=6 | 1.220 | 0 | 0.00 | yes |
| adaptive | e_ref=0.02, lam0=10 | 1.240 | 0 | 0.00 | yes |
| adaptive | e_ref=0.02, lam0=14 | 1.315 | 0 | 0.46 | yes |
| adaptive | e_ref=0.02, lam0=6 | 1.810 | 0 | 0.00 | yes |
| pid | kd=0.02, ki=0.5, kp=10 | 0.610 | 0 | 0.10 | yes (chosen) |
| pid | kd=0.06, ki=0.5, kp=10 | 0.660 | 0 | 0.00 | yes |
| pid | kd=0.02, ki=0.1, kp=10 | 0.665 | 0 | 0.00 | yes |
| pid | kd=0.06, ki=0.1, kp=14 | 0.715 | 0 | 1.63 | yes |
| pid | kd=0.02, ki=0.5, kp=14 | 0.715 | 0 | 2.04 | yes |
| pid | kd=0.06, ki=0.5, kp=14 | 0.715 | 0 | 1.69 | yes |
| pid | kd=0.02, ki=0.1, kp=14 | 0.735 | 0 | 2.00 | yes |
| pid | kd=0.06, ki=0.1, kp=18 | 0.740 | 0 | 2.89 | yes |
| pid | kd=0.06, ki=0.1, kp=10 | 0.795 | 0 | 0.00 | yes |
| pid | kd=0.02, ki=0.1, kp=6 | 0.840 | 0 | 0.00 | yes |
| pid | kd=0.06, ki=0.1, kp=6 | 0.875 | 0 | 0.00 | yes |
| pid | kd=0.02, ki=0.5, kp=6 | 0.930 | 0 | 0.00 | yes |
| pid | kd=0.06, ki=0.5, kp=6 | 0.930 | 0 | 0.00 | yes |
| pid | kd=0.06, ki=0.5, kp=18 | 0.940 | 0 | 2.93 | yes |
| pid | kd=0.02, ki=0.1, kp=18 | 1.095 | 0 | 3.06 | yes |
| pid | kd=0.02, ki=0.5, kp=18 | 1.095 | 0 | 3.08 | yes |
| smith | k=20 | 0.615 | 0 | 0.00 | yes (chosen) |
| smith | k=14 | 0.620 | 0 | 0.00 | yes |
| smith | k=6 | 0.905 | 0 | 0.00 | yes |
| smith | k=10 | 0.955 | 0 | 0.00 | yes |
| broyden | lam=6 | 0.620 | 0 | 0.00 | yes (chosen) |
| broyden | lam=10 | 0.735 | 0 | 2.24 | yes |
| broyden | lam=14 | 0.955 | 0 | 3.66 | yes |
| broyden | lam=20 | 1.120 | 0 | 4.61 | no |
| lqr | q_pos=9, q_vel=0.5 | 1.090 | 0 | 0.00 | yes (chosen) |
| lqr | q_pos=4, q_vel=0.24 | 1.365 | 0 | 0.00 | yes |
| lqr | q_pos=1, q_vel=0.05 | 1.405 | 0 | 0.00 | yes |
| lqr | q_pos=2, q_vel=0.1 | 1.465 | 0 | 0.00 | yes |
| lqr | q_pos=4, q_vel=0.1 | 1.055 | 0 | 8.99 | no |
| lqr | q_pos=9, q_vel=0.24 | 1.250 | 0 | 9.26 | no |
| governor | none | 0.705 | 0 | 1.91 | yes (chosen) |

## Selected (frozen) parameters

| controller | parameters | nominal MT median s |
|---|---|---|
| fixed | k=10 | 0.610 |
| adaptive | e_ref=0.05, lam0=14 | 0.610 |
| pid | kd=0.02, ki=0.5, kp=10 | 0.610 |
| smith | k=20 | 0.615 |
| broyden | lam=6 | 0.620 |
| lqr | q_pos=9, q_vel=0.5 | 1.090 |
| governor | none | 0.705 |
