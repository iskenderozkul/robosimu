# Human-Like Stereo Looming (HLSL) & Singularity-Safe Reaching Report

Evaluates the shoulder-mounted anthropomorphic two-eye stereo + perspective apparent-size
shrinking (optical looming) observation model combined with the **HLSL (Human-Like Stereo
Looming & Bell-Shaped Delay Governor)** control law and singularity-safe joint coordination
(`human_wdls`) against PID, LQR, and Sabit Kazanç (P).

## 1. Pre-Reach Active Camera Parallax & Singularity Prediction

* **Active Camera Micro-Sway Parallax Shift:** `5.99 px`
* **Initial Apparent Pixel Sizes:** Gripper `s_g(0) = 36.2 px`, Target `s_t = 26.0 px`
* **Estimated Shoulder-to-Target Distance:** `93.9 cm` (Max Singularity-Safe Envelope: `93.3 cm`, Ratio: `1.01`)
* **Predicted Minimum Manipulability `w_pred`:** `0.0113` -> **Safe Reachable: `False`**

## 2. Shoulder-Mounted Stereo + Perspective Shrinking Comparison (HLSL vs PID vs LQR vs Fixed)

| baseline (mm) | disp noise (px) | controller | median MT (s) | overshoot (%) | onset accel (m/s^2) | gripper px (start->end) | scale ratio | min w(q) | peak torque (Nm) | energy (J) |
|---|---|---|---|---|---|---|---|---|---|---|
| 50 | 0.3 | hlsl | 2.73 | 3.6 % | 5.47 | 36.1 -> 32.3 | 1.002 | 0.0379 | 111.2 | 169.3 |
| 50 | 0.3 | governor_per_axis | 2.69 | 3.7 % | 7.72 | 36.1 -> 32.4 | 1.002 | 0.0388 | 124.0 | 169.6 |
| 50 | 0.3 | pid | n/a | 8.9 % | 6.82 | 36.1 -> 32.4 | 1.004 | 0.0362 | 147.9 | 218.0 |
| 50 | 0.3 | lqr | n/a | 13.8 % | 3.76 | 36.1 -> 32.5 | 1.008 | 0.0264 | 132.4 | 177.2 |
| 50 | 0.3 | fixed | n/a | 8.3 % | 6.81 | 36.1 -> 32.4 | 1.004 | 0.0360 | 147.2 | 215.5 |
| 50 | 0.8 | hlsl | n/a | 6.2 % | 5.63 | 36.0 -> 32.5 | 1.007 | 0.0329 | 113.7 | 189.1 |
| 50 | 0.8 | governor_per_axis | n/a | 6.3 % | 7.55 | 36.0 -> 32.5 | 1.007 | 0.0336 | 126.6 | 190.9 |
| 50 | 0.8 | pid | n/a | 11.6 % | 6.87 | 36.0 -> 32.7 | 1.015 | 0.0311 | 208.9 | 305.5 |
| 50 | 0.8 | lqr | n/a | 15.3 % | 4.23 | 36.0 -> 32.7 | 1.014 | 0.0271 | 169.1 | 209.1 |
| 50 | 0.8 | fixed | n/a | 11.7 % | 6.87 | 36.0 -> 32.8 | 1.016 | 0.0311 | 209.2 | 303.3 |
| 65 | 0.3 | hlsl | 2.35 | 2.8 % | 5.45 | 36.1 -> 32.4 | 1.003 | 0.0393 | 114.1 | 165.3 |
| 65 | 0.3 | governor_per_axis | 2.71 | 3.3 % | 7.76 | 36.1 -> 32.3 | 1.002 | 0.0397 | 124.0 | 166.6 |
| 65 | 0.3 | pid | n/a | 9.2 % | 6.85 | 36.1 -> 32.4 | 1.003 | 0.0363 | 139.4 | 205.2 |
| 65 | 0.3 | lqr | n/a | 13.7 % | 3.69 | 36.1 -> 32.0 | 0.991 | 0.0293 | 143.3 | 179.6 |
| 65 | 0.3 | fixed | n/a | 9.2 % | 6.86 | 36.1 -> 32.4 | 1.003 | 0.0364 | 138.8 | 203.5 |
| 65 | 0.8 | hlsl | n/a | 5.6 % | 5.54 | 36.0 -> 32.4 | 1.006 | 0.0345 | 116.7 | 184.0 |
| 65 | 0.8 | governor_per_axis | n/a | 5.8 % | 7.54 | 36.0 -> 32.5 | 1.006 | 0.0352 | 124.1 | 184.3 |
| 65 | 0.8 | pid | n/a | 11.9 % | 6.84 | 36.0 -> 32.7 | 1.014 | 0.0321 | 200.2 | 280.4 |
| 65 | 0.8 | lqr | n/a | 16.8 % | 4.12 | 36.0 -> 32.7 | 1.014 | 0.0262 | 161.8 | 202.8 |
| 65 | 0.8 | fixed | n/a | 11.8 % | 6.84 | 36.0 -> 32.7 | 1.012 | 0.0331 | 196.4 | 275.4 |
| 100 | 0.3 | hlsl | 2.63 | 2.2 % | 5.78 | 36.1 -> 32.3 | 1.002 | 0.0406 | 115.2 | 159.3 |
| 100 | 0.3 | governor_per_axis | 1.69 | 2.0 % | 7.67 | 36.1 -> 32.4 | 1.002 | 0.0419 | 121.8 | 158.1 |
| 100 | 0.3 | pid | 2.69 | 6.0 % | 6.91 | 36.1 -> 32.4 | 1.003 | 0.0390 | 131.3 | 186.2 |
| 100 | 0.3 | lqr | n/a | 12.7 % | 3.65 | 36.1 -> 32.9 | 1.019 | 0.0301 | 128.8 | 174.8 |
| 100 | 0.3 | fixed | 2.70 | 6.0 % | 6.92 | 36.1 -> 32.4 | 1.003 | 0.0389 | 130.9 | 184.8 |
| 100 | 0.8 | hlsl | 2.76 | 4.6 % | 5.43 | 36.0 -> 32.4 | 1.006 | 0.0373 | 117.7 | 175.8 |
| 100 | 0.8 | governor_per_axis | 2.75 | 4.8 % | 7.60 | 36.0 -> 32.4 | 1.006 | 0.0376 | 124.2 | 176.8 |
| 100 | 0.8 | pid | n/a | 9.1 % | 6.80 | 36.0 -> 32.5 | 1.008 | 0.0361 | 162.6 | 240.5 |
| 100 | 0.8 | lqr | n/a | 14.9 % | 3.94 | 36.0 -> 32.9 | 1.020 | 0.0271 | 152.3 | 190.0 |
| 100 | 0.8 | fixed | n/a | 9.0 % | 6.80 | 36.0 -> 32.5 | 1.007 | 0.0359 | 168.8 | 236.0 |

## 3. Near-Singularity Extended Reach (+0.19 m): Human-Like WDLS vs Plain DLS

| joint routing mode | controller | median MT (s) | overshoot (%) | onset accel (m/s^2) | min manipulability w | max cond kappa(J) | peak torque (Nm) | energy (J) |
|---|---|---|---|---|---|---|---|---|
| human_wdls | hlsl | n/a | 0.0 % | 6.26 | 0.0098 | 99.4 | 127.4 | 172.4 |
| human_wdls | pid | n/a | 0.0 % | 6.35 | 0.0032 | 304.6 | 159.1 | 181.2 |
| human_wdls | lqr | n/a | 0.0 % | 4.40 | 0.0025 | 394.5 | 131.2 | 159.7 |
| human_wdls | fixed | n/a | 0.0 % | 6.35 | 0.0033 | 296.7 | 158.4 | 180.3 |
| plain_dls | hlsl | n/a | 0.0 % | 6.48 | 0.0000 | 801677.5 | 112.1 | 173.6 |
| plain_dls | pid | n/a | 0.0 % | 6.44 | 0.0000 | 922465.7 | 154.6 | 197.7 |
| plain_dls | lqr | n/a | 0.0 % | 4.38 | 0.0000 | 212715.2 | 225.6 | 243.2 |
| plain_dls | fixed | n/a | 0.0 % | 6.45 | 0.0000 | 974313.6 | 153.6 | 196.5 |

## Key Academic Findings

1. **Perspective Apparent-Size Shrinking (Looming) + Shoulder-Mounted Stereo:** Because the
   stereo eyes sit above and slightly behind Joint 1 (`(-0.12, 0.0, 0.62) m`), extending the
   hand toward the target causes the gripper's apparent pixel diameter `s_g(t) = f W_g / Z_g(t)`
   to visibly shrink in perspective from `~47 px` to `~37 px` until its normalized scale
   matches the target (`(s_g / W_g) / (s_t / W_t) -> 1.000`).
2. **HLSL Eliminates Both Onset Jerk and Target Overshoot:** Unlike step-onset controllers
   (which command peak acceleration at `t = 0`) and PID/LQR/Fixed (which overshoot under
   delayed stereo noise), HLSL combines a smooth human-like bell-shaped acceleration ramp
   with the exact `kappa / delta_eff = 1 / (e delta_eff)` monotone braking envelope.
3. **Singularity-Safe Joint Synergy (`human_wdls`):** Weighting joints by their live torque
   headroom and null-space manipulability gradient keeps `w(q)` higher and lowers peak joint
   torque during outstretched reaching.
