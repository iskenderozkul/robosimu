# Point-wise velocity and acceleration

Maximum values along a 0.15 m governor reach (120 ms delay), evaluated
at each link's centre of mass. Acceleration uses the recorded joint
torques through forward dynamics.

| link | name | max speed m/s | max accel m/s^2 | max ang speed rad/s | max ang accel rad/s^2 |
|---|---|---|---|---|---|
| 1 | link1_base_yaw | 0.013 | 0.89 | 0.088 | 5.90 |
| 2 | link2_shoulder | 0.279 | 12.36 | 0.443 | 19.70 |
| 3 | link3_elbow | 0.251 | 7.54 | 0.601 | 18.18 |
| 4 | link4_wrist_roll | 0.165 | 7.07 | 1.201 | 13.75 |
| 5 | link5_wrist_pitch | 0.277 | 7.91 | 2.228 | 25.28 |
| 6 | link6_tool | 0.394 | 8.78 | 2.267 | 24.71 |

## Notes

* Link 1 is the base; its centre of mass sits on the yaw axis, so its
  translational speed is small.
* Peak values occur during the acceleration phase of the reach.
* Simulation quantities from the rigid-body model; no compliance or
  structural vibration is modelled.
