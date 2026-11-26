# Models

A model is a JSON file with a `links` array. Each link carries the DH
parameters, the rigid-body properties, the joint limits and a complete motor /
drive block. **Every joint can have a different motor type, gear ratio and
driver settings** — nothing is shared between joints.

See the main `README.md` for the field reference and `six_axis_arm.json` for a
complete working example.

## Files

| file | description |
|---|---|
| `six_axis_arm.json` | design-study 6R arm: 0.96 m reach, harmonic-drive joints, BLDC drives, 15.1 kg |
| `human_arm.json` | human-limb-proportioned 6R humanoid arm: upper arm 0.31 m, forearm 0.26 m, hand 0.12 m; shoulder height 0.90 m; ~5.0 kg arm; `q2 = 0` is the neutral hanging pose (arm down, gravity torque ≈ 0); named postures embedded (`hanging_natural`, `hanging_vertical`, `ready_desk`, `reach_high`) |

## Validity

`python -m robosimu validate models/<file>.json` reports missing or inconsistent
fields. The validator checks positive mass/inertia, ordered joint limits,
positive electrical parameters and the motor type.

The parameters are representative, not measured from a physical arm.
