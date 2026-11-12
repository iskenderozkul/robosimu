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

## Validity

`python -m robosimu validate models/<file>.json` reports missing or inconsistent
fields. The validator checks positive mass/inertia, ordered joint limits,
positive electrical parameters and the motor type.

The parameters are representative, not measured from a physical arm.
