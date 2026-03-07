# Mocap Body Node Map

## Recommended Configurations

## 11-Node MVP

1. Pelvis (root)
2. Chest
3. Head
4. Left upper arm
5. Left forearm
6. Right upper arm
7. Right forearm
8. Left thigh
9. Left shin
10. Right thigh
11. Right shin

## 15-Node Baseline (Recommended)

11-node MVP plus:

12. Left foot
13. Right foot
14. Left hand
15. Right hand

## 17-19 Node High Fidelity

15-node baseline plus optional:

- Left shoulder/clavicle
- Right shoulder/clavicle
- Mid-spine (or split torso into upper/lower)

## Node ID Assignment (15-Node Baseline)

- `1`: Pelvis
- `2`: Chest
- `3`: Head
- `4`: Left upper arm
- `5`: Left forearm
- `6`: Right upper arm
- `7`: Right forearm
- `8`: Left thigh
- `9`: Left shin
- `10`: Right thigh
- `11`: Right shin
- `12`: Left foot
- `13`: Right foot
- `14`: Left hand
- `15`: Right hand

## Calibration Sequence (Wearable Node View)

1. Strap orientation: align each node's +X axis with segment forward direction.
2. Stationary bias phase: 3-5 s still pose to capture gyro bias and gravity baseline.
3. T-pose phase: capture reference segment quaternion offsets.
4. Dynamic check: short squat + arm swing to verify sign conventions and joint parenting.

## Notes

- Pelvis node is the kinematic root for full-body solving.
- Feet materially improve jump, landing, and turn quality.
- Shoulder/clavicle nodes improve upper-body realism during fighting and dance moves.
