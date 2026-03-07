# Mocap Wearable Node Architecture (nRF52840)

## Scope

This document defines the body-worn mocap sensor node only. Central receiver and PC retargeting are out of scope for this phase.

## Hardware Baseline

- MCU: `nRF52840` (Cortex-M4F, BLE 5, 1 MB flash, 256 KB RAM)
- IMU: `LSM6DSO` on dedicated `TWIM0`
- Magnetometer: `BMM350` on shared `TWIM1`
- Barometer: `LPS22DF` on shared `TWIM1`
- Power: LiPo + `MCP73831` charger + `AP2112K-3.3` LDO
- Runtime target: 4-5 hours on 150-250 mAh pack

## Firmware Data Path

1. Read IMU (accel + gyro) every cycle.
2. Read magnetometer when available; fallback to 6-DOF AHRS if mag sample fails.
3. Read barometer opportunistically for height event features (jump/somersault).
4. Run AHRS and produce orientation quaternion.
5. Emit quaternion frame at `50 Hz` over BLE.

`middleware/motion/MocapNodePipeline` now implements this sensor fusion loop as a reusable unit.

## Timing Contract

- Output cadence: `20 ms` period (`50 Hz`).
- AHRS `dt`: fixed at `0.02 s` in current MVP.
- Recommended next step: switch to measured `dt` after central-node sync protocol is ready.

## Transport Contract (Node -> Central)

- Current frame format: `FrameCodec::encodeQuaternion(nodeId, timestampUs, q, ...)`.
- Required fields per packet:
  - `nodeId`
  - `timestampUs` (node local clock for now)
  - quaternion `(w, x, y, z)`
- Pending additions: battery %, link quality, calibration state.

## Immediate Next Engineering Tasks

- Implement BLE sender in `examples/nrf52-motion-tracker/main.cpp` (`bleSend` placeholder).
- Add calibration and stationary-detection commands.
- Add timestamp synchronization with the central node.

## Related Docs

- Body placement and node IDs: `docs/design/mocap-node-map.md`
- Runtime/power operating modes: `docs/design/mocap-power-profiles.md`
