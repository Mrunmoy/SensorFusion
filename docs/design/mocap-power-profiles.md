# Mocap Node Power Profiles

## Goal

Define concrete operating modes so node runtime can be traded against motion fidelity.

## Runtime Estimates (nRF52840 node)

Assumed average current:

- Performance mode: ~15-20 mA
- Balanced mode: ~10-15 mA
- Battery mode: ~7-10 mA

Approximate runtime:

- 150 mAh: ~7.5-21 h (depending on mode)
- 250 mAh: ~12.5-35 h (depending on mode)

Your 1-5 hour target is comfortably inside these ranges, even with margin for RF retries.

## Suggested Modes

## Performance Mode (capture quality priority)

- Output stream: `50 Hz` quaternion
- LSM6DSO ODR: accel/gyro `208 Hz`
- BMM350 ODR: `100 Hz`
- LPS22DF ODR: `200 Hz`
- BLE:
  - Connection interval target: `7.5-15 ms`
  - PHY: `2M` preferred
  - TX power: medium/high as needed for body shadowing

## Battery Mode (runtime priority)

- Output stream: `40 Hz` quaternion
- LSM6DSO ODR: accel/gyro `104 Hz`
- BMM350 ODR: `50 Hz`
- LPS22DF ODR: `25 Hz`
- BLE:
  - Connection interval target: `20-30 ms`
  - PHY: `1M` or `2M` based on link stability
  - TX power: minimum stable for your environment

## Firmware Rules

- Keep radio and sensor cadence aligned to avoid wakeups outside frame windows.
- Use dedicated IMU bus to prevent baro/mag reads from stalling gyro loop.
- Allow barometer decimation (not every fusion step) in battery mode.
- Report battery %, packet loss %, and average queue depth to central node.
