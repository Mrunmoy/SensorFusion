# Mocap BLE Contract (Node -> Central)

## Packet Format

- Payload bytes are raw `FrameCodec::encodeQuaternion(...)` frames.
- Frame type: `SensorType::QUATERNION`
- Frame fields:
  - `nodeId` (1 byte)
  - `timestampUs` (8 bytes, node-local for now)
  - quaternion (`w,x,y,z`, 16 bytes float32)
  - CRC-16

Frame size is fixed at `30` bytes.

## GATT/ATT Requirements

- Notification payload must carry full 30-byte frame.
- ATT overhead is 3 bytes, so minimum ATT MTU is:
  - `33` bytes (`30 + 3`)
- Recommended ATT MTU: `185` (default in current example wiring).

## Transport Behavior

`MocapBleTransport` provides:

- MTU guard (`canSendQuaternion()`)
- Configurable retry count for transient notify failures
- Delivery stats:
  - sent count
  - retry count
  - dropped count

## Integration Point (nRF52 Example)

`examples/nrf52-motion-tracker/main.cpp` calls a weak symbol:

`extern "C" bool sf_mocap_ble_notify(const uint8_t* data, size_t len);`

Implement this symbol in board BLE code (NUS/custom GATT) to forward notifications.
