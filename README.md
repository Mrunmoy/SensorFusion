# SensorFusion ESP32 Project

This project runs on an ESP32 (tested on WROOM-32D) and collects real-time IMU and environmental data from sensors via I2C. The data is visualized on a 3D rotating cube through a WebSocket-enabled HTML frontend.

## Current Features

- **Drivers Implemented**:
  - `MPU6050` over I2C (accel, gyro, temp)
  - `ADXL345` over I2C (acceleration only)
- `SensorI2CBus`: Shared I2C bus abstraction with mutex protection
- Sensor polling with `startPolling(intervalMs)` / `stopPolling()` methods
- Observable sensors via `notifyObservers()`
- Web server (ESP-IDF `http_server`) with embedded `index.html`
- WebSocket for real-time data stream to browser
- 3D rotating cube UI (HTML+Three.js) driven by live sensor values

## Pin Connections

```
   ┌─────────────────────┬────────────┬────────────┬──────────────────────────┐
   │      Sensor         │   Pin Name │ ESP32 GPIO │       Description        │
   ├─────────────────────┼────────────┼────────────┼──────────────────────────┤
   │     MPU6050 (I2C)   │    VCC     │    3.3V    │ Power                    │
   │                     │    GND     │    GND     │ Ground                   │
   │                     │    SDA     │    GPIO21  │ I2C SDA (I2C_NUM_0)      │
   │                     │    SCL     │    GPIO22  │ I2C SCL (I2C_NUM_0)      │
   ├─────────────────────┼────────────┼────────────┼──────────────────────────┤
   │    ADXL345 (I2C)    │    VCC     │    3.3V    │ Power                    │
   │                     │    GND     │    GND     │ Ground                   │
   │                     │    SDA     │    GPIO17  │ I2C SDA (I2C_NUM_1)      │
   │                     │    SCL     │    GPIO16  │ I2C SCL (I2C_NUM_1)      │
   │                     │    CS      │    VCC     │ Must be HIGH for I2C     │
   │                     │    SDO     │    GND     │ Address select (0x53)    │
   └─────────────────────┴────────────┴────────────┴──────────────────────────┘
```

## How to Use

1. Flash the firmware to your ESP32
2. Connect the sensors as shown above
3. Connect to the ESP32 via SoftAP or STA mode
4. Open browser to ESP32 IP (e.g. `192.168.4.1` or STA IP)
5. Observe the live 3D cube rotating with sensor values

