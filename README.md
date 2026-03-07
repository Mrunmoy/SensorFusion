# SensorFusion

[![CI](https://github.com/Mrunmoy/SensorFusion/actions/workflows/ci.yml/badge.svg)](https://github.com/Mrunmoy/SensorFusion/actions/workflows/ci.yml)

Portable sensor fusion library for embedded systems — motion capture suits, medical devices, environmental sensor nodes.

## Overview

SensorFusion provides platform-agnostic sensor drivers and fusion middleware for resource-constrained microcontrollers. All code depends only on abstract HAL interfaces, so it runs on any platform (ESP32, nRF52, STM32) with any RTOS (FreeRTOS, Zephyr, ThreadX) or bare-metal — just implement the HAL for your target.

```
┌─────────────────────────────────────────────────┐
│                  Application                    │
├─────────────────────────────────────────────────┤
│   Middleware   │ AHRS · SensorHub · FrameCodec  │
│                │ AltitudeEstimator · HeartRate   │
├─────────────────────────────────────────────────┤
│    Drivers     │ MPU6050 · LSM6DSO · QMC5883L   │
│                │ BMM350 · LPS22DF · BMP180 · …  │
├─────────────────────────────────────────────────┤
│      HAL       │ II2CBus · ISPIBus · IAdcChannel │
│  (interfaces)  │ IDelayProvider · IGpioInterrupt │
├─────────────────────────────────────────────────┤
│   Platform     │ ESP32 / nRF52 / STM32 / Host   │
└─────────────────────────────────────────────────┘
```

## Supported Sensors

| Sensor   | Type                | Interface | Data                          | I2C Address |
|----------|---------------------|-----------|-------------------------------|-------------|
| MPU6050  | 6-axis IMU          | I2C       | Accel + Gyro                  | 0x68        |
| LSM6DSO  | 6-axis IMU          | I2C       | Accel + Gyro                  | 0x6A        |
| QMC5883L | Magnetometer        | I2C       | 3-axis magnetic field (uT)    | 0x0D        |
| BMM350   | Magnetometer        | I2C       | 3-axis magnetic field (uT)    | 0x14        |
| BMP180   | Barometer           | I2C       | Pressure (hPa) + Temperature  | 0x77        |
| LPS22DF  | Barometer           | I2C       | Pressure (hPa) + Temperature  | 0x5D        |
| ADXL345  | Accelerometer       | I2C       | 3-axis acceleration (g)       | 0x53        |
| AD8232   | ECG Analog Frontend | ADC       | Heart signal (mV)             | —           |

## Middleware

| Component          | Description                                 | RAM   | .text  |
|--------------------|---------------------------------------------|-------|--------|
| MahonyAHRS         | 6/9-DOF complementary filter, quaternion out| ~36 B | 3.9 KB |
| SensorHub          | Multi-sensor manager with calibration       | ~40 B | 1.2 KB |
| FrameCodec         | Binary frame encode/decode, CRC-16 CCITT    | 0     | 4.1 KB |
| AltitudeEstimator  | Baro + accel complementary filter           | ~16 B | 0.5 KB |
| HeartRateDetector  | Pan-Tompkins R-peak detection               | ~224 B| 2.8 KB |
| Quaternion         | Header-only quaternion math                 | 16 B  | 0      |
| RingBuffer\<T,N\>  | Header-only SPSC lock-free ring buffer      | N*T   | 0      |

## Code Size

Release build (.text, GCC x86-64 — ARM Thumb-2 will be smaller):

| Library    | .text    |
|------------|----------|
| Drivers    | ~19.5 KB |
| Middleware | ~12.5 KB |
| **Total**  | **~32 KB** |

## Building

Host build (runs on your development machine):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/test/driver_tests          # 428 tests
```

Library-only build (no tests, no stub app):

```bash
cmake -B build -DSENSORFUSION_BUILD_TESTS=OFF
cmake --build build --parallel
```

Enable a shipped platform backend (optional):

```bash
cmake -B build -DSENSORFUSION_BUILD_TESTS=OFF -DSENSORFUSION_PLATFORM=esp32
cmake --build build --parallel
```

Regenerate the GitHub Pages code-size dashboard locally:

```bash
scripts/generate_size_report.sh build _site
```

Cross-target smoke build validation (off-target, toolchain-level):

```bash
# ESP32 (ESP-IDF)
source third_party/esp-idf/export.sh
cd /mnt/data/sandbox/embedded/sf-validation/esp32-sensor-node
idf.py set-target esp32
idf.py build

# nRF52840 (nix + arm-none-eabi)
cd /mnt/data/sandbox/embedded/sf-validation/nrf52840-sensor-node
nix develop --command bash -lc 'cmake -S . -B build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake && cmake --build build-arm -v'

# STM32F407 (nix + arm-none-eabi)
cd /mnt/data/sandbox/embedded/sf-validation/stm32f407zgt6-sensor-node
nix develop --command bash -lc 'cmake -S . -B build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake && cmake --build build-arm -v'
```

See `docs/design/build-validation.md` for project layout and validation goals.

## Using as a Library

Add SensorFusion to your project via `add_subdirectory`:

```cmake
set(SENSORFUSION_BUILD_TESTS OFF)
set(SENSORFUSION_PLATFORM "none") # or: esp32 / stm32 / nrf52
add_subdirectory(SensorFusion)
target_link_libraries(my_app PRIVATE sensorfusion_middleware)
```

Then implement the HAL interfaces for your platform:

```cpp
#include "II2CBus.hpp"

class MyI2C : public sf::II2CBus {
    bool readRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override {
        // Your platform's I2C read
    }
    bool writeRegister(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) override {
        // Your platform's I2C write
    }
};
```

Instantiate drivers with your HAL implementation:

```cpp
MyI2C i2c;
MyDelay delay;
sf::MPU6050 imu(i2c, delay);
imu.init();
```

## Platform Strategy

This is a **monorepo** — one branch, all platforms. The library itself is pure platform-agnostic code. Platform-specific HAL implementations go in `platform/<target>/` subdirectories:

```
platform/
  esp32/        # EspI2CBus, EspSpiBus, EspGpio*, EspAdcChannel, EspDelay, EspNvStore
  nrf52/        # NrfTwimBus, NrfSpimBus, NrfGpio*, NrfSaadcChannel, NrfDelay, NrfFdsStore
  stm32/        # StmI2CBus, StmSpiBus, StmGpio*, StmAdcChannel, StmDelay, StmNvStore
```

No per-platform branches — that's a maintenance nightmare. The library compiles and tests on any host with a C++17 compiler. Cross-compilation for a specific target only needs the matching `platform/` HAL.

Example app skeletons are provided in:

```
examples/esp32-sensor-node/
examples/stm32-env-monitor/
examples/nrf52-motion-tracker/
```

Per-sensor platform examples are provided in:

```
examples/sensors/
```

Start here for step-by-step usage instructions:
[examples/sensors/README.md](examples/sensors/README.md)

Mocap wearable-node architecture (nRF52840 focus):
[docs/design/mocap-node-architecture.md](docs/design/mocap-node-architecture.md)

Mocap body map and power profiles:
[docs/design/mocap-node-map.md](docs/design/mocap-node-map.md)
[docs/design/mocap-power-profiles.md](docs/design/mocap-power-profiles.md)

## Tests

428 unit tests, all passing. Tests use GoogleTest v1.14.0 with mock HAL implementations.

```bash
./build/test/driver_tests
```

## Version

| Component  | Version |
|------------|---------|
| Drivers    | 1.1.0   |
| Middleware | 1.0.0   |
| Firmware   | 0.1.0   |

See `drivers/hal/Version.hpp`.
