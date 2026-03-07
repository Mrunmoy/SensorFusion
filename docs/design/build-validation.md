# Build Validation Matrix

This project uses host-side unit tests plus off-target cross-build smoke tests to reduce integration risk for embedded users.

## Goals

- Prove SensorFusion compiles with real platform toolchains.
- Validate integration shape users will copy into their own firmware trees.
- Catch SDK/toolchain/API drift early (especially ESP-IDF and CMSIS/newlib builds).

## Validation Projects

Validation projects live under:

- `/mnt/data/sandbox/embedded/sf-validation/esp32-sensor-node`
- `/mnt/data/sandbox/embedded/sf-validation/nrf52840-sensor-node`
- `/mnt/data/sandbox/embedded/sf-validation/stm32f407zgt6-sensor-node`

Each project vendors SensorFusion as `external/SensorFusion` (git submodule style) and links platform backends directly.

## Build Commands

ESP32:

```bash
source third_party/esp-idf/export.sh
cd /mnt/data/sandbox/embedded/sf-validation/esp32-sensor-node
idf.py set-target esp32
idf.py build
```

nRF52840 and STM32F407:

```bash
nix develop --command bash -lc 'cmake -S . -B build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake && cmake --build build-arm -v'
```

Run the command in each corresponding `sf-validation/<platform>-sensor-node` directory.

## Current Status (2026-03-07)

- ESP32 build passes on ESP-IDF v5.5.
- nRF52840 build passes with `gcc-arm-embedded` in nix shell.
- STM32F407 build passes with `gcc-arm-embedded` in nix shell.
- nRF/STM link warnings about `_read/_write/_close/_lseek` are expected with `nosys.specs` in smoke-build mode.
