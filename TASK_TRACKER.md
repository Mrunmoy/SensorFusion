# SensorFusion Implementation Tracker

Use this file as the working checklist while implementing the roadmap in `TODO.md`.
Execution order is intentional: deliver small vertical slices with TDD and docs updates each time.

## Workflow Rules (for every task)
- [ ] Write/extend failing tests first (`test/test_*.cpp`)
- [ ] Implement minimal production code to pass tests
- [ ] Refactor while keeping tests green
- [ ] Run full suite: `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel && ./build/test/driver_tests`
- [ ] Update relevant docs (`README.md`, `docs/design/*`, driver design docs)
- [ ] Update this tracker and check off completed items

## Phase 1: Immediate Integration Gaps
- [x] Add ADXL345 middleware-facing interface support
- [x] Define environmental interfaces (`IHumiditySensor`, `IVocSensor`)
- [x] Integrate SHT40 + SGP40 into SensorHub using new interfaces
- [x] Bump `DRIVER_VERSION` in `drivers/hal/Version.hpp`
- [x] Regenerate code-size dashboard output

## Phase 2: Factory Test Overhaul
- [ ] Add per-sensor self-test coverage (ID, sanity, range)
- [ ] Add bus verification helpers (probe + raw read/write checks)
- [ ] Add environmental validation (SHT40 plausibility, SGP40 baseline)
- [ ] Add BQ25101 charge-path checks
- [ ] Add structured pass/fail report format for manufacturing

## Phase 3: Calibration Completion
- [ ] Magnetometer calibration (QMC5883L, BMM350, LIS3MDL)
- [ ] Accelerometer calibration (MPU6050, LSM6DSO, ADXL345)
- [ ] Gyroscope bias calibration (MPU6050, LSM6DSO)
- [ ] Barometer sea-level reference support (BMP180, LPS22DF)
- [ ] First-boot auto-calibration when no valid data exists
- [ ] Calibration validation + fallback defaults on corruption

## Phase 4: NV Storage Implementation
- [ ] Implement EEPROM/flash-backed `INvStore` backend
- [ ] Wire `CalibrationStore` load/save to persistent backend
- [ ] Add corruption detection via CRC checks
- [ ] Add basic wear-leveling strategy (if flash backend)

## Phase 5: Platform HAL Backends
- [ ] ESP32 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [ ] STM32 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [ ] nRF52 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [ ] Add one end-to-end example per platform
- [ ] Document integration paths (submodule, FetchContent, custom HAL)

## Phase 6: Future / Stretch
- [ ] CI cross-compilation jobs for ARM targets
- [ ] Power profiling hooks
- [ ] Transport integration planning (BLE/WiFi)
- [ ] OTA update support planning

## Progress Log
- [x] Tracker created (2026-03-07)
- [x] Phase 1.1 complete: ADXL345 middleware-facing interface support via `IAccelSensor` + SensorHub accel-only path (2026-03-07)
- [x] Phase 1.2 complete: Added `IHumiditySensor`/`IVocSensor` and wired SHT40/SGP40 to middleware interfaces (2026-03-07)
- [x] Phase 1.3 complete: SensorHub humidity/VOC registration and read APIs with host-side tests (2026-03-07)
- [x] Phase 1.4 complete: Bumped driver version to `1.1.0` with regression test coverage (2026-03-07)
- [x] Phase 1.5 complete: Regenerated size dashboard and verified new objects appear in output (2026-03-07)
