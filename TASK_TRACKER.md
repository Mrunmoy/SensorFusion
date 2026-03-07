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
- [x] Add per-sensor self-test coverage (ID, sanity, range)
- [x] Add bus verification helpers (probe + raw read/write checks)
- [x] Add environmental validation (SHT40 plausibility, SGP40 baseline)
- [x] Add BQ25101 charge-path checks
- [x] Add structured pass/fail report format for manufacturing

## Phase 3: Calibration Completion
- [x] Magnetometer calibration (QMC5883L, BMM350, LIS3MDL)
- [x] Accelerometer calibration (MPU6050, LSM6DSO, ADXL345)
- [x] Gyroscope bias calibration (MPU6050, LSM6DSO)
- [x] Barometer sea-level reference support (BMP180, LPS22DF)
- [x] First-boot auto-calibration when no valid data exists
- [x] Calibration validation + fallback defaults on corruption

## Phase 4: NV Storage Implementation
- [x] Implement EEPROM/flash-backed `INvStore` backend
- [x] Wire `CalibrationStore` load/save to persistent backend
- [x] Add corruption detection via CRC checks
- [x] Add basic wear-leveling strategy (if flash backend)

## Phase 5: Platform HAL Backends
- [x] ESP32 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [x] STM32 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [x] nRF52 backend (`II2CBus`, `ISPIBus`, `IGpio*`, `IAdcChannel`, `IDelayProvider`, `INvStore`)
- [x] Add one end-to-end example per platform
- [x] Document integration paths (submodule, FetchContent, custom HAL)
- [x] Add one per-sensor example per platform with step-by-step usage docs

## Phase 6: Future / Stretch
- [x] Off-target cross-build validation harness for ESP32/nRF52840/STM32F407
- [ ] CI cross-compilation jobs for ARM targets
- [ ] Power profiling hooks
- [ ] Transport integration planning (BLE/WiFi)
- [ ] OTA update support planning

## Phase 7: Mocap Wearable Node (nRF52840)
- [x] Freeze node hardware baseline (LSM6DSO + BMM350 + LPS22DF, dual-TWIM, 50 Hz quaternion output)
- [x] Add platform-agnostic mocap node pipeline in middleware (`MocapNodePipeline`)
- [x] Add host-side unit tests for pipeline success/fallback behavior
- [x] Update nRF52 motion-tracker example to dual-bus node architecture
- [x] Define body node map (11/15/17+ variants) and fixed node ID plan
- [x] Define performance vs battery operating profiles and wire mode selection in example
- [ ] Add BLE transport implementation (NUS/custom GATT) for quaternion frames
- [ ] Add node calibration command flow (stationary/T-pose triggers)
- [ ] Add timestamp sync protocol with central node
- [ ] Add battery and health telemetry frame
- [ ] Add node-side logging/replay hooks for hard-motion tuning

## Progress Log
- [x] Tracker created (2026-03-07)
- [x] Phase 1.1 complete: ADXL345 middleware-facing interface support via `IAccelSensor` + SensorHub accel-only path (2026-03-07)
- [x] Phase 1.2 complete: Added `IHumiditySensor`/`IVocSensor` and wired SHT40/SGP40 to middleware interfaces (2026-03-07)
- [x] Phase 1.3 complete: SensorHub humidity/VOC registration and read APIs with host-side tests (2026-03-07)
- [x] Phase 1.4 complete: Bumped driver version to `1.1.0` with regression test coverage (2026-03-07)
- [x] Phase 1.5 complete: Regenerated size dashboard and verified new objects appear in output (2026-03-07)
- [x] Phase 2.1 complete: Added `I2CBusRoundTripTest` helper with host-side coverage for write/read/mismatch failures (2026-03-07)
- [x] Phase 2.2 complete: Added humidity/VOC factory validation helpers for SHT40/SGP40 plausibility checks (2026-03-07)
- [x] Phase 2.3 complete: Added BQ25101 charge-path TS toggle verification helper and tests (2026-03-07)
- [x] Phase 2.4 complete: Added `formatFactoryReportCsv` output for manufacturing logs with test coverage (2026-03-07)
- [x] Phase 2.5 complete: Added per-sensor accel/mag/baro sanity-range self-test helpers with host-side coverage (2026-03-07)
- [x] Phase 3.1 complete: Added calibration sanity validation and `loadOrDefault` fallback behavior with tests (2026-03-07)
- [x] Phase 3.2 complete: Added BARO calibration slot support for persisted sea-level reference data (2026-03-07)
- [x] Phase 3.3 complete: Added first-boot calibration seeding helper (`ensureInitialized`) with persistence tests (2026-03-07)
- [x] Phase 3.4 complete: Added magnetometer hard-iron/soft-iron fitting helper (`CalibrationFitter`) with tests (2026-03-07)
- [x] Phase 3.5 complete: Added accelerometer offset/scale and gyroscope bias fitters with tests (2026-03-07)
- [x] Phase 4.1 complete: Added AT24Cxx EEPROM-backed `INvStore` implementation and build wiring (`AT24CxxNvStore`) with unit tests (2026-03-07)
- [x] Phase 4.2 complete: Added `CalibrationStore` integration tests against AT24Cxx backend with sampled bus behavior (2026-03-07)
- [x] Phase 4.3 complete: Verified CRC corruption detection in AT24Cxx-backed calibration persistence tests (2026-03-07)
- [x] Phase 4.4 complete: Added flash-style `WearLevelingNvStore` page-rotation backend with host-side tests for rotation, cross-page writes, and corruption fallback (2026-03-07)
- [x] Phase 5.1 complete: Added optional `SENSORFUSION_PLATFORM` selection and platform build routing in top-level CMake (2026-03-07)
- [x] Phase 5.2 complete: Implemented ESP32 HAL backend (`platform/esp32`) for I2C/SPI/GPIO/ADC/Delay/NVS (2026-03-07)
- [x] Phase 5.3 complete: Implemented STM32 HAL backend (`platform/stm32`) for I2C/SPI/GPIO/ADC/Delay/NV region store (2026-03-07)
- [x] Phase 5.4 complete: Implemented nRF52 HAL backend (`platform/nrf52`) for TWIM/SPIM/GPIO/SAADC/Delay/FDS store (2026-03-07)
- [x] Phase 5.5 complete: Added platform example skeleton apps and backend integration docs (`platform/README.md`, design doc, README updates) (2026-03-07)
- [x] Phase 5.6 complete: Added per-sensor example matrix (`examples/sensors/`) for ESP32/STM32/nRF52 and step-by-step user guide (2026-03-07)
- [x] Phase 6.1 complete: Added off-target cross-build validation projects and verified successful builds for ESP32 (ESP-IDF), nRF52840, and STM32F407 (nix + arm-none-eabi) (2026-03-07)
- [x] Phase 7.1 complete: Added `MocapNodePipeline` middleware slice with tests and updated nRF52 motion-tracker example for LSM6DSO + BMM350 + LPS22DF dual-bus architecture (2026-03-07)
- [x] Phase 7.2 complete: Added mocap body node map and power/runtime profile docs; updated nRF52 example with selectable `PERFORMANCE` and `BATTERY` node modes (2026-03-07)
