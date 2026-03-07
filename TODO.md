# SensorFusion — TODO Tracker

## Status Legend
- [ ] Not started
- [~] In progress
- [x] Done

---

## 1. Driver & Middleware Gaps

- [x] **ADXL345 middleware interface** — accel-only interface added via `IAccelSensor` and SensorHub support
- [x] **Environmental sensor interfaces** — `IHumiditySensor` and `IVocSensor` added; SHT40/SGP40 now implement them
- [x] **SensorHub integration** — SensorHub now supports humidity/VOC registration and read paths via SHT40/SGP40 interfaces
- [x] **Version bump** — `DRIVER_VERSION` updated to `1.1.0` for expanded driver set (12 total)
- [x] **Code size dashboard** — regenerated report includes new objects (LIS3MDL, SHT40, SGP40, BQ25101)

---

## 2. Factory Test — Needs Major Work

The current factory test code (`drivers/factory_test/`) is very rudimentary and nowhere near production-quality. Needs:

- [x] Comprehensive per-sensor self-test (WHO_AM_I, data sanity, range checks)
- [x] Communication bus verification (I2C probe, raw read/write round-trip)
- [x] Environmental sensor validation (SHT40 humidity plausibility, SGP40 VOC baseline)
- [x] BQ25101 charge path verification (CHG pin responds to TS toggle)
- [x] Test report output (pass/fail per sensor, structured for manufacturing)

---

## 3. Calibration — Incomplete

Current `CalibrationStore` is minimal. Needs to be completed for all sensors requiring calibration:

- [x] **Magnetometer calibration** — added hard-iron/diagonal soft-iron fitter with host-side tests
- [x] **Accelerometer calibration** — added axis-sweep offset/scale fitter with host-side tests
- [x] **Gyroscope calibration** — added stationary bias fitter with host-side tests
- [x] **Barometer calibration** — added BARO calibration slot support for persisted sea-level pressure reference
- [x] **Auto-calibrate on first boot** — added first-boot initialization helper to seed and persist defaults when missing
- [x] **Calibration validation** — sanity checks added for saved/loaded params, with `loadOrDefault` fallback support

---

## 4. EEPROM / NV Storage Driver

Need a persistent storage driver for calibration parameters:

- [x] **EEPROM driver** — I2C EEPROM backend implemented for AT24Cxx family (`AT24CxxNvStore`)
- [x] **Wire up to `INvStore` HAL** — concrete `INvStore` implementation now available via `AT24CxxNvStore`
- [x] **CalibrationStore ↔ EEPROM** — covered with host-side integration tests over AT24Cxx bus model
- [x] **Wear leveling** (if flash-based) — added `WearLevelingNvStore` with per-page slot rotation and CRC-validated latest-record selection
- [x] **CRC protection** — corruption detection enforced in `CalibrationStore` and verified in AT24Cxx integration test

---

## 5. Platform Layer — HAL Implementations

> **Design doc:** [`docs/design/library-integration.md`](docs/design/library-integration.md)
> — covers distribution model (submodule / FetchContent), CMake integration,
> platform backend structure, and a full ESP32 example.

All drivers are platform-agnostic behind HAL interfaces. Need real implementations.
Ship optional `platform/<target>/` backends so users don't rewrite the same I2C/GPIO wrappers.
Users who prefer custom HAL just implement the interfaces directly (Method 3 in design doc).

### STM32 (e.g., STM32F4, STM32L4)
- [x] `II2CBus` → STM32 HAL I2C (register + raw command modes)
- [x] `ISPIBus` → STM32 HAL SPI
- [x] `IGpioInterrupt` → EXTI interrupt hook (`StmGpioInterrupt::handleIrq`)
- [x] `IGpioInput` / `IGpioOutput` → GPIO read/write
- [x] `IAdcChannel` → STM32 ADC (for AD8232)
- [x] `IDelayProvider` → SysTick / HAL_Delay + DWT cycle counter
- [x] `INvStore` → internal flash-backed region adapter

### nRF52 (e.g., nRF52840)
- [x] `II2CBus` → nRF TWI/TWIM driver
- [x] `ISPIBus` → nRF SPIM driver
- [x] `IGpioInterrupt` → GPIOTE hook (`NrfGpioInterrupt::handleIrq`)
- [x] `IGpioInput` / `IGpioOutput` → nRF GPIO
- [x] `IAdcChannel` → nRF SAADC
- [x] `IDelayProvider` → nrf_delay + RTC/timer
- [x] `INvStore` → FDS (Flash Data Storage) backend

### ESP32 (ESP-IDF)
- [x] `II2CBus` → ESP-IDF I2C master driver
- [x] `ISPIBus` → ESP-IDF SPI master
- [x] `IGpioInterrupt` → GPIO ISR
- [x] `IGpioInput` / `IGpioOutput` → gpio_get/set_level
- [x] `IAdcChannel` → ESP ADC oneshot
- [x] `IDelayProvider` → vTaskDelay / esp_timer
- [x] `INvStore` → NVS (Non-Volatile Storage) partition

### Common platform concerns
- [ ] RTOS integration (FreeRTOS mutex for bus sharing, Zephyr k_mutex, bare-metal critical sections)
- [ ] DMA support for bulk sensor reads (optional, per platform)
- [ ] Low-power modes (sleep/wake coordination with sensor duty cycling)
- [ ] Board-specific pin mapping / config headers
- [x] Per-sensor example loops for each platform (`examples/sensors/<platform>/<sensor>/main.cpp`) with usage guide

---

## 6. Future Considerations

- [ ] OTA firmware update support
- [ ] BLE/WiFi transport layer for sensor data
- [ ] Power profiling per platform
- [ ] CI cross-compilation for target platforms (ARM toolchain in GitHub Actions)
