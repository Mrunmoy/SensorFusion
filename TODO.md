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
- [ ] **Wear leveling** (if flash-based) — simple page-rotation scheme
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
- [ ] `II2CBus` → STM32 HAL I2C (register + raw command modes)
- [ ] `ISPIBus` → STM32 HAL SPI
- [ ] `IGpioInterrupt` → EXTI interrupt
- [ ] `IGpioInput` / `IGpioOutput` → GPIO read/write
- [ ] `IAdcChannel` → STM32 ADC (for AD8232)
- [ ] `IDelayProvider` → SysTick / HAL_Delay + DWT cycle counter
- [ ] `INvStore` → internal flash or external EEPROM

### nRF52 (e.g., nRF52840)
- [ ] `II2CBus` → nRF TWI/TWIM driver
- [ ] `ISPIBus` → nRF SPIM driver
- [ ] `IGpioInterrupt` → GPIOTE
- [ ] `IGpioInput` / `IGpioOutput` → nRF GPIO
- [ ] `IAdcChannel` → nRF SAADC
- [ ] `IDelayProvider` → nrf_delay + RTC/timer
- [ ] `INvStore` → FDS (Flash Data Storage) or external EEPROM

### ESP32 (ESP-IDF)
- [ ] `II2CBus` → ESP-IDF I2C master driver
- [ ] `ISPIBus` → ESP-IDF SPI master
- [ ] `IGpioInterrupt` → GPIO ISR
- [ ] `IGpioInput` / `IGpioOutput` → gpio_get/set_level
- [ ] `IAdcChannel` → ESP ADC oneshot/continuous
- [ ] `IDelayProvider` → vTaskDelay / esp_timer
- [ ] `INvStore` → NVS (Non-Volatile Storage) partition

### Common platform concerns
- [ ] RTOS integration (FreeRTOS mutex for bus sharing, Zephyr k_mutex, bare-metal critical sections)
- [ ] DMA support for bulk sensor reads (optional, per platform)
- [ ] Low-power modes (sleep/wake coordination with sensor duty cycling)
- [ ] Board-specific pin mapping / config headers

---

## 6. Future Considerations

- [ ] OTA firmware update support
- [ ] BLE/WiFi transport layer for sensor data
- [ ] Power profiling per platform
- [ ] CI cross-compilation for target platforms (ARM toolchain in GitHub Actions)
