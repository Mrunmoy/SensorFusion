# Platform Backends

SensorFusion now ships optional platform HAL backends under `platform/<target>`.

## Available Targets

- `platform/esp32`: ESP-IDF based implementations
  - `EspI2CBus`, `EspSpiBus`, `EspGpioInput`/`EspGpioOutput`/`EspGpioInterrupt`
  - `EspAdcChannel`, `EspDelay`, `EspNvStore`
- `platform/stm32`: STM32 HAL based implementations
  - `StmI2CBus`, `StmSpiBus`, `StmGpioInput`/`StmGpioOutput`/`StmGpioInterrupt`
  - `StmAdcChannel`, `StmDelay`, `StmNvStore`
- `platform/nrf52`: nRF5 SDK / nrfx based implementations
  - `NrfTwimBus`, `NrfSpimBus`, `NrfGpioInput`/`NrfGpioOutput`/`NrfGpioInterrupt`
  - `NrfSaadcChannel`, `NrfDelay`, `NrfFdsStore`

## Enabling a Backend

```bash
cmake -B build -DSENSORFUSION_PLATFORM=esp32 -DSENSORFUSION_BUILD_TESTS=OFF
cmake --build build --parallel
```

Valid values: `none` (default), `esp32`, `stm32`, `nrf52`.

## Notes

- Platform backends are optional and not built when `SENSORFUSION_PLATFORM=none`.
- Host CI keeps using `none`; target SDK/toolchain builds should be done in platform-specific pipelines.
- For interrupt adapters (`StmGpioInterrupt`, `NrfGpioInterrupt`), call `handleIrq(...)` from your ISR callback path.
