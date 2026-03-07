# Per-Sensor Examples (ESP32 / STM32 / nRF52)

This folder provides one example `main.cpp` per sensor per platform backend.

## Sensors Covered

- `mpu6050`
- `lsm6dso`
- `adxl345`
- `qmc5883l`
- `bmm350`
- `lis3mdl`
- `bmp180`
- `lps22df`
- `sht40`
- `sgp40`
- `ad8232`

## Folder Layout

```text
examples/sensors/
  esp32/<sensor>/main.cpp
  stm32/<sensor>/main.cpp
  nrf52/<sensor>/main.cpp
```

## Step-by-Step: Use an Example

1. Pick platform and sensor example path.
2. Copy that `main.cpp` into your application target (or adapt its content into your existing app loop).
3. Replace platform placeholders with your board handles:
   - ESP32: `I2C_NUM_0`, `g_adc_unit`, ADC channel, and your driver init code.
   - STM32: `hi2c1`, `hadc1`, and board clock/GPIO/ADC init.
   - nRF52: `g_twim`, SAADC channel index, and nrfx/FDS init.
4. Enable backend in CMake:
   - `-DSENSORFUSION_PLATFORM=esp32`
   - `-DSENSORFUSION_PLATFORM=stm32`
   - `-DSENSORFUSION_PLATFORM=nrf52`
5. Link your app against:
   - `sensorfusion_middleware`
   - `sensorfusion_platform_<platform>`
6. Build with your platform SDK/toolchain and flash.

## Minimal CMake Pattern

```cmake
set(SENSORFUSION_PLATFORM "esp32" CACHE STRING "")
add_subdirectory(SensorFusion)
target_link_libraries(my_app PRIVATE
  sensorfusion_middleware
  sensorfusion_platform_esp32
)
```

## Notes

- These are integration skeletons, intentionally concise.
- They show correct library wiring and sensor read paths.
- Transport/logging (UART/BLE/Wi-Fi), buffering, and production error handling should be added in your app.
