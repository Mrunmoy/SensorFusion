# LPS22DF Barometric Pressure Sensor Driver Design

## Overview
ST LPS22DF: high-precision barometric pressure sensor. I2C + SPI.

## Hardware Details
- I2C address: 0x5C (SA0=LOW), 0x5D (SA0=HIGH, typical default)
- WHO_AM_I: 0xB4 at register 0x0F
- Pressure: 24-bit signed LE, 4096 LSB/hPa
- Temperature: 16-bit signed LE, 100 LSB/°C
- Factory calibrated, no user calibration registers needed
- FIFO: 128 slots

## Key Registers
| Register | Addr | Description |
|----------|------|-------------|
| WHO_AM_I | 0x0F | Expect 0xB4 |
| CTRL_REG1 | 0x10 | ODR + averaging |
| CTRL_REG2 | 0x11 | BDU, one-shot, reset |
| CTRL_REG3 | 0x12 | Interrupt polarity |
| CTRL_REG4 | 0x13 | DRDY, FIFO interrupts |
| STATUS | 0x27 | Data ready flags |
| PRESS_OUT_XL | 0x28 | Pressure out (3 bytes) |
| TEMP_OUT_L | 0x2B | Temperature out (2 bytes) |

## Conversion
- pressure_hPa = raw_24bit / 4096.0
- temperature_C = raw_16bit / 100.0
- altitude from barometric formula

## API
```cpp
class LPS22DF {
    LPS22DF(II2CBus& bus, IDelayProvider& delay, const LPS22DFConfig& cfg = {});
    bool init();
    bool readPressure(float& hPa);
    bool readTemperature(float& tempC);
    bool readAll(float& hPa, float& tempC);
    float altitudeFromPressure(float hPa, float seaLevelHPa = 1013.25f);
    bool enableDataReadyInterrupt(...);
    bool disableDataReadyInterrupt();
};
```

## Test Plan
1. Init: WHO_AM_I check, config write
2. Init failure: wrong ID, bus error
3. Read pressure: 24-bit LE → hPa
4. Read temperature: 16-bit LE → °C
5. Read all (burst)
6. Altitude calculation
7. Interrupt enable/disable
8. Null pin guard
