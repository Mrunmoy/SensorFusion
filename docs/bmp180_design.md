# BMP180 Driver Design

## Device Overview

Barometric pressure sensor + thermometer at I2C address **0x77**.
Provides calibrated temperature, pressure, and derived altitude.
Uses factory-programmed calibration coefficients stored in EEPROM.

## Register Map

| Register | Addr | Description |
|----------|------|-------------|
| Calib (AC1..MD) | 0xAA–0xBF | 11 calibration coefficients (22 bytes, BE) |
| CHIP_ID | 0xD0 | Device ID (expected 0x55) |
| SOFT_RESET | 0xE0 | Write 0xB6 to reset |
| CTRL_MEAS | 0xF4 | Measurement control |
| OUT_MSB | 0xF6 | ADC result high byte |
| OUT_LSB | 0xF7 | ADC result low byte |
| OUT_XLSB | 0xF8 | ADC result extra low (pressure only) |

## Calibration Coefficients

Read once at init. 22 bytes, 11 x int16_t big-endian:

| Name | Addr | Type |
|------|------|------|
| AC1 | 0xAA | int16 |
| AC2 | 0xAC | int16 |
| AC3 | 0xAE | int16 |
| AC4 | 0xB0 | uint16 |
| AC5 | 0xB2 | uint16 |
| AC6 | 0xB4 | uint16 |
| B1 | 0xB6 | int16 |
| B2 | 0xB8 | int16 |
| MB | 0xBA | int16 |
| MC | 0xBC | int16 |
| MD | 0xBE | int16 |

## Measurement Commands

| Command | Value | Wait |
|---------|-------|------|
| Temperature | 0x2E | 4.5 ms |
| Pressure (OSS=0) | 0x34 | 4.5 ms |
| Pressure (OSS=1) | 0x74 | 7.5 ms |
| Pressure (OSS=2) | 0xB4 | 13.5 ms |
| Pressure (OSS=3) | 0xF4 | 25.5 ms |

## Temperature Compensation Algorithm

```
X1 = (UT - AC6) * AC5 / 2^15
X2 = MC * 2^11 / (X1 + MD)
B5 = X1 + X2
T = (B5 + 8) / 2^4   (in 0.1°C)
```

## Pressure Compensation Algorithm

Complex multi-step calculation using B5 from temperature, all 11
calibration coefficients, and oversampling setting. See BMP180 datasheet
section 3.5.

## Driver API

```cpp
enum class BMP180Oss { ULTRA_LOW = 0, STANDARD = 1, HIGH_RES = 2, ULTRA_HIGH = 3 };

struct BMP180Config {
    BMP180Oss oss     = BMP180Oss::STANDARD;
    uint8_t   address = 0x77;
};

class BMP180 {
    bool init();                    // reads chip ID + calibration
    bool readTemperature(float& tempC);
    bool readPressure(int32_t& pressurePa);
    bool readTempAndPressure(float& tempC, int32_t& pressurePa);
    static float pressureToAltitude(int32_t pressurePa, float seaLevelPa = 101325.0f);
};
```

## Init Sequence

1. Read CHIP_ID, verify == 0x55
2. Read 22 bytes of calibration data from 0xAA
3. Parse into AC1–MD

## Test Plan

1. `Init_ChipIdOk` — correct ID, calibration read succeeds
2. `Init_WrongChipId` — wrong ID, init fails
3. `Init_CalibReadFail` — I2C error reading calibration
4. `ReadTemp_Converts` — inject known UT, verify with datasheet example
5. `ReadPressure_Converts` — inject known UP, verify with datasheet example
6. `ReadTempAndPressure` — combined read
7. `Altitude_Calculation` — verify barometric formula
8. `BusFail_ReadTemp` — I2C error during measurement
