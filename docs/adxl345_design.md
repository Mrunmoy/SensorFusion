# ADXL345 Driver Design

## Device Overview

3-axis digital accelerometer at I2C address **0x53** (or 0x1D if ALT
ADDRESS is high). 13-bit resolution with configurable ranges up to ±16g.
Data is **little-endian**.

## Register Map (subset)

| Register | Addr | Description |
|----------|------|-------------|
| DEVID | 0x00 | Device ID (expected 0xE5) |
| BW_RATE | 0x2C | Data rate and power mode |
| POWER_CTL | 0x2D | Power control (bit 3 = measure) |
| DATA_FORMAT | 0x31 | Range, full-res, justify |
| DATAX0 | 0x32 | X-axis data (6 bytes, LE) |

## DATA_FORMAT (0x31)

```
Bit 3: FULL_RES — 1 = 4 mg/LSB regardless of range
Bit 1-0: Range — 00=±2g, 01=±4g, 10=±8g, 11=±16g
```

## Conversion

In full-resolution mode: 4 mg/LSB (0.004 g/LSB) regardless of range.
In fixed 10-bit mode: depends on range.

We'll use full-resolution mode, output in **g** (not m/s²).

## Driver API

```cpp
enum class AdxlRange : uint8_t { G2=0, G4=1, G8=2, G16=3 };

struct ADXL345Config {
    AdxlRange range   = AdxlRange::G4;
    bool      fullRes = true;
    uint8_t   address = 0x53;
};

class ADXL345 {
    bool init();
    bool readAccel(AccelData& out);  // reuses AccelData from MPU6050
};
```

## Init Sequence

1. Read DEVID (0x00), verify == 0xE5
2. Write POWER_CTL (0x2D) = 0x08 (measurement mode)
3. Write DATA_FORMAT (0x31) with range + full-res bit
4. Write BW_RATE (0x2C) = 0x0A (100 Hz)

## Test Plan

1. `Init_Success` — correct DEVID, all writes succeed
2. `Init_WrongId` — wrong DEVID returns false
3. `Init_BusFail` — I2C failure propagated
4. `ReadAccel_ConvertsToG` — verify 4 mg/LSB in full-res
5. `ReadAccel_Negative` — verify signed LE parsing
6. `ReadAccel_BusFail` — I2C error during read
7. `RangeConfig` — verify DATA_FORMAT register value
