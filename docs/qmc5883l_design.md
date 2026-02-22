# QMC5883L Driver Design

## Device Overview

3-axis magnetometer at I2C address **0x0D**. Measures Earth's magnetic
field for heading/compass applications. Data is **little-endian** (unlike
MPU6050).

## Register Map

| Register | Addr | Description |
|----------|------|-------------|
| DATA_X_LSB | 0x00 | X output low byte |
| DATA_X_MSB | 0x01 | X output high byte |
| DATA_Y_LSB | 0x02 | Y low |
| DATA_Y_MSB | 0x03 | Y high |
| DATA_Z_LSB | 0x04 | Z low |
| DATA_Z_MSB | 0x05 | Z high |
| STATUS | 0x06 | DRDY (bit 0), OVL (bit 1), DOR (bit 2) |
| CTRL1 | 0x09 | Mode[1:0], ODR[3:2], RNG[5:4], OSR[7:6] |
| CTRL2 | 0x0A | SOFT_RST (bit 7), ROL_PNT (bit 6) |
| SET_RST | 0x0B | Set/Reset period register |

## CTRL1 Bit Layout

```
[7:6] OSR   — 00=512, 01=256, 10=128, 11=64
[5:4] RNG   — 00=2 Gauss, 01=8 Gauss
[3:2] ODR   — 00=10Hz, 01=50Hz, 10=100Hz, 11=200Hz
[1:0] MODE  — 00=Standby, 01=Continuous
```

## Conversion

| Range | LSB/Gauss | LSB/µT |
|-------|-----------|--------|
| 2G | 12000 | 1200 |
| 8G | 3000 | 300 |

Output in microTesla: `µT = raw / lsb_per_uT`

## Driver API

```cpp
enum class MagOsr  { OSR_512, OSR_256, OSR_128, OSR_64 };
enum class MagOdr  { HZ_10, HZ_50, HZ_100, HZ_200 };
enum class MagRange { GAUSS_2, GAUSS_8 };
enum class MagMode { STANDBY, CONTINUOUS };

struct QMC5883LConfig {
    MagOsr   osr     = MagOsr::OSR_512;
    MagRange range   = MagRange::GAUSS_8;
    MagOdr   odr     = MagOdr::HZ_200;
    MagMode  mode    = MagMode::CONTINUOUS;
    uint8_t  address = 0x0D;
};

struct MagData { float x, y, z; };  // in µT

class QMC5883L {
    bool init();
    bool isDataReady(bool& ready);
    bool readRaw(int16_t& x, int16_t& y, int16_t& z);
    bool readMicroTesla(MagData& out);
    float headingDegrees(float mx, float my);
};
```

## Init Sequence

1. Write 0x80 to CTRL2 (soft reset)
2. Delay 10 ms
3. Write 0x01 to SET_RST (recommended by datasheet)
4. Write CTRL1 with OSR | RNG | ODR | MODE
5. Delay 5 ms

## Test Plan

1. `Init_Success` — verify soft reset + config register writes
2. `Init_BusFail` — verify failure propagation
3. `IsDataReady` — inject STATUS byte, verify DRDY bit
4. `ReadRaw_LittleEndian` — verify LE byte order parsing
5. `ReadMicroTesla_2G` — raw→µT at 2 Gauss range
6. `ReadMicroTesla_8G` — raw→µT at 8 Gauss range
7. `HeadingDegrees` — verify atan2 heading calculation
8. `ReadWhenNotReady` — verify STATUS polling
