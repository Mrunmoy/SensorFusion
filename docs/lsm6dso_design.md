# LSM6DSO 6-Axis IMU Driver Design

## Overview
ST LSM6DSO: accelerometer + gyroscope. I2C + SPI.

## Hardware Details
- I2C address: 0x6A (SA0=LOW), 0x6B (SA0=HIGH)
- WHO_AM_I: 0x6C at register 0x0F
- Data: 16-bit signed, little-endian per axis
- Factory calibrated, user offset registers for accel (0x73-0x75)

## Key Registers
| Register | Addr | Description |
|----------|------|-------------|
| WHO_AM_I | 0x0F | Expect 0x6C |
| CTRL1_XL | 0x10 | Accel ODR + full-scale |
| CTRL2_G | 0x11 | Gyro ODR + full-scale |
| CTRL3_C | 0x12 | BDU, IF_INC, SW_RESET |
| INT1_CTRL | 0x0D | INT1 pin routing |
| INT2_CTRL | 0x0E | INT2 pin routing |
| STATUS_REG | 0x1E | Data ready flags |
| OUT_TEMP_L | 0x20 | Temperature (2 bytes) |
| OUTX_L_G | 0x22 | Gyro output (6 bytes) |
| OUTX_L_A | 0x28 | Accel output (6 bytes) |

## Sensitivity
| Accel Range | mg/LSB |
|-------------|--------|
| ±2g | 0.061 |
| ±4g | 0.122 |
| ±8g | 0.244 |
| ±16g | 0.488 |

| Gyro Range | mdps/LSB |
|------------|----------|
| ±125 | 4.375 |
| ±250 | 8.750 |
| ±500 | 17.50 |
| ±1000 | 35.0 |
| ±2000 | 70.0 |

## Conversion
- accel_g = raw * sensitivity_mg / 1000
- gyro_dps = raw * sensitivity_mdps / 1000
- temp_C = raw / 256.0 + 25.0

## API
```cpp
class LSM6DSO {
    LSM6DSO(II2CBus& bus, IDelayProvider& delay, const LSM6DSOConfig& cfg = {});
    bool init();
    bool readAccel(AccelData& out);
    bool readGyro(GyroData& out);
    bool readTemperature(float& tempC);
    bool readAll(AccelData& accel, GyroData& gyro, float& tempC);
    bool enableDataReadyInterrupt(...);
    bool disableDataReadyInterrupt();
};
```

## Test Plan
1. Init: WHO_AM_I, config writes, BDU/IF_INC
2. Init failure: wrong ID, bus error
3. Read accel: ±2g, ±4g, ±16g ranges
4. Read gyro: ±250, ±2000 ranges
5. Read temperature
6. Burst read (all)
7. Interrupt enable/disable
8. Null pin guard
