# MPU6050 Driver Design

## Device Overview

The MPU6050 is a 6-axis IMU combining a 3-axis accelerometer and 3-axis
gyroscope on a single chip, communicating via I2C at address **0x68**
(or 0x69 if AD0 is high).

## Register Map (subset)

| Register | Addr | Description |
|----------|------|-------------|
| SMPLRT_DIV | 0x19 | Sample rate = Gyro_Rate / (1 + DIV) |
| CONFIG | 0x1A | DLPF bandwidth selection |
| GYRO_CONFIG | 0x1B | Gyro full-scale: ±250/500/1000/2000 dps |
| ACCEL_CONFIG | 0x1C | Accel full-scale: ±2/4/8/16 g |
| INT_PIN_CFG | 0x37 | I2C bypass enable (bit 1) |
| ACCEL_XOUT_H | 0x3B | Accel data start (6 bytes, big-endian) |
| TEMP_OUT_H | 0x41 | Temperature data (2 bytes) |
| GYRO_XOUT_H | 0x43 | Gyro data start (6 bytes, big-endian) |
| USER_CTRL | 0x6A | I2C master disable (bit 5) |
| PWR_MGMT_1 | 0x6B | Power management / clock source |
| WHO_AM_I | 0x75 | Device ID (expected 0x68) |

## Driver API

```cpp
namespace sf {

struct AccelData { float x, y, z; };  // in g
struct GyroData  { float x, y, z; };  // in deg/s

enum class AccelRange { G2, G4, G8, G16 };
enum class GyroRange  { DPS250, DPS500, DPS1000, DPS2000 };
enum class DlpfBandwidth { BW260, BW184, BW94, BW44, BW21, BW10, BW5 };

struct MPU6050Config {
    AccelRange   accelRange = AccelRange::G2;
    GyroRange    gyroRange  = GyroRange::DPS250;
    DlpfBandwidth dlpf      = DlpfBandwidth::BW44;
    uint8_t      sampleRateDiv = 4;  // 200 Hz at 1 kHz internal
    bool         i2cBypass     = true;
    uint8_t      address       = 0x68;
};

class MPU6050 {
public:
    MPU6050(II2CBus& bus, IDelayProvider& delay, const MPU6050Config& cfg = {});

    bool init();
    bool readAccel(AccelData& out);
    bool readGyro(GyroData& out);
    bool readTemperature(float& tempC);
    bool readAll(AccelData& accel, GyroData& gyro, float& tempC);

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    MPU6050Config cfg_;
    float accelScale_;  // LSB/g for current range
    float gyroScale_;   // LSB/(deg/s) for current range
};
```

## Conversion Factors

| Accel Range | LSB/g |
|-------------|-------|
| ±2g | 16384 |
| ±4g | 8192 |
| ±8g | 4096 |
| ±16g | 2048 |

| Gyro Range | LSB/(deg/s) |
|------------|-------------|
| ±250 dps | 131 |
| ±500 dps | 65.5 |
| ±1000 dps | 32.8 |
| ±2000 dps | 16.4 |

## Temperature formula

`temp_C = raw / 340.0 + 36.53`

## Init sequence

1. Write 0x80 to PWR_MGMT_1 (device reset)
2. Delay 100 ms
3. Write 0x01 to PWR_MGMT_1 (PLL with X gyro ref)
4. Read WHO_AM_I, verify == 0x68
5. Write SMPLRT_DIV
6. Write CONFIG (DLPF)
7. Write GYRO_CONFIG (range)
8. Write ACCEL_CONFIG (range)
9. If i2cBypass: disable I2C master in USER_CTRL, set bypass in INT_PIN_CFG

## Test Plan

1. `Init_Success` — mock correct WHO_AM_I, verify all register writes
2. `Init_WrongId` — mock wrong WHO_AM_I, expect init() returns false
3. `Init_BusFail` — mock I2C failure, expect init() returns false
4. `ReadAccel_ConvertsToG` — inject raw data, verify g conversion
5. `ReadGyro_ConvertsToDps` — inject raw data, verify deg/s conversion
6. `ReadTemp_Converts` — inject raw temp, verify formula
7. `ReadAll_ReturnsAllData` — verify burst read of 14 bytes
8. `ConfigRanges` — verify scale factors for each range
9. `BypassMode` — verify USER_CTRL and INT_PIN_CFG writes
