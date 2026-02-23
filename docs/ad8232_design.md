# AD8232 ECG Driver Design

## Device Overview

The AD8232 is a single-lead heart rate monitor analog frontend. Unlike
the I2C sensors, it outputs an **analog voltage** proportional to the
ECG signal. The MCU reads it via an ADC channel.

## Interface

Uses `IAdcChannel` HAL interface (not I2C).

## Driver API

```cpp
struct ECGSample {
    int32_t  millivolts;  // -1 if uncalibrated
    uint64_t timestampUs;
};

struct AD8232Config {
    // No I2C address — purely analog
};

class AD8232 {
    AD8232(IAdcChannel& adc, IDelayProvider& delay);
    bool readRaw(int32_t& out);
    bool readMillivolts(int32_t& out);
    bool sample(ECGSample& out);
};
```

## Design Notes

- The driver is a thin wrapper around `IAdcChannel`
- Main value: consistent API pattern with other drivers, timestamp
  attachment, future filtering hooks
- No init sequence needed (pure analog device)
- Calibration handled by the ADC HAL implementation

## Test Plan

1. `ReadRaw_ReturnsAdcValue` — delegates to IAdcChannel::readRaw
2. `ReadMillivolts_ReturnsCalibrated` — delegates to IAdcChannel::readMillivolts
3. `Sample_CombinesValueAndTimestamp` — verifies ECGSample population
4. `ReadRaw_BusFail` — ADC error propagated
5. `ReadMillivolts_BusFail` — ADC error propagated
