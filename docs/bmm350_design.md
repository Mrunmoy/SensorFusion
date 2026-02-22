# BMM350 Magnetometer Driver Design

## Overview
Bosch BMM350 3-axis magnetometer. I2C only (up to 1MHz). Fixed range ±2000µT.

## Hardware Details
- I2C address: 0x14 (ADSEL=LOW), 0x15 (ADSEL=HIGH)
- CHIP_ID: 0x33 at register 0x00
- Data: 24-bit (21-bit significant) little-endian per axis
- OTP: 31x16-bit factory calibration words

## Key Registers
| Register | Addr | Description |
|----------|------|-------------|
| CHIP_ID | 0x00 | Expect 0x33 |
| PMU_CMD | 0x06 | Power mode command |
| PMU_CMD_STATUS_0 | 0x07 | PMU status |
| INT_CTRL | 0x2E | Interrupt config |
| INT_STATUS | 0x30 | Interrupt status |
| MAG_X_XLSB | 0x31 | X data start (3 bytes per axis) |
| OTP_CMD_REG | 0x50 | OTP access command |
| OTP_DATA_MSB | 0x52 | OTP data high byte |
| OTP_DATA_LSB | 0x53 | OTP data low byte |
| OTP_STATUS | 0x55 | OTP ready status |
| CMD | 0x7E | Soft reset |

## Power Modes
- Suspend (0x00), Normal (0x01), Forced (0x03), Forced-fast (0x04)
- Write mode to PMU_CMD, poll PMU_CMD_STATUS_0 for confirmation

## OTP Compensation
Factory calibration data stored in OTP must be read at boot:
1. Read 31 words from OTP (write word address to OTP_CMD_REG, read OTP_DATA)
2. Extract: offsets, sensitivities, temperature coefficients, cross-axis terms
3. Apply compensation pipeline to raw readings

For our driver we simplify: read OTP, store coefficients, apply linear compensation.

## Conversion
- Raw 21-bit signed → compensated µT using OTP-derived factors
- Temperature: raw * 0.00204 + (-25.49) approximation (simplified)

## API
```cpp
class BMM350 {
    BMM350(II2CBus& bus, IDelayProvider& delay, const BMM350Config& cfg = {});
    bool init();                    // reset, read OTP, set normal mode
    bool readMag(MagData& out);     // compensated µT
    bool readTemperature(float& tempC);
    bool enableDataReadyInterrupt(IGpioInterrupt* pin, IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();
};
```

## Test Plan
1. Init: CHIP_ID check, OTP read sequence, mode set
2. Init failure: wrong ID, bus error
3. Read mag: raw → compensated µT with known OTP values
4. Read temperature
5. Interrupt enable/disable
6. Null interrupt pin guard
