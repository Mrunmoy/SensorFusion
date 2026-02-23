# HAL (Hardware Abstraction Layer) Design

## Purpose

Provide pure-virtual C++ interfaces that decouple sensor drivers from any
specific MCU vendor SDK or RTOS. A driver written against these interfaces
can be compiled and unit-tested on a Linux/macOS host with mock
implementations, then linked against real platform code on ESP32, nRF52,
or STM32.

## Interfaces

### 1. `II2CBus`

Abstracts a single I2C master bus.

```cpp
class II2CBus {
public:
    virtual ~II2CBus() = default;

    // Single-register helpers
    virtual bool readRegister(uint8_t devAddr, uint8_t reg,
                              uint8_t* buf, size_t len) = 0;
    virtual bool writeRegister(uint8_t devAddr, uint8_t reg,
                               const uint8_t* data, size_t len) = 0;

    // Convenience
    bool read8(uint8_t devAddr, uint8_t reg, uint8_t& out);
    bool write8(uint8_t devAddr, uint8_t reg, uint8_t val);
    bool read16BE(uint8_t devAddr, uint8_t reg, uint16_t& out);

    virtual bool probe(uint8_t devAddr) = 0;
};
```

**Design rationale**: The two pure-virtual methods (`readRegister`,
`writeRegister`) are the minimal set needed by every I2C sensor driver.
Convenience wrappers (`read8`, `write8`, `read16BE`) are non-virtual and
implemented inline in terms of the two virtuals, keeping the mock surface
small.

### 2. `IAdcChannel`

Abstracts a single ADC input channel (for analog sensors like the AD8232).

```cpp
class IAdcChannel {
public:
    virtual ~IAdcChannel() = default;
    virtual bool readRaw(int32_t& out) = 0;
    virtual bool readMillivolts(int32_t& out) = 0;
};
```

### 3. `IDelayProvider`

Abstracts time and delays. Drivers that must wait for sensor conversion
(e.g. BMP180 pressure measurement) use this instead of calling
`vTaskDelay()` or `k_msleep()` directly.

```cpp
class IDelayProvider {
public:
    virtual ~IDelayProvider() = default;
    virtual void delayMs(uint32_t ms) = 0;
    virtual void delayUs(uint32_t us) = 0;
    virtual uint64_t getTimestampUs() = 0;
};
```

## Mock Strategy

For unit tests we provide GMock-based mocks in `test/mocks/`:

- `MockI2CBus` — records calls and returns canned register data
- `MockAdcChannel` — returns pre-set ADC values
- `MockDelayProvider` — instant (no real delay), tracks calls

## File Layout

```
drivers/hal/
    II2CBus.hpp
    IAdcChannel.hpp
    IDelayProvider.hpp
test/mocks/
    MockI2CBus.hpp
    MockAdcChannel.hpp
    MockDelayProvider.hpp
```

## Platform Implementation Examples

| Interface | ESP32 (ESP-IDF) | nRF52 (Zephyr) | STM32 (HAL) |
|-----------|-----------------|----------------|--------------|
| II2CBus | `i2c_master_*` | `i2c_transfer()` | `HAL_I2C_Mem_*` |
| IAdcChannel | `adc_oneshot_*` | `adc_read()` | `HAL_ADC_*` |
| IDelayProvider | `vTaskDelay` / `esp_timer` | `k_msleep` / `k_cycle_get` | `HAL_Delay` / `DWT->CYCCNT` |
