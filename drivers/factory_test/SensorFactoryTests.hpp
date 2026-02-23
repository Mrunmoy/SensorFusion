#pragma once

#include "FactoryTest.hpp"
#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include <cstdint>

namespace sf {

// Generic I2C sensor probe test: checks probe + WHO_AM_I register
class I2CSensorProbeTest : public IFactoryTest {
public:
    I2CSensorProbeTest(const char* sensorName, II2CBus& bus,
                       uint8_t address, uint8_t whoAmIReg, uint8_t expectedId);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    II2CBus& bus_;
    uint8_t address_;
    uint8_t whoAmIReg_;
    uint8_t expectedId_;
};

// Generic I2C sensor data sanity test: init sensor, read data, check non-zero
class I2CSensorDataTest : public IFactoryTest {
public:
    // dataReader: function that inits sensor and reads a value.
    // Returns true if data looks sane. context is passed through.
    using DataCheckFn = bool(*)(void* context, const char** failReason);

    I2CSensorDataTest(const char* testName, DataCheckFn fn, void* context);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    DataCheckFn fn_;
    void* ctx_;
};

} // namespace sf
