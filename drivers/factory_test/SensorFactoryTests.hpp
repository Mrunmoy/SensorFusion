#pragma once

#include "FactoryTest.hpp"
#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "SensorInterface.hpp"
#include "BQ25101.hpp"
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

// Generic I2C bus round-trip test: raw write command + raw read response check
class I2CBusRoundTripTest : public IFactoryTest {
public:
    I2CBusRoundTripTest(const char* testName, II2CBus& bus, uint8_t address,
                        const uint8_t* writeBuf, size_t writeLen,
                        const uint8_t* expectedReadBuf, size_t readLen);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    II2CBus& bus_;
    uint8_t address_;
    const uint8_t* writeBuf_;
    size_t writeLen_;
    const uint8_t* expectedReadBuf_;
    size_t readLen_;
};

class HumidityPlausibilityTest : public IFactoryTest {
public:
    HumidityPlausibilityTest(const char* testName, IHumiditySensor& sensor,
                             float minPercent, float maxPercent);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    IHumiditySensor& sensor_;
    float minPercent_;
    float maxPercent_;
};

class VocBaselineTest : public IFactoryTest {
public:
    VocBaselineTest(const char* testName, IVocSensor& sensor,
                    uint16_t minRaw, uint16_t maxRaw);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    IVocSensor& sensor_;
    uint16_t minRaw_;
    uint16_t maxRaw_;
};

class BQ25101ChargePathTest : public IFactoryTest {
public:
    BQ25101ChargePathTest(const char* testName, BQ25101& charger);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    BQ25101& charger_;
};

class AccelSanityRangeTest : public IFactoryTest {
public:
    AccelSanityRangeTest(const char* testName, IAccelSensor& sensor,
                         float minAxisG, float maxAxisG,
                         float minNormG, float maxNormG);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    IAccelSensor& sensor_;
    float minAxisG_;
    float maxAxisG_;
    float minNormG_;
    float maxNormG_;
};

class MagSanityRangeTest : public IFactoryTest {
public:
    MagSanityRangeTest(const char* testName, IMagSensor& sensor, float minAxisUt, float maxAxisUt);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    IMagSensor& sensor_;
    float minAxisUt_;
    float maxAxisUt_;
};

class BaroSanityRangeTest : public IFactoryTest {
public:
    BaroSanityRangeTest(const char* testName, IBaroSensor& sensor, float minHpa, float maxHpa);

    const char* name() const override { return name_; }
    TestResult run() override;

private:
    const char* name_;
    IBaroSensor& sensor_;
    float minHpa_;
    float maxHpa_;
};

} // namespace sf
