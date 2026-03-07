#include "SensorFactoryTests.hpp"
#include <cstring>

namespace sf {

I2CSensorProbeTest::I2CSensorProbeTest(const char* sensorName, II2CBus& bus,
                                       uint8_t address, uint8_t whoAmIReg,
                                       uint8_t expectedId)
    : name_(sensorName), bus_(bus), address_(address),
      whoAmIReg_(whoAmIReg), expectedId_(expectedId)
{}

TestResult I2CSensorProbeTest::run() {
    // Step 1: I2C probe
    if (!bus_.probe(address_)) {
        return {name_, TestStatus::FAIL, "I2C probe failed"};
    }

    // Step 2: Read WHO_AM_I
    uint8_t id = 0;
    if (!bus_.read8(address_, whoAmIReg_, id)) {
        return {name_, TestStatus::FAIL, "WHO_AM_I read failed"};
    }

    if (id != expectedId_) {
        return {name_, TestStatus::FAIL, "WHO_AM_I mismatch"};
    }

    return {name_, TestStatus::PASS, nullptr};
}

I2CSensorDataTest::I2CSensorDataTest(const char* testName, DataCheckFn fn, void* context)
    : name_(testName), fn_(fn), ctx_(context)
{}

TestResult I2CSensorDataTest::run() {
    const char* reason = nullptr;
    if (!fn_) {
        return {name_, TestStatus::SKIPPED, "no check function"};
    }
    if (!fn_(ctx_, &reason)) {
        return {name_, TestStatus::FAIL, reason ? reason : "data check failed"};
    }
    return {name_, TestStatus::PASS, nullptr};
}

I2CBusRoundTripTest::I2CBusRoundTripTest(const char* testName, II2CBus& bus, uint8_t address,
                                         const uint8_t* writeBuf, size_t writeLen,
                                         const uint8_t* expectedReadBuf, size_t readLen)
    : name_(testName), bus_(bus), address_(address), writeBuf_(writeBuf), writeLen_(writeLen),
      expectedReadBuf_(expectedReadBuf), readLen_(readLen)
{}

TestResult I2CBusRoundTripTest::run() {
    if (!writeBuf_ || !expectedReadBuf_ || writeLen_ == 0 || readLen_ == 0) {
        return {name_, TestStatus::SKIPPED, "invalid round-trip config"};
    }

    if (!bus_.rawWrite(address_, writeBuf_, writeLen_)) {
        return {name_, TestStatus::FAIL, "I2C rawWrite failed"};
    }

    uint8_t readBuf[16] = {};
    if (readLen_ > sizeof(readBuf)) {
        return {name_, TestStatus::SKIPPED, "read length too large"};
    }

    if (!bus_.rawRead(address_, readBuf, readLen_)) {
        return {name_, TestStatus::FAIL, "I2C rawRead failed"};
    }

    if (std::memcmp(readBuf, expectedReadBuf_, readLen_) != 0) {
        return {name_, TestStatus::FAIL, "I2C round-trip mismatch"};
    }

    return {name_, TestStatus::PASS, nullptr};
}

HumidityPlausibilityTest::HumidityPlausibilityTest(const char* testName, IHumiditySensor& sensor,
                                                   float minPercent, float maxPercent)
    : name_(testName), sensor_(sensor), minPercent_(minPercent), maxPercent_(maxPercent)
{}

TestResult HumidityPlausibilityTest::run() {
    float humidityPercent = 0.0f;
    if (!sensor_.readHumidityPercent(humidityPercent)) {
        return {name_, TestStatus::FAIL, "humidity read failed"};
    }
    if (humidityPercent < minPercent_ || humidityPercent > maxPercent_) {
        return {name_, TestStatus::FAIL, "humidity out of range"};
    }
    return {name_, TestStatus::PASS, nullptr};
}

VocBaselineTest::VocBaselineTest(const char* testName, IVocSensor& sensor,
                                 uint16_t minRaw, uint16_t maxRaw)
    : name_(testName), sensor_(sensor), minRaw_(minRaw), maxRaw_(maxRaw)
{}

TestResult VocBaselineTest::run() {
    uint16_t vocRaw = 0;
    if (!sensor_.readVocRaw(vocRaw)) {
        return {name_, TestStatus::FAIL, "VOC read failed"};
    }
    if (vocRaw < minRaw_ || vocRaw > maxRaw_) {
        return {name_, TestStatus::FAIL, "VOC baseline out of range"};
    }
    return {name_, TestStatus::PASS, nullptr};
}

} // namespace sf
