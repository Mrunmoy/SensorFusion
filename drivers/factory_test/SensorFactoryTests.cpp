#include "SensorFactoryTests.hpp"

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

} // namespace sf
