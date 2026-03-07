#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockGpioInput.hpp"
#include "MockGpioOutput.hpp"
#include "BQ25101.hpp"
#include "FactoryTest.hpp"
#include "SensorFactoryTests.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;
using ::testing::InSequence;

// --- FactoryTestRunner tests ---

class StubTest : public IFactoryTest {
public:
    StubTest(const char* n, TestStatus s) : name_(n), status_(s) {}
    const char* name() const override { return name_; }
    TestResult run() override { return {name_, status_, nullptr}; }
private:
    const char* name_;
    TestStatus status_;
};

class MockHumiditySensor : public IHumiditySensor {
public:
    MOCK_METHOD(bool, readHumidityPercent, (float&), (override));
};

class MockVocSensor : public IVocSensor {
public:
    MOCK_METHOD(bool, readVocRaw, (uint16_t&), (override));
};

TEST(FactoryTestRunnerTest, RunAllCollectsResults) {
    StubTest t1("test1", TestStatus::PASS);
    StubTest t2("test2", TestStatus::FAIL);
    StubTest t3("test3", TestStatus::PASS);

    FactoryTestRunner runner;
    EXPECT_TRUE(runner.addTest(&t1));
    EXPECT_TRUE(runner.addTest(&t2));
    EXPECT_TRUE(runner.addTest(&t3));

    TestResult results[3];
    size_t ran = runner.runAll(results, 3);
    EXPECT_EQ(ran, 3u);
    EXPECT_EQ(runner.passCount(), 2u);
    EXPECT_EQ(runner.failCount(), 1u);
    EXPECT_EQ(results[0].status, TestStatus::PASS);
    EXPECT_EQ(results[1].status, TestStatus::FAIL);
}

TEST(FactoryTestRunnerTest, RunAllWithCallback) {
    StubTest t1("pass", TestStatus::PASS);
    StubTest t2("fail", TestStatus::FAIL);

    FactoryTestRunner runner;
    runner.addTest(&t1);
    runner.addTest(&t2);

    int callCount = 0;
    auto cb = [](const TestResult&, void* ctx) {
        ++(*static_cast<int*>(ctx));
    };

    runner.runAll(cb, &callCount);
    EXPECT_EQ(callCount, 2);
    EXPECT_EQ(runner.passCount(), 1u);
    EXPECT_EQ(runner.failCount(), 1u);
}

TEST(FactoryTestRunnerTest, AddNullTestFails) {
    FactoryTestRunner runner;
    EXPECT_FALSE(runner.addTest(nullptr));
}

TEST(FactoryTestRunnerTest, MaxTestsEnforced) {
    StubTest tests[FactoryTestRunner::MAX_TESTS + 1] = {
        {"t0", TestStatus::PASS}, {"t1", TestStatus::PASS},
        {"t2", TestStatus::PASS}, {"t3", TestStatus::PASS},
        {"t4", TestStatus::PASS}, {"t5", TestStatus::PASS},
        {"t6", TestStatus::PASS}, {"t7", TestStatus::PASS},
        {"t8", TestStatus::PASS}, {"t9", TestStatus::PASS},
        {"t10", TestStatus::PASS}, {"t11", TestStatus::PASS},
        {"t12", TestStatus::PASS}, {"t13", TestStatus::PASS},
        {"t14", TestStatus::PASS}, {"t15", TestStatus::PASS},
        {"overflow", TestStatus::PASS},
    };

    FactoryTestRunner runner;
    for (size_t i = 0; i < FactoryTestRunner::MAX_TESTS; ++i) {
        EXPECT_TRUE(runner.addTest(&tests[i]));
    }
    EXPECT_FALSE(runner.addTest(&tests[FactoryTestRunner::MAX_TESTS]));
}

// --- I2CSensorProbeTest tests ---

TEST(SensorProbeTest, ProbeAndIdMatch) {
    MockI2CBus bus;
    EXPECT_CALL(bus, probe(0x6A)).WillOnce(Return(true));
    EXPECT_CALL(bus, readRegister(0x6A, 0x0F, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x6C;
            return true;
        });

    I2CSensorProbeTest test("LSM6DSO probe", bus, 0x6A, 0x0F, 0x6C);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(SensorProbeTest, ProbeFails) {
    MockI2CBus bus;
    EXPECT_CALL(bus, probe(0x6A)).WillOnce(Return(false));

    I2CSensorProbeTest test("LSM6DSO probe", bus, 0x6A, 0x0F, 0x6C);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "I2C probe failed");
}

TEST(SensorProbeTest, WhoAmIMismatch) {
    MockI2CBus bus;
    EXPECT_CALL(bus, probe(0x6A)).WillOnce(Return(true));
    EXPECT_CALL(bus, readRegister(0x6A, 0x0F, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xFF; // wrong ID
            return true;
        });

    I2CSensorProbeTest test("LSM6DSO probe", bus, 0x6A, 0x0F, 0x6C);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "WHO_AM_I mismatch");
}

// --- I2CSensorDataTest tests ---

TEST(SensorDataTest, PassingDataCheck) {
    auto check = [](void*, const char**) -> bool { return true; };
    I2CSensorDataTest test("data check", check, nullptr);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(SensorDataTest, FailingDataCheck) {
    auto check = [](void*, const char** reason) -> bool {
        *reason = "data out of range";
        return false;
    };
    I2CSensorDataTest test("data check", check, nullptr);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "data out of range");
}

TEST(SensorDataTest, NullFunctionSkips) {
    I2CSensorDataTest test("data check", nullptr, nullptr);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::SKIPPED);
}

TEST(SensorDataTest, ContextPassedThrough) {
    int value = 42;
    auto check = [](void* ctx, const char**) -> bool {
        return *static_cast<int*>(ctx) == 42;
    };
    I2CSensorDataTest test("context test", check, &value);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

// --- Integration: full factory test scenario ---

TEST(FactoryIntegration, MultiSensorProbe) {
    MockI2CBus bus;

    // LSM6DSO at 0x6A
    EXPECT_CALL(bus, probe(0x6A)).WillOnce(Return(true));
    EXPECT_CALL(bus, readRegister(0x6A, 0x0F, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x6C;
            return true;
        });

    // LPS22DF at 0x5D
    EXPECT_CALL(bus, probe(0x5D)).WillOnce(Return(true));
    EXPECT_CALL(bus, readRegister(0x5D, 0x0F, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xB4;
            return true;
        });

    // BMM350 at 0x14
    EXPECT_CALL(bus, probe(0x14)).WillOnce(Return(true));
    EXPECT_CALL(bus, readRegister(0x14, 0x00, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x33;
            return true;
        });

    I2CSensorProbeTest lsmTest("LSM6DSO", bus, 0x6A, 0x0F, 0x6C);
    I2CSensorProbeTest lpsTest("LPS22DF", bus, 0x5D, 0x0F, 0xB4);
    I2CSensorProbeTest bmmTest("BMM350", bus, 0x14, 0x00, 0x33);

    FactoryTestRunner runner;
    runner.addTest(&lsmTest);
    runner.addTest(&lpsTest);
    runner.addTest(&bmmTest);

    TestResult results[3];
    runner.runAll(results, 3);

    EXPECT_EQ(runner.passCount(), 3u);
    EXPECT_EQ(runner.failCount(), 0u);
}

// --- I2C bus round-trip tests ---

TEST(I2CBusRoundTripTest, RoundTripSuccess) {
    MockI2CBus bus;
    const uint8_t writePayload[2] = {0xAA, 0x55};
    const uint8_t expectedRead[3] = {0x10, 0x20, 0x30};

    EXPECT_CALL(bus, rawWrite(0x44, _, 2))
        .WillOnce([](uint8_t, const uint8_t* data, size_t len) {
            return len == 2 && data[0] == 0xAA && data[1] == 0x55;
        });
    EXPECT_CALL(bus, rawRead(0x44, _, 3))
        .WillOnce([](uint8_t, uint8_t* data, size_t len) {
            if (len != 3) return false;
            data[0] = 0x10;
            data[1] = 0x20;
            data[2] = 0x30;
            return true;
        });

    sf::I2CBusRoundTripTest test("i2c roundtrip", bus, 0x44, writePayload, 2, expectedRead, 3);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(I2CBusRoundTripTest, WriteFailurePropagates) {
    MockI2CBus bus;
    const uint8_t writePayload[1] = {0x01};
    const uint8_t expectedRead[1] = {0x00};

    EXPECT_CALL(bus, rawWrite(0x44, _, 1)).WillOnce(Return(false));

    sf::I2CBusRoundTripTest test("i2c roundtrip", bus, 0x44, writePayload, 1, expectedRead, 1);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "I2C rawWrite failed");
}

TEST(I2CBusRoundTripTest, ReadFailurePropagates) {
    MockI2CBus bus;
    const uint8_t writePayload[1] = {0x01};
    const uint8_t expectedRead[2] = {0x12, 0x34};

    EXPECT_CALL(bus, rawWrite(0x44, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, rawRead(0x44, _, 2)).WillOnce(Return(false));

    sf::I2CBusRoundTripTest test("i2c roundtrip", bus, 0x44, writePayload, 1, expectedRead, 2);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "I2C rawRead failed");
}

TEST(I2CBusRoundTripTest, DataMismatchFails) {
    MockI2CBus bus;
    const uint8_t writePayload[1] = {0x99};
    const uint8_t expectedRead[2] = {0x12, 0x34};

    EXPECT_CALL(bus, rawWrite(0x44, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, rawRead(0x44, _, 2))
        .WillOnce([](uint8_t, uint8_t* data, size_t len) {
            if (len != 2) return false;
            data[0] = 0x12;
            data[1] = 0x00;
            return true;
        });

    sf::I2CBusRoundTripTest test("i2c roundtrip", bus, 0x44, writePayload, 1, expectedRead, 2);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "I2C round-trip mismatch");
}

// --- Environmental validation tests ---

TEST(HumidityPlausibilityTest, PassesForNominalHumidity) {
    MockHumiditySensor humidity;
    EXPECT_CALL(humidity, readHumidityPercent(_))
        .WillOnce([](float& out) {
            out = 45.0f;
            return true;
        });

    sf::HumidityPlausibilityTest test("SHT40 plausibility", humidity, 5.0f, 95.0f);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(HumidityPlausibilityTest, FailsWhenOutOfRange) {
    MockHumiditySensor humidity;
    EXPECT_CALL(humidity, readHumidityPercent(_))
        .WillOnce([](float& out) {
            out = 99.5f;
            return true;
        });

    sf::HumidityPlausibilityTest test("SHT40 plausibility", humidity, 5.0f, 95.0f);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "humidity out of range");
}

TEST(HumidityPlausibilityTest, FailsOnReadError) {
    MockHumiditySensor humidity;
    EXPECT_CALL(humidity, readHumidityPercent(_)).WillOnce(Return(false));

    sf::HumidityPlausibilityTest test("SHT40 plausibility", humidity, 5.0f, 95.0f);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "humidity read failed");
}

TEST(VocBaselineTest, PassesForNominalVoc) {
    MockVocSensor voc;
    EXPECT_CALL(voc, readVocRaw(_))
        .WillOnce([](uint16_t& out) {
            out = 15000;
            return true;
        });

    sf::VocBaselineTest test("SGP40 baseline", voc, 100, 60000);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(VocBaselineTest, FailsWhenOutOfRange) {
    MockVocSensor voc;
    EXPECT_CALL(voc, readVocRaw(_))
        .WillOnce([](uint16_t& out) {
            out = 50;
            return true;
        });

    sf::VocBaselineTest test("SGP40 baseline", voc, 100, 60000);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "VOC baseline out of range");
}

TEST(VocBaselineTest, FailsOnReadError) {
    MockVocSensor voc;
    EXPECT_CALL(voc, readVocRaw(_)).WillOnce(Return(false));

    sf::VocBaselineTest test("SGP40 baseline", voc, 100, 60000);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "VOC read failed");
}

// --- BQ25101 charge-path verification tests ---

TEST(BQ25101ChargePathTest, PassesWhenChargeStateChangesAfterTsToggle) {
    MockGpioInput chgPin;
    MockGpioOutput tsPin;
    BQ25101 charger(chgPin, tsPin);

    {
        InSequence seq;
        EXPECT_CALL(tsPin, write(false)).WillOnce(Return(true)); // inhibit charge
        EXPECT_CALL(chgPin, read(_))
            .WillOnce([](bool& level) { level = true; return true; }); // NOT_CHARGING
        EXPECT_CALL(tsPin, write(true)).WillOnce(Return(true));  // enable charge
        EXPECT_CALL(chgPin, read(_))
            .WillOnce([](bool& level) { level = false; return true; }); // CHARGING
    }

    sf::BQ25101ChargePathTest test("BQ25101 charge path", charger);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::PASS);
}

TEST(BQ25101ChargePathTest, FailsWhenTsWriteFails) {
    MockGpioInput chgPin;
    MockGpioOutput tsPin;
    BQ25101 charger(chgPin, tsPin);

    EXPECT_CALL(tsPin, write(false)).WillOnce(Return(false));

    sf::BQ25101ChargePathTest test("BQ25101 charge path", charger);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "failed to inhibit charging");
}

TEST(BQ25101ChargePathTest, FailsWhenStatusReadFails) {
    MockGpioInput chgPin;
    MockGpioOutput tsPin;
    BQ25101 charger(chgPin, tsPin);

    EXPECT_CALL(tsPin, write(false)).WillOnce(Return(true));
    EXPECT_CALL(chgPin, read(_)).WillOnce(Return(false)); // UNKNOWN

    sf::BQ25101ChargePathTest test("BQ25101 charge path", charger);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "CHG status read failed");
}

TEST(BQ25101ChargePathTest, FailsWhenChargeStateDoesNotChange) {
    MockGpioInput chgPin;
    MockGpioOutput tsPin;
    BQ25101 charger(chgPin, tsPin);

    {
        InSequence seq;
        EXPECT_CALL(tsPin, write(false)).WillOnce(Return(true));
        EXPECT_CALL(chgPin, read(_))
            .WillOnce([](bool& level) { level = true; return true; }); // NOT_CHARGING
        EXPECT_CALL(tsPin, write(true)).WillOnce(Return(true));
        EXPECT_CALL(chgPin, read(_))
            .WillOnce([](bool& level) { level = true; return true; }); // still NOT_CHARGING
    }

    sf::BQ25101ChargePathTest test("BQ25101 charge path", charger);
    TestResult r = test.run();
    EXPECT_EQ(r.status, TestStatus::FAIL);
    EXPECT_STREQ(r.detail, "CHG did not respond to TS toggle");
}
