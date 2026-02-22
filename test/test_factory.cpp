#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "FactoryTest.hpp"
#include "SensorFactoryTests.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

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
