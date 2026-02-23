#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"
#include "LPS22DF.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR           = 0x5D;
static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
static constexpr uint8_t REG_CTRL1      = 0x10;
static constexpr uint8_t REG_CTRL2      = 0x11;
static constexpr uint8_t REG_CTRL4      = 0x13;
static constexpr uint8_t REG_PRESS_OUT  = 0x28;
static constexpr uint8_t REG_TEMP_OUT   = 0x2B;

class LPS22DFTest : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectReset() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1))
            .WillOnce(Return(true))   // reset
            .WillOnce(Return(true));  // BDU
        EXPECT_CALL(delay, delayMs(10));
    }

    void expectWhoAmI(uint8_t id) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_WHO_AM_I, _, 1))
            .WillOnce([id](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = id;
                return true;
            });
    }

    void expectConfig() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1)).WillOnce(Return(true));
    }

    void expectFullInit() {
        expectReset();
        expectWhoAmI(0xB4);
        expectConfig();
    }
};

TEST_F(LPS22DFTest, InitSuccess) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    EXPECT_TRUE(sensor.init());
}

TEST_F(LPS22DFTest, InitWrongId) {
    // Reset writes CTRL2 once (then fails before BDU)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1))
        .WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(10));
    expectWhoAmI(0xFF);
    LPS22DF sensor(bus, delay);
    EXPECT_FALSE(sensor.init());
}

TEST_F(LPS22DFTest, InitBusFail) {
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1))
        .WillOnce(Return(false));
    LPS22DF sensor(bus, delay);
    EXPECT_FALSE(sensor.init());
}

TEST_F(LPS22DFTest, ReadPressure) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // 1000.0 hPa * 4096 = 4096000 = 0x3E8000
    // LE: 0x00, 0x80, 0x3E
    EXPECT_CALL(bus, readRegister(ADDR, REG_PRESS_OUT, _, 3))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; buf[1] = 0x80; buf[2] = 0x3E;
            return true;
        });

    float hPa;
    ASSERT_TRUE(sensor.readPressure(hPa));
    EXPECT_NEAR(hPa, 1000.0f, 0.01f);
}

TEST_F(LPS22DFTest, ReadTemperature) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // 25.0°C * 100 = 2500 = 0x09C4, LE: 0xC4, 0x09
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_OUT, _, 2))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xC4; buf[1] = 0x09;
            return true;
        });

    float tempC;
    ASSERT_TRUE(sensor.readTemperature(tempC));
    EXPECT_NEAR(tempC, 25.0f, 0.01f);
}

TEST_F(LPS22DFTest, ReadAllBurst) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // Pressure 1000.0 hPa + temp 25.0°C in 5 contiguous bytes
    EXPECT_CALL(bus, readRegister(ADDR, REG_PRESS_OUT, _, 5))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // Pressure: 4096000 = 0x3E8000 → 0x00, 0x80, 0x3E
            buf[0] = 0x00; buf[1] = 0x80; buf[2] = 0x3E;
            // Temperature: 2500 = 0x09C4 → 0xC4, 0x09
            buf[3] = 0xC4; buf[4] = 0x09;
            return true;
        });

    float hPa, tempC;
    ASSERT_TRUE(sensor.readAll(hPa, tempC));
    EXPECT_NEAR(hPa, 1000.0f, 0.01f);
    EXPECT_NEAR(tempC, 25.0f, 0.01f);
}

TEST_F(LPS22DFTest, AltitudeCalculation) {
    // At sea level pressure, altitude = 0
    EXPECT_NEAR(LPS22DF::altitudeFromPressure(1013.25f), 0.0f, 0.1f);
    // ~900 hPa ≈ ~988m
    float alt = LPS22DF::altitudeFromPressure(900.0f);
    EXPECT_GT(alt, 900.0f);
    EXPECT_LT(alt, 1100.0f);
}

TEST_F(LPS22DFTest, ReadPressureBusFail) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_PRESS_OUT, _, 3))
        .WillOnce(Return(false));

    float hPa;
    EXPECT_FALSE(sensor.readPressure(hPa));
}

TEST_F(LPS22DFTest, EnableDataReadyInterrupt) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x18); // INT_EN | DRDY
            return true;
        });

    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(sensor.enableDataReadyInterrupt(&intPin, cb, &called));

    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(LPS22DFTest, DisableDataReadyInterrupt) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1))
        .WillOnce(Return(true))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x00);
            return true;
        });
    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(sensor.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(sensor.disableDataReadyInterrupt());
}

TEST_F(LPS22DFTest, EnableInterruptBusFail) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    MockGpioInterrupt intPin;
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1))
        .WillOnce(Return(false));

    EXPECT_FALSE(sensor.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
}

TEST_F(LPS22DFTest, EnableInterruptNullPinFails) {
    expectFullInit();
    LPS22DF sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_FALSE(sensor.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}
