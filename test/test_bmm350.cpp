#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"
#include "BMM350.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR          = 0x14;
static constexpr uint8_t REG_CHIP_ID   = 0x00;
static constexpr uint8_t REG_PMU_AGGR  = 0x04;
static constexpr uint8_t REG_PMU_AXIS  = 0x05;
static constexpr uint8_t REG_PMU_CMD   = 0x06;
static constexpr uint8_t REG_PMU_STAT  = 0x07;
static constexpr uint8_t REG_INT_CTRL  = 0x2E;
static constexpr uint8_t REG_MAG_DATA  = 0x31;
static constexpr uint8_t REG_TEMP_DATA = 0x3A;
static constexpr uint8_t REG_OTP_CMD   = 0x50;
static constexpr uint8_t REG_OTP_MSB   = 0x52;
static constexpr uint8_t REG_OTP_LSB   = 0x53;
static constexpr uint8_t REG_OTP_STAT  = 0x55;
static constexpr uint8_t REG_CMD       = 0x7E;

class BMM350Test : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectSoftReset() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CMD, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(24));
    }

    void expectChipId(uint8_t id) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_CHIP_ID, _, 1))
            .WillOnce([id](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = id;
                return true;
            });
    }

    // Expect OTP read sequence: 5 words read with OTP_CMD writes
    // Returns all zeros for simplicity (offsets=0, sensitivities=0, t0=0)
    void expectOtpRead() {
        // Each OTP word: write OTP_CMD, delayUs(300), read status, read MSB, read LSB
        // We expect 5 OTP word reads (T0, OFF_X, OFF_Y, OFF_Z/SENS_X, SENS_Y, TCO_X)
        // Total 6 words
        EXPECT_CALL(bus, writeRegister(ADDR, REG_OTP_CMD, _, 1))
            .Times(6)
            .WillRepeatedly(Return(true));
        EXPECT_CALL(delay, delayUs(300)).Times(6);
        EXPECT_CALL(bus, readRegister(ADDR, REG_OTP_STAT, _, 1))
            .Times(6)
            .WillRepeatedly([](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = 0x01; // OTP ready
                return true;
            });
        EXPECT_CALL(bus, readRegister(ADDR, REG_OTP_MSB, _, 1))
            .Times(6)
            .WillRepeatedly([](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = 0x00;
                return true;
            });
        EXPECT_CALL(bus, readRegister(ADDR, REG_OTP_LSB, _, 1))
            .Times(6)
            .WillRepeatedly([](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = 0x00;
                return true;
            });
    }

    void expectNormalMode() {
        // Enable all axes
        EXPECT_CALL(bus, writeRegister(ADDR, REG_PMU_AXIS, _, 1)).WillOnce(Return(true));
        // ODR + averaging
        EXPECT_CALL(bus, writeRegister(ADDR, REG_PMU_AGGR, _, 1)).WillOnce(Return(true));
        // Normal mode command
        EXPECT_CALL(bus, writeRegister(ADDR, REG_PMU_CMD, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(38));
        // Verify mode
        EXPECT_CALL(bus, readRegister(ADDR, REG_PMU_STAT, _, 1))
            .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = 0x01; // normal mode
                return true;
            });
    }

    void expectFullInit() {
        expectSoftReset();
        expectChipId(0x33);
        expectOtpRead();
        expectNormalMode();
    }
};

TEST_F(BMM350Test, InitSuccess) {
    expectFullInit();
    BMM350 mag(bus, delay);
    EXPECT_TRUE(mag.init());
}

TEST_F(BMM350Test, InitWrongChipId) {
    expectSoftReset();
    expectChipId(0xFF);
    BMM350 mag(bus, delay);
    EXPECT_FALSE(mag.init());
}

TEST_F(BMM350Test, InitBusFail) {
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CMD, _, 1))
        .WillOnce(Return(false));
    BMM350 mag(bus, delay);
    EXPECT_FALSE(mag.init());
}

TEST_F(BMM350Test, ReadMagWithZeroOtp) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // With zero OTP (offsets=0, sensitivities=0), raw values pass through directly
    // Raw X=14550 (little-endian 24-bit) → 14550/14.55 = 1000 µT
    EXPECT_CALL(bus, readRegister(ADDR, REG_MAG_DATA, _, 9))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // X = 14550 = 0x38D6 → LE 24-bit: 0xD6, 0x38, 0x00
            buf[0] = 0xD6; buf[1] = 0x38; buf[2] = 0x00;
            // Y = 0
            buf[3] = 0x00; buf[4] = 0x00; buf[5] = 0x00;
            // Z = 9000 = 0x2328 → LE 24-bit: 0x28, 0x23, 0x00
            buf[6] = 0x28; buf[7] = 0x23; buf[8] = 0x00;
            return true;
        });

    MagData m;
    ASSERT_TRUE(mag.readMag(m));
    // X: 14550 / 14.55 = 1000.0 µT
    EXPECT_NEAR(m.x, 1000.0f, 1.0f);
    EXPECT_NEAR(m.y, 0.0f, 0.01f);
    // Z: 9000 / 9.0 = 1000.0 µT
    EXPECT_NEAR(m.z, 1000.0f, 1.0f);
}

TEST_F(BMM350Test, ReadMagNegativeValues) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // X = -14550 → 21-bit two's complement
    // -14550 in 21-bit: 0x1FC72A → LE: 0x2A, 0xC7, 0x1F
    EXPECT_CALL(bus, readRegister(ADDR, REG_MAG_DATA, _, 9))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x2A; buf[1] = 0xC7; buf[2] = 0x1F;
            buf[3] = 0x00; buf[4] = 0x00; buf[5] = 0x00;
            buf[6] = 0x00; buf[7] = 0x00; buf[8] = 0x00;
            return true;
        });

    MagData m;
    ASSERT_TRUE(mag.readMag(m));
    EXPECT_NEAR(m.x, -1000.0f, 1.0f);
}

TEST_F(BMM350Test, ReadTemperature) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // raw = 25000 → 25000 * 0.00204 + (-25.49) = 51.0 - 25.49 = 25.51°C
    // 25000 = 0x61A8 → LE 24-bit: 0xA8, 0x61, 0x00
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_DATA, _, 3))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xA8; buf[1] = 0x61; buf[2] = 0x00;
            return true;
        });

    float tempC;
    ASSERT_TRUE(mag.readTemperature(tempC));
    EXPECT_NEAR(tempC, 25.51f, 0.1f);
}

TEST_F(BMM350Test, ReadMagBusFail) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_MAG_DATA, _, 9))
        .WillOnce(Return(false));

    MagData m;
    EXPECT_FALSE(mag.readMag(m));
}

TEST_F(BMM350Test, EnableDataReadyInterrupt) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_CTRL, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x07);
            return true;
        });

    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(mag.enableDataReadyInterrupt(&intPin, cb, &called));

    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(BMM350Test, DisableDataReadyInterrupt) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_CTRL, _, 1))
        .WillOnce(Return(true))   // enable
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x00);
            return true;
        });
    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(mag.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(mag.disableDataReadyInterrupt());
}

TEST_F(BMM350Test, EnableInterruptNullPinFails) {
    expectFullInit();
    BMM350 mag(bus, delay);
    ASSERT_TRUE(mag.init());

    EXPECT_FALSE(mag.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}
