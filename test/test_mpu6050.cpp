#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"
#include "MPU6050.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;
using ::testing::InSequence;

// MPU6050 register addresses
static constexpr uint8_t ADDR        = 0x68;
static constexpr uint8_t REG_SMPLRT  = 0x19;
static constexpr uint8_t REG_CONFIG  = 0x1A;
static constexpr uint8_t REG_GYRO_CFG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CFG = 0x1C;
static constexpr uint8_t REG_INT_PIN   = 0x37;
static constexpr uint8_t REG_INT_EN    = 0x38;
static constexpr uint8_t REG_INT_STAT  = 0x3A;
static constexpr uint8_t REG_ACCEL_OUT = 0x3B;
static constexpr uint8_t REG_GYRO_OUT  = 0x43;
static constexpr uint8_t REG_TEMP_OUT  = 0x41;
static constexpr uint8_t REG_USER_CTRL = 0x6A;
static constexpr uint8_t REG_PWR_MGMT  = 0x6B;
static constexpr uint8_t REG_WHO_AM_I  = 0x75;

class MPU6050Test : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectReset() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_PWR_MGMT, _, 1))
            .WillOnce(Return(true))   // reset (0x80)
            .WillOnce(Return(true));  // clock source (0x01)
        EXPECT_CALL(delay, delayMs(100));
    }

    void expectWhoAmI(uint8_t id) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_WHO_AM_I, _, 1))
            .WillOnce([id](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = id;
                return true;
            });
    }

    void expectConfig() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_SMPLRT, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CONFIG, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_GYRO_CFG, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_ACCEL_CFG, _, 1)).WillOnce(Return(true));
    }

    void expectBypass() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_USER_CTRL, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_PIN, _, 1)).WillOnce(Return(true));
    }

    void expectFullInit() {
        expectReset();
        expectWhoAmI(0x68);
        expectConfig();
        expectBypass();
    }
};

TEST_F(MPU6050Test, InitSuccess) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    EXPECT_TRUE(imu.init());
}

TEST_F(MPU6050Test, InitWrongId) {
    expectReset();
    expectWhoAmI(0xFF);
    MPU6050 imu(bus, delay);
    EXPECT_FALSE(imu.init());
}

TEST_F(MPU6050Test, InitBusFail) {
    // First write (reset) fails
    EXPECT_CALL(bus, writeRegister(ADDR, REG_PWR_MGMT, _, 1))
        .WillOnce(Return(false));
    MPU6050 imu(bus, delay);
    EXPECT_FALSE(imu.init());
}

TEST_F(MPU6050Test, ReadAccelConvertsToG) {
    // Raw value: X=16384 (1g), Y=-16384 (-1g), Z=0
    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 16384 = 0x4000
            buf[0] = 0x40; buf[1] = 0x00;  // X high, low
            buf[2] = 0xC0; buf[3] = 0x00;  // Y = -16384 = 0xC000
            buf[4] = 0x00; buf[5] = 0x00;  // Z = 0
            return true;
        });

    // Default config: ±2g → 16384 LSB/g
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    AccelData a;
    ASSERT_TRUE(imu.readAccel(a));
    EXPECT_NEAR(a.x, 1.0f, 0.001f);
    EXPECT_NEAR(a.y, -1.0f, 0.001f);
    EXPECT_NEAR(a.z, 0.0f, 0.001f);
}

TEST_F(MPU6050Test, ReadGyroConvertsToDps) {
    // Raw value: X=131 (1 dps at ±250), Y=0, Z=-131
    EXPECT_CALL(bus, readRegister(ADDR, REG_GYRO_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 131 = 0x0083
            buf[0] = 0x00; buf[1] = 0x83;  // X
            buf[2] = 0x00; buf[3] = 0x00;  // Y
            buf[4] = 0xFF; buf[5] = 0x7D;  // Z = -131 = 0xFF7D
            return true;
        });

    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    GyroData g;
    ASSERT_TRUE(imu.readGyro(g));
    EXPECT_NEAR(g.x, 1.0f, 0.01f);
    EXPECT_NEAR(g.y, 0.0f, 0.01f);
    EXPECT_NEAR(g.z, -1.0f, 0.01f);
}

TEST_F(MPU6050Test, ReadTemperature) {
    // Formula: temp = raw / 340.0 + 36.53
    // raw = 0 → 36.53°C
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_OUT, _, 2))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; buf[1] = 0x00;
            return true;
        });

    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    float temp;
    ASSERT_TRUE(imu.readTemperature(temp));
    EXPECT_NEAR(temp, 36.53f, 0.01f);
}

TEST_F(MPU6050Test, ReadAllReturnsBurstData) {
    // readAll reads 14 bytes from 0x3B: accel(6) + temp(2) + gyro(6)
    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 14))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // Accel X=16384(1g), Y=0, Z=0
            buf[0] = 0x40; buf[1] = 0x00;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            // Temp = 0 → 36.53
            buf[6] = 0x00; buf[7] = 0x00;
            // Gyro X=0, Y=0, Z=131(1dps)
            buf[8] = 0x00; buf[9] = 0x00;
            buf[10] = 0x00; buf[11] = 0x00;
            buf[12] = 0x00; buf[13] = 0x83;
            return true;
        });

    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    AccelData a;
    GyroData g;
    float temp;
    ASSERT_TRUE(imu.readAll(a, g, temp));
    EXPECT_NEAR(a.x, 1.0f, 0.001f);
    EXPECT_NEAR(g.z, 1.0f, 0.01f);
    EXPECT_NEAR(temp, 36.53f, 0.01f);
}

TEST_F(MPU6050Test, AccelRange4G) {
    // ±4g → 8192 LSB/g
    MPU6050Config cfg;
    cfg.accelRange = AccelRange::G4;

    expectReset();
    expectWhoAmI(0x68);
    expectConfig();
    expectBypass();

    MPU6050 imu(bus, delay, cfg);
    ASSERT_TRUE(imu.init());

    // 8192 raw = 1g at ±4g
    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x20; buf[1] = 0x00;  // 8192 = 0x2000
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    AccelData a;
    ASSERT_TRUE(imu.readAccel(a));
    EXPECT_NEAR(a.x, 1.0f, 0.001f);
}

TEST_F(MPU6050Test, GyroRange2000Dps) {
    // ±2000 → 16.4 LSB/(deg/s)
    MPU6050Config cfg;
    cfg.gyroRange = GyroRange::DPS2000;

    expectReset();
    expectWhoAmI(0x68);
    expectConfig();
    expectBypass();

    MPU6050 imu(bus, delay, cfg);
    ASSERT_TRUE(imu.init());

    // 164 raw ≈ 10 dps at ±2000
    EXPECT_CALL(bus, readRegister(ADDR, REG_GYRO_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 164 = 0x00A4
            buf[0] = 0x00; buf[1] = 0xA4;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    GyroData g;
    ASSERT_TRUE(imu.readGyro(g));
    EXPECT_NEAR(g.x, 10.0f, 0.1f);
}

TEST_F(MPU6050Test, NoBypassMode) {
    MPU6050Config cfg;
    cfg.i2cBypass = false;

    expectReset();
    expectWhoAmI(0x68);
    expectConfig();
    // No bypass writes expected

    MPU6050 imu(bus, delay, cfg);
    EXPECT_TRUE(imu.init());
}

TEST_F(MPU6050Test, ReadAccelBusFail) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce(Return(false));
    AccelData a;
    EXPECT_FALSE(imu.readAccel(a));
}

// --- Interrupt tests ---

TEST_F(MPU6050Test, EnableDataReadyInterrupt) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    // Expect INT_PIN_CFG write (active low, latch, clear-on-read, preserve bypass)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_PIN, _, 1))
        .WillOnce(Return(true));
    // Expect INT_ENABLE = 0x01 (DATA_RDY_EN)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_EN, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x01);
            return true;
        });

    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(imu.enableDataReadyInterrupt(&intPin, cb, &called));

    // Simulate interrupt firing
    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(MPU6050Test, DisableDataReadyInterrupt) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    // Enable first
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_PIN, _, 1))
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_EN, _, 1))
        .WillOnce(Return(true))  // enable
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x00);  // disable
            return true;
        });
    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(imu.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(imu.disableDataReadyInterrupt());
}

TEST_F(MPU6050Test, ClearInterrupt) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_INT_STAT, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x01; // DATA_RDY_INT
            return true;
        });

    uint8_t status = 0;
    EXPECT_TRUE(imu.clearInterrupt(status));
    EXPECT_EQ(status & 0x01, 0x01);
}

TEST_F(MPU6050Test, EnableInterruptBusFail) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_PIN, _, 1))
        .WillOnce(Return(false));

    EXPECT_FALSE(imu.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
}

TEST_F(MPU6050Test, EnableInterruptNullPinFails) {
    expectFullInit();
    MPU6050 imu(bus, delay);
    ASSERT_TRUE(imu.init());

    EXPECT_FALSE(imu.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}
