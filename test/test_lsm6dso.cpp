#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"
#include "LSM6DSO.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR          = 0x6A;
static constexpr uint8_t REG_INT1_CTRL = 0x0D;
static constexpr uint8_t REG_WHO_AM_I  = 0x0F;
static constexpr uint8_t REG_CTRL1_XL  = 0x10;
static constexpr uint8_t REG_CTRL2_G   = 0x11;
static constexpr uint8_t REG_CTRL3_C   = 0x12;
static constexpr uint8_t REG_TEMP_OUT  = 0x20;
static constexpr uint8_t REG_GYRO_OUT  = 0x22;
static constexpr uint8_t REG_ACCEL_OUT = 0x28;

class LSM6DSOTest : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectReset() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3_C, _, 1))
            .WillOnce(Return(true))   // reset
            .WillOnce(Return(true));  // BDU + IF_INC
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
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1_XL, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2_G, _, 1)).WillOnce(Return(true));
    }

    void expectFullInit() {
        expectReset();
        expectWhoAmI(0x6C);
        expectConfig();
    }
};

TEST_F(LSM6DSOTest, InitSuccess) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    EXPECT_TRUE(imu.init());
}

TEST_F(LSM6DSOTest, InitWrongId) {
    // Reset writes CTRL3_C once (fails before BDU)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3_C, _, 1))
        .WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(10));
    expectWhoAmI(0xFF);
    LSM6DSO imu(bus, delay);
    EXPECT_FALSE(imu.init());
}

TEST_F(LSM6DSOTest, InitBusFail) {
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3_C, _, 1))
        .WillOnce(Return(false));
    LSM6DSO imu(bus, delay);
    EXPECT_FALSE(imu.init());
}

TEST_F(LSM6DSOTest, ReadAccelDefaultRange) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    // ±2g: 0.061 mg/LSB → 1g = 16393.4 LSB ≈ 16393
    // 16393 = 0x4009 → LE: 0x09, 0x40
    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x09; buf[1] = 0x40;  // X
            buf[2] = 0x00; buf[3] = 0x00;  // Y
            buf[4] = 0x00; buf[5] = 0x00;  // Z
            return true;
        });

    AccelData a;
    ASSERT_TRUE(imu.readAccel(a));
    EXPECT_NEAR(a.x, 1.0f, 0.01f);
    EXPECT_NEAR(a.y, 0.0f, 0.001f);
}

TEST_F(LSM6DSOTest, ReadAccel16G) {
    LSM6DSOConfig cfg;
    cfg.accelRange = LsmAccelRange::G16;

    expectFullInit();
    LSM6DSO imu(bus, delay, cfg);
    ASSERT_TRUE(imu.init());

    // ±16g: 0.488 mg/LSB → 1g = 2049.2 LSB ≈ 2049
    // 2049 = 0x0801 → LE: 0x01, 0x08
    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x01; buf[1] = 0x08;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    AccelData a;
    ASSERT_TRUE(imu.readAccel(a));
    EXPECT_NEAR(a.x, 1.0f, 0.01f);
}

TEST_F(LSM6DSOTest, ReadGyroDefaultRange) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    // ±250: 8.75 mdps/LSB → 100 dps = 11428.6 LSB ≈ 11429
    // 11429 = 0x2CA5 → LE: 0xA5, 0x2C
    EXPECT_CALL(bus, readRegister(ADDR, REG_GYRO_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xA5; buf[1] = 0x2C;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    GyroData g;
    ASSERT_TRUE(imu.readGyro(g));
    EXPECT_NEAR(g.x, 100.0f, 0.1f);
}

TEST_F(LSM6DSOTest, ReadGyro2000Dps) {
    LSM6DSOConfig cfg;
    cfg.gyroRange = LsmGyroRange::DPS2000;

    expectFullInit();
    LSM6DSO imu(bus, delay, cfg);
    ASSERT_TRUE(imu.init());

    // ±2000: 70.0 mdps/LSB → 100 dps = 1428.6 LSB ≈ 1429
    // 1429 = 0x0595 → LE: 0x95, 0x05
    EXPECT_CALL(bus, readRegister(ADDR, REG_GYRO_OUT, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x95; buf[1] = 0x05;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    GyroData g;
    ASSERT_TRUE(imu.readGyro(g));
    EXPECT_NEAR(g.x, 100.0f, 0.1f);
}

TEST_F(LSM6DSOTest, ReadTemperature) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    // Formula: raw/256.0 + 25.0; raw=0 → 25°C
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_OUT, _, 2))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; buf[1] = 0x00;
            return true;
        });

    float tempC;
    ASSERT_TRUE(imu.readTemperature(tempC));
    EXPECT_NEAR(tempC, 25.0f, 0.01f);
}

TEST_F(LSM6DSOTest, ReadAllBurst) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    // 14 bytes from OUT_TEMP_L: temp(2) + gyro(6) + accel(6)
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_OUT, _, 14))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // Temp = 0 → 25°C
            buf[0] = 0x00; buf[1] = 0x00;
            // Gyro X=0, Y=0, Z=0
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            buf[6] = 0x00; buf[7] = 0x00;
            // Accel X=16393(1g), Y=0, Z=0
            buf[8] = 0x09; buf[9] = 0x40;
            buf[10] = 0x00; buf[11] = 0x00;
            buf[12] = 0x00; buf[13] = 0x00;
            return true;
        });

    AccelData a;
    GyroData g;
    float tempC;
    ASSERT_TRUE(imu.readAll(a, g, tempC));
    EXPECT_NEAR(a.x, 1.0f, 0.01f);
    EXPECT_NEAR(g.x, 0.0f, 0.001f);
    EXPECT_NEAR(tempC, 25.0f, 0.01f);
}

TEST_F(LSM6DSOTest, ReadAccelBusFail) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_ACCEL_OUT, _, 6))
        .WillOnce(Return(false));
    AccelData a;
    EXPECT_FALSE(imu.readAccel(a));
}

TEST_F(LSM6DSOTest, EnableDataReadyInterrupt) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT1_CTRL, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x03); // DRDY_XL + DRDY_G
            return true;
        });

    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(imu.enableDataReadyInterrupt(&intPin, cb, &called));

    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(LSM6DSOTest, DisableDataReadyInterrupt) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT1_CTRL, _, 1))
        .WillOnce(Return(true))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x00);
            return true;
        });
    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(imu.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(imu.disableDataReadyInterrupt());
}

TEST_F(LSM6DSOTest, EnableInterruptBusFail) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    MockGpioInterrupt intPin;
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT1_CTRL, _, 1))
        .WillOnce(Return(false));

    EXPECT_FALSE(imu.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
}

TEST_F(LSM6DSOTest, EnableInterruptNullPinFails) {
    expectFullInit();
    LSM6DSO imu(bus, delay);
    ASSERT_TRUE(imu.init());

    EXPECT_FALSE(imu.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}
