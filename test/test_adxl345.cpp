#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockGpioInterrupt.hpp"
#include "ADXL345.hpp"
#include <type_traits>

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR         = 0x53;
static constexpr uint8_t REG_DEVID    = 0x00;
static constexpr uint8_t REG_BW_RATE  = 0x2C;
static constexpr uint8_t REG_PWR_CTL  = 0x2D;
static constexpr uint8_t REG_INT_EN   = 0x2E;
static constexpr uint8_t REG_INT_MAP  = 0x2F;
static constexpr uint8_t REG_INT_SRC  = 0x30;
static constexpr uint8_t REG_DATA_FMT = 0x31;
static constexpr uint8_t REG_DATAX0   = 0x32;

static_assert(std::is_base_of<IAccelSensor, ADXL345>::value,
              "ADXL345 must satisfy accel middleware interface");

class ADXL345Test : public ::testing::Test {
protected:
    MockI2CBus bus;

    void expectDevId(uint8_t id) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_DEVID, _, 1))
            .WillOnce([id](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = id;
                return true;
            });
    }

    void expectConfig() {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_PWR_CTL, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_DATA_FMT, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_BW_RATE, _, 1)).WillOnce(Return(true));
    }

    void expectInit() {
        expectDevId(0xE5);
        expectConfig();
    }
};

TEST_F(ADXL345Test, InitSuccess) {
    expectInit();
    ADXL345 accel(bus);
    EXPECT_TRUE(accel.init());
}

TEST_F(ADXL345Test, InitWrongId) {
    expectDevId(0x00);
    ADXL345 accel(bus);
    EXPECT_FALSE(accel.init());
}

TEST_F(ADXL345Test, InitBusFail) {
    EXPECT_CALL(bus, readRegister(ADDR, REG_DEVID, _, 1))
        .WillOnce(Return(false));
    ADXL345 accel(bus);
    EXPECT_FALSE(accel.init());
}

TEST_F(ADXL345Test, ReadAccelConvertsToG) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    // Full-res: 3.9 mg/LSB
    // X=256 → ~0.998g, Y=0, Z=0
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATAX0, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 256 = 0x0100 (LE: 0x00, 0x01)
            buf[0] = 0x00; buf[1] = 0x01;  // X
            buf[2] = 0x00; buf[3] = 0x00;  // Y
            buf[4] = 0x00; buf[5] = 0x00;  // Z
            return true;
        });

    AccelData a;
    ASSERT_TRUE(accel.readAccel(a));
    EXPECT_NEAR(a.x, 1.0f, 0.01f);
    EXPECT_NEAR(a.y, 0.0f, 0.001f);
    EXPECT_NEAR(a.z, 0.0f, 0.001f);
}

TEST_F(ADXL345Test, ReadAccelNegative) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    // -256 LSB → ~-0.998g
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATAX0, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // -256 = 0xFF00 (LE: 0x00, 0xFF)
            buf[0] = 0x00; buf[1] = 0xFF;  // X = -256
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    AccelData a;
    ASSERT_TRUE(accel.readAccel(a));
    EXPECT_NEAR(a.x, -1.0f, 0.01f);
}

TEST_F(ADXL345Test, ReadAccelBusFail) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_DATAX0, _, 6))
        .WillOnce(Return(false));

    AccelData a;
    EXPECT_FALSE(accel.readAccel(a));
}

TEST_F(ADXL345Test, RangeConfig) {
    ADXL345Config cfg;
    cfg.range = AdxlRange::G16;

    expectDevId(0xE5);
    EXPECT_CALL(bus, writeRegister(ADDR, REG_PWR_CTL, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_DATA_FMT, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            // ±16g (0x03) + FULL_RES (0x08) = 0x0B
            EXPECT_EQ(data[0], 0x0B);
            return true;
        });
    EXPECT_CALL(bus, writeRegister(ADDR, REG_BW_RATE, _, 1)).WillOnce(Return(true));

    ADXL345 accel(bus, cfg);
    EXPECT_TRUE(accel.init());
}

// --- Interrupt tests ---

TEST_F(ADXL345Test, EnableDataReadyInterrupt) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    // Expect INT_MAP write (route DATA_READY to INT1)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_MAP, _, 1))
        .WillOnce(Return(true));
    // Expect INT_ENABLE = 0x80 (DATA_READY bit)
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_EN, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x80);
            return true;
        });

    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(accel.enableDataReadyInterrupt(&intPin, cb, &called));

    // Simulate interrupt firing
    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(ADXL345Test, DisableDataReadyInterrupt) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    // Enable first
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_MAP, _, 1))
        .WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_EN, _, 1))
        .WillOnce(Return(true))   // enable (0x80)
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x00);  // disable
            return true;
        });
    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(accel.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(accel.disableDataReadyInterrupt());
}

TEST_F(ADXL345Test, ClearInterrupt) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_INT_SRC, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x80; // DATA_READY
            return true;
        });

    uint8_t source = 0;
    EXPECT_TRUE(accel.clearInterrupt(source));
    EXPECT_EQ(source & 0x80, 0x80);
}

TEST_F(ADXL345Test, EnableInterruptBusFail) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    MockGpioInterrupt intPin;
    EXPECT_CALL(bus, writeRegister(ADDR, REG_INT_MAP, _, 1))
        .WillOnce(Return(false));

    EXPECT_FALSE(accel.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
}

TEST_F(ADXL345Test, EnableInterruptNullPinFails) {
    expectInit();
    ADXL345 accel(bus);
    ASSERT_TRUE(accel.init());

    EXPECT_FALSE(accel.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}
