#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "ADXL345.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR         = 0x53;
static constexpr uint8_t REG_DEVID    = 0x00;
static constexpr uint8_t REG_BW_RATE  = 0x2C;
static constexpr uint8_t REG_PWR_CTL  = 0x2D;
static constexpr uint8_t REG_DATA_FMT = 0x31;
static constexpr uint8_t REG_DATAX0   = 0x32;

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

    // Full-res: 4 mg/LSB = 0.004 g/LSB
    // X=250 → 1.0g, Y=0, Z=0
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATAX0, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 250 = 0x00FA (LE: 0xFA, 0x00)
            buf[0] = 0xFA; buf[1] = 0x00;  // X
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

    // -250 LSB → -1.0g
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATAX0, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // -250 = 0xFF06 (LE: 0x06, 0xFF)
            buf[0] = 0x06; buf[1] = 0xFF;  // X = -250
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
