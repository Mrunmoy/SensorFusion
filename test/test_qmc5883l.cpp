#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "QMC5883L.hpp"
#include <cmath>

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR       = 0x0D;
static constexpr uint8_t REG_DATA   = 0x00;
static constexpr uint8_t REG_STATUS = 0x06;
static constexpr uint8_t REG_CTRL1  = 0x09;
static constexpr uint8_t REG_CTRL2  = 0x0A;
static constexpr uint8_t REG_SETRST = 0x0B;

class QMC5883LTest : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectInit() {
        // Soft reset
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(10));
        // SET_RST
        EXPECT_CALL(bus, writeRegister(ADDR, REG_SETRST, _, 1)).WillOnce(Return(true));
        // CTRL1 config
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(5));
    }
};

TEST_F(QMC5883LTest, InitSuccess) {
    expectInit();
    QMC5883L mag(bus, delay);
    EXPECT_TRUE(mag.init());
}

TEST_F(QMC5883LTest, InitBusFail) {
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1))
        .WillOnce(Return(false));
    QMC5883L mag(bus, delay);
    EXPECT_FALSE(mag.init());
}

TEST_F(QMC5883LTest, IsDataReady) {
    EXPECT_CALL(bus, readRegister(ADDR, REG_STATUS, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x01; // DRDY set
            return true;
        });
    expectInit();
    QMC5883L mag(bus, delay);
    ASSERT_TRUE(mag.init());

    bool ready = false;
    ASSERT_TRUE(mag.isDataReady(ready));
    EXPECT_TRUE(ready);
}

TEST_F(QMC5883LTest, IsDataNotReady) {
    EXPECT_CALL(bus, readRegister(ADDR, REG_STATUS, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; // DRDY not set
            return true;
        });
    expectInit();
    QMC5883L mag(bus, delay);
    ASSERT_TRUE(mag.init());

    bool ready = true;
    ASSERT_TRUE(mag.isDataReady(ready));
    EXPECT_FALSE(ready);
}

TEST_F(QMC5883LTest, ReadRawLittleEndian) {
    // X = 0x0100 = 256, Y = 0xFF00 = -256 (signed), Z = 0x0001 = 1
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATA, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; buf[1] = 0x01;  // X LSB, MSB
            buf[2] = 0x00; buf[3] = 0xFF;  // Y LSB, MSB → -256
            buf[4] = 0x01; buf[5] = 0x00;  // Z LSB, MSB
            return true;
        });
    expectInit();
    QMC5883L mag(bus, delay);
    ASSERT_TRUE(mag.init());

    int16_t x, y, z;
    ASSERT_TRUE(mag.readRaw(x, y, z));
    EXPECT_EQ(x, 256);
    EXPECT_EQ(y, -256);
    EXPECT_EQ(z, 1);
}

TEST_F(QMC5883LTest, ReadMicroTesla8G) {
    // 8G range: 3000 LSB/Gauss = 300 LSB/µT
    // raw 300 → 1.0 µT
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATA, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 300 = 0x012C (LE: 0x2C, 0x01)
            buf[0] = 0x2C; buf[1] = 0x01;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    expectInit();
    QMC5883L mag(bus, delay); // default: 8G
    ASSERT_TRUE(mag.init());

    MagData m;
    ASSERT_TRUE(mag.readMicroTesla(m));
    EXPECT_NEAR(m.x, 1.0f, 0.01f);
    EXPECT_NEAR(m.y, 0.0f, 0.01f);
    EXPECT_NEAR(m.z, 0.0f, 0.01f);
}

TEST_F(QMC5883LTest, ReadMicroTesla2G) {
    // 2G range: 12000 LSB/Gauss = 1200 LSB/µT
    // raw 1200 → 1.0 µT
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATA, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 1200 = 0x04B0 (LE: 0xB0, 0x04)
            buf[0] = 0xB0; buf[1] = 0x04;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    QMC5883LConfig cfg;
    cfg.range = MagRange::GAUSS_2;

    expectInit();
    QMC5883L mag(bus, delay, cfg);
    ASSERT_TRUE(mag.init());

    MagData m;
    ASSERT_TRUE(mag.readMicroTesla(m));
    EXPECT_NEAR(m.x, 1.0f, 0.01f);
}

TEST_F(QMC5883LTest, HeadingDegrees) {
    QMC5883L mag(bus, delay);
    // Pointing North: mx=1, my=0 → 0°
    EXPECT_NEAR(mag.headingDegrees(1.0f, 0.0f), 0.0f, 0.1f);
    // Pointing East: mx=0, my=-1 → 90° (NED convention)
    EXPECT_NEAR(mag.headingDegrees(0.0f, -1.0f), 90.0f, 0.1f);
    // Pointing South: mx=-1, my=0 → 180°
    EXPECT_NEAR(mag.headingDegrees(-1.0f, 0.0f), 180.0f, 0.1f);
}

TEST_F(QMC5883LTest, ReadBusFail) {
    EXPECT_CALL(bus, readRegister(ADDR, REG_DATA, _, 6))
        .WillOnce(Return(false));
    expectInit();
    QMC5883L mag(bus, delay);
    ASSERT_TRUE(mag.init());

    MagData m;
    EXPECT_FALSE(mag.readMicroTesla(m));
}
