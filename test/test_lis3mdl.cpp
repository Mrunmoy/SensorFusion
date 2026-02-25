#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"
#include "LIS3MDL.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR        = 0x1E;
static constexpr uint8_t REG_WHO     = 0x0F;
static constexpr uint8_t REG_CTRL1   = 0x20;
static constexpr uint8_t REG_CTRL2   = 0x21;
static constexpr uint8_t REG_CTRL3   = 0x22;
static constexpr uint8_t REG_CTRL4   = 0x23;
static constexpr uint8_t REG_CTRL5   = 0x24;
static constexpr uint8_t REG_OUT_X_L = 0x28;
static constexpr uint8_t REG_TEMP_L  = 0x2E;

class LIS3MDLTest : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectWhoAmI(uint8_t val = 0x3D) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_WHO, _, 1))
            .WillOnce([val](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = val;
                return true;
            });
    }

    void expectInit() {
        expectWhoAmI();
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL5, _, 1)).WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(5));
    }
};

TEST_F(LIS3MDLTest, InitSuccess) {
    expectInit();
    LIS3MDL mag(bus, delay);
    EXPECT_TRUE(mag.init());
}

TEST_F(LIS3MDLTest, InitFailsWrongWhoAmI) {
    expectWhoAmI(0x00);
    LIS3MDL mag(bus, delay);
    EXPECT_FALSE(mag.init());
}

TEST_F(LIS3MDLTest, InitFailsBusError) {
    EXPECT_CALL(bus, readRegister(ADDR, REG_WHO, _, 1))
        .WillOnce(Return(false));
    LIS3MDL mag(bus, delay);
    EXPECT_FALSE(mag.init());
}

TEST_F(LIS3MDLTest, InitCtrl1For80Hz) {
    expectWhoAmI();
    // 80 Hz: TEMP_EN=1, OM=11, DO=111, FAST_ODR=0 → 0x80|0x60|0x1C = 0xFC
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0xFC);
            return true;
        });
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL5, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(5));

    LIS3MDL mag(bus, delay);
    EXPECT_TRUE(mag.init());
}

TEST_F(LIS3MDLTest, InitCtrl1For10Hz) {
    expectWhoAmI();
    // 10 Hz (ODR=0x04): TEMP_EN=1, OM=11, DO=100 → 0x80|0x60|(0x04<<2) = 0xF0
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0xF0);
            return true;
        });
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL5, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(5));

    LIS3MDLConfig cfg;
    cfg.odr = Lis3mdlOdr::HZ_10;
    LIS3MDL mag(bus, delay, cfg);
    EXPECT_TRUE(mag.init());
}

TEST_F(LIS3MDLTest, InitCtrl2FullScale) {
    expectWhoAmI();
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL1, _, 1)).WillOnce(Return(true));
    // 16 gauss: FS=11 → (0x03 << 5) = 0x60
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL2, _, 1))
        .WillOnce([](uint8_t, uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x60);
            return true;
        });
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL3, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL4, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL5, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(5));

    LIS3MDLConfig cfg;
    cfg.scale = Lis3mdlScale::GAUSS_16;
    LIS3MDL mag(bus, delay, cfg);
    EXPECT_TRUE(mag.init());
}

TEST_F(LIS3MDLTest, ReadRawLittleEndian) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // X = 0x0100 = 256, Y = 0xFF00 = -256, Z = 0x0001 = 1
    EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_X_L, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x00; buf[1] = 0x01;  // X LSB, MSB
            buf[2] = 0x00; buf[3] = 0xFF;  // Y LSB, MSB → -256
            buf[4] = 0x01; buf[5] = 0x00;  // Z LSB, MSB
            return true;
        });

    int16_t x, y, z;
    ASSERT_TRUE(mag.readRaw(x, y, z));
    EXPECT_EQ(x, 256);
    EXPECT_EQ(y, -256);
    EXPECT_EQ(z, 1);
}

TEST_F(LIS3MDLTest, ReadMicroTesla4Gauss) {
    expectInit();
    LIS3MDL mag(bus, delay); // default: 4 gauss
    ASSERT_TRUE(mag.init());

    // 4G: 6842 LSB/Gauss → 1 Gauss = 6842 LSB → 100 uT
    // raw 6842 → 100.0 uT
    EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_X_L, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 6842 = 0x1ABA (LE: 0xBA, 0x1A)
            buf[0] = 0xBA; buf[1] = 0x1A;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    MagData m;
    ASSERT_TRUE(mag.readMicroTesla(m));
    EXPECT_NEAR(m.x, 100.0f, 0.1f);
    EXPECT_NEAR(m.y, 0.0f, 0.01f);
    EXPECT_NEAR(m.z, 0.0f, 0.01f);
}

TEST_F(LIS3MDLTest, ReadMicroTesla16Gauss) {
    LIS3MDLConfig cfg;
    cfg.scale = Lis3mdlScale::GAUSS_16;

    expectInit();
    LIS3MDL mag(bus, delay, cfg);
    ASSERT_TRUE(mag.init());

    // 16G: 1711 LSB/Gauss → raw 1711 → 100.0 uT
    EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_X_L, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 1711 = 0x06AF (LE: 0xAF, 0x06)
            buf[0] = 0xAF; buf[1] = 0x06;
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    MagData m;
    ASSERT_TRUE(mag.readMicroTesla(m));
    EXPECT_NEAR(m.x, 100.0f, 0.1f);
}

TEST_F(LIS3MDLTest, ReadMagInterface) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_X_L, _, 6))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xBA; buf[1] = 0x1A;  // 6842 → 100 uT
            buf[2] = 0x00; buf[3] = 0x00;
            buf[4] = 0x00; buf[5] = 0x00;
            return true;
        });

    IMagSensor& sensor = mag;
    MagData m;
    ASSERT_TRUE(sensor.readMag(m));
    EXPECT_NEAR(m.x, 100.0f, 0.1f);
}

TEST_F(LIS3MDLTest, ReadBusFail) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_X_L, _, 6))
        .WillOnce(Return(false));

    MagData m;
    EXPECT_FALSE(mag.readMicroTesla(m));
}

TEST_F(LIS3MDLTest, ReadTemperature) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // 8 LSB/degC. raw = 80 → 25 + 80/8 = 35°C
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_L, _, 2))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // 80 = 0x0050 (LE: 0x50, 0x00)
            buf[0] = 0x50; buf[1] = 0x00;
            return true;
        });

    float temp;
    ASSERT_TRUE(mag.readTemperature(temp));
    EXPECT_NEAR(temp, 35.0f, 0.1f);
}

TEST_F(LIS3MDLTest, ReadTemperatureNegative) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    // raw = -80 → 25 + (-80)/8 = 15°C
    EXPECT_CALL(bus, readRegister(ADDR, REG_TEMP_L, _, 2))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            // -80 = 0xFFB0 (LE: 0xB0, 0xFF)
            buf[0] = 0xB0; buf[1] = 0xFF;
            return true;
        });

    float temp;
    ASSERT_TRUE(mag.readTemperature(temp));
    EXPECT_NEAR(temp, 15.0f, 0.1f);
}

TEST_F(LIS3MDLTest, EnableDataReadyInterrupt) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    // DRDY needs no register config — just GPIO edge attachment
    bool called = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
    EXPECT_TRUE(mag.enableDataReadyInterrupt(&intPin, cb, &called));

    intPin.fire();
    EXPECT_TRUE(called);
}

TEST_F(LIS3MDLTest, DisableDataReadyInterrupt) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    MockGpioInterrupt intPin;
    intPin.captureCallback();

    EXPECT_CALL(intPin, disable()).WillOnce(Return(true));

    EXPECT_TRUE(mag.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
    EXPECT_TRUE(mag.disableDataReadyInterrupt());
}

TEST_F(LIS3MDLTest, EnableInterruptNullPinFails) {
    expectInit();
    LIS3MDL mag(bus, delay);
    ASSERT_TRUE(mag.init());

    EXPECT_FALSE(mag.enableDataReadyInterrupt(nullptr, nullptr, nullptr));
}

TEST_F(LIS3MDLTest, CustomAddress) {
    constexpr uint8_t ALT_ADDR = 0x1C;

    EXPECT_CALL(bus, readRegister(ALT_ADDR, REG_WHO, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x3D;
            return true;
        });
    EXPECT_CALL(bus, writeRegister(ALT_ADDR, _, _, 1))
        .Times(5)
        .WillRepeatedly(Return(true));
    EXPECT_CALL(delay, delayMs(5));

    LIS3MDLConfig cfg;
    cfg.address = ALT_ADDR;
    LIS3MDL mag(bus, delay, cfg);
    EXPECT_TRUE(mag.init());
}

TEST_F(LIS3MDLTest, SizeofIsSmall) {
    EXPECT_LE(sizeof(LIS3MDL), 40);
}
