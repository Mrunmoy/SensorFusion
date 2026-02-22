#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "BMP180.hpp"
#include <cmath>

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;

static constexpr uint8_t ADDR         = 0x77;
static constexpr uint8_t REG_CALIB    = 0xAA;
static constexpr uint8_t REG_CHIP_ID  = 0xD0;
static constexpr uint8_t REG_CTRL     = 0xF4;
static constexpr uint8_t REG_OUT_MSB  = 0xF6;

// BMP180 datasheet example calibration values
// AC1=408, AC2=-72, AC3=-14383, AC4=32741, AC5=32757, AC6=23153
// B1=6190, B2=4, MB=-32768, MC=-8711, MD=2868
static void fillDatasheetCalib(uint8_t* buf) {
    auto put16 = [&](int idx, int16_t val) {
        buf[idx * 2]     = static_cast<uint8_t>(val >> 8);
        buf[idx * 2 + 1] = static_cast<uint8_t>(val & 0xFF);
    };
    put16(0, 408);     // AC1
    put16(1, -72);     // AC2
    put16(2, -14383);  // AC3
    // AC4, AC5, AC6 are unsigned but stored the same way
    put16(3, static_cast<int16_t>(32741));  // AC4
    put16(4, static_cast<int16_t>(32757));  // AC5
    put16(5, static_cast<int16_t>(23153));  // AC6
    put16(6, 6190);    // B1
    put16(7, 4);       // B2
    put16(8, -32768);  // MB
    put16(9, -8711);   // MC
    put16(10, 2868);   // MD
}

class BMP180Test : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectChipId(uint8_t id) {
        EXPECT_CALL(bus, readRegister(ADDR, REG_CHIP_ID, _, 1))
            .WillOnce([id](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = id;
                return true;
            });
    }

    void expectCalibRead() {
        EXPECT_CALL(bus, readRegister(ADDR, REG_CALIB, _, 22))
            .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
                fillDatasheetCalib(buf);
                return true;
            });
    }

    void expectInit() {
        expectChipId(0x55);
        expectCalibRead();
    }

    // Helper: expect a temp-only measurement sequence
    void expectTempOnlyMeasurement(uint16_t rawTemp) {
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL, _, 1))
            .WillOnce(Return(true));
        EXPECT_CALL(delay, delayMs(_));
        EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_MSB, _, 2))
            .WillOnce([rawTemp](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = static_cast<uint8_t>(rawTemp >> 8);
                buf[1] = static_cast<uint8_t>(rawTemp & 0xFF);
                return true;
            });
    }

    // Helper: expect temp + pressure measurement sequence (two CTRL writes)
    void expectTempThenPressure(uint16_t rawTemp, int32_t rawPressBytes) {
        // Both temp and pressure write to CTRL and read from OUT_MSB
        EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL, _, 1))
            .WillOnce(Return(true))   // temp command
            .WillOnce(Return(true));  // pressure command
        EXPECT_CALL(delay, delayMs(_))
            .Times(2);
        EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_MSB, _, 2))
            .WillOnce([rawTemp](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = static_cast<uint8_t>(rawTemp >> 8);
                buf[1] = static_cast<uint8_t>(rawTemp & 0xFF);
                return true;
            });
        EXPECT_CALL(bus, readRegister(ADDR, REG_OUT_MSB, _, 3))
            .WillOnce([rawPressBytes](uint8_t, uint8_t, uint8_t* buf, size_t) {
                buf[0] = static_cast<uint8_t>((rawPressBytes >> 16) & 0xFF);
                buf[1] = static_cast<uint8_t>((rawPressBytes >> 8) & 0xFF);
                buf[2] = static_cast<uint8_t>(rawPressBytes & 0xFF);
                return true;
            });
    }
};

TEST_F(BMP180Test, InitSuccess) {
    expectInit();
    BMP180 baro(bus, delay);
    EXPECT_TRUE(baro.init());
}

TEST_F(BMP180Test, InitWrongChipId) {
    expectChipId(0xAA);
    BMP180 baro(bus, delay);
    EXPECT_FALSE(baro.init());
}

TEST_F(BMP180Test, InitCalibReadFail) {
    expectChipId(0x55);
    EXPECT_CALL(bus, readRegister(ADDR, REG_CALIB, _, 22))
        .WillOnce(Return(false));
    BMP180 baro(bus, delay);
    EXPECT_FALSE(baro.init());
}

TEST_F(BMP180Test, ReadTemperature) {
    // Datasheet example: UT=27898 → T=15.0°C (150 in 0.1°C)
    expectInit();
    BMP180 baro(bus, delay);
    ASSERT_TRUE(baro.init());

    expectTempOnlyMeasurement(27898);

    float temp;
    ASSERT_TRUE(baro.readTemperature(temp));
    EXPECT_NEAR(temp, 15.0f, 0.1f);
}

TEST_F(BMP180Test, ReadPressure) {
    // Datasheet example: UT=27898, UP=23843 (OSS=0) → P=69964 Pa
    BMP180Config cfg;
    cfg.oss = BMP180Oss::ULTRA_LOW;

    expectInit();
    BMP180 baro(bus, delay, cfg);
    ASSERT_TRUE(baro.init());

    // readPressure reads temp first then pressure
    // Raw bytes: UP=23843 at oss=0, need (MSB<<16|LSB<<8|XLSB)>>8 = 23843
    // So raw 3 bytes = 0x5D, 0x23, 0x00
    expectTempThenPressure(27898, 0x5D2300);

    int32_t pressure;
    ASSERT_TRUE(baro.readPressure(pressure));
    EXPECT_NEAR(pressure, 69964, 5);
}

TEST_F(BMP180Test, ReadTempAndPressure) {
    BMP180Config cfg;
    cfg.oss = BMP180Oss::ULTRA_LOW;

    expectInit();
    BMP180 baro(bus, delay, cfg);
    ASSERT_TRUE(baro.init());

    expectTempThenPressure(27898, 0x5D2300);

    float temp;
    int32_t pressure;
    ASSERT_TRUE(baro.readTempAndPressure(temp, pressure));
    EXPECT_NEAR(temp, 15.0f, 0.1f);
    EXPECT_NEAR(pressure, 69964, 5);
}

TEST_F(BMP180Test, AltitudeCalculation) {
    // Sea level: 101325 Pa → altitude ≈ 0 m
    EXPECT_NEAR(BMP180::pressureToAltitude(101325), 0.0f, 1.0f);
    // ~900 hPa → ~1000m (approximate)
    float alt = BMP180::pressureToAltitude(90000);
    EXPECT_GT(alt, 800.0f);
    EXPECT_LT(alt, 1200.0f);
}

TEST_F(BMP180Test, ReadTempBusFail) {
    expectInit();
    BMP180 baro(bus, delay);
    ASSERT_TRUE(baro.init());

    // Trigger command write fails
    EXPECT_CALL(bus, writeRegister(ADDR, REG_CTRL, _, 1))
        .WillOnce(Return(false));

    float temp;
    EXPECT_FALSE(baro.readTemperature(temp));
}
