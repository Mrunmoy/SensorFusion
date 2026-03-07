#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "SHT40.hpp"
#include "Crc8Sensirion.hpp"
#include <type_traits>

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;

static constexpr uint8_t ADDR = 0x44;

static_assert(std::is_base_of<IHumiditySensor, SHT40>::value,
              "SHT40 must satisfy humidity middleware interface");

// Helper: compute CRC for a 2-byte word
static uint8_t crc2(uint8_t a, uint8_t b) {
    uint8_t data[2] = {a, b};
    return crc8Sensirion(data, 2);
}

class SHT40Test : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    // Expect a successful init (soft reset + serial read)
    void expectInit() {
        // Soft reset command
        EXPECT_CALL(bus, rawWrite(ADDR, _, 1))
            .WillOnce([](uint8_t, const uint8_t* data, size_t) {
                EXPECT_EQ(data[0], 0x94);
                return true;
            })
            // Serial read command
            .WillOnce([](uint8_t, const uint8_t* data, size_t) {
                EXPECT_EQ(data[0], 0x89);
                return true;
            });
        EXPECT_CALL(delay, delayMs(1)).Times(2);

        // Serial read response: 6 bytes with valid CRCs
        EXPECT_CALL(bus, rawRead(ADDR, _, 6))
            .WillOnce([](uint8_t, uint8_t* buf, size_t) {
                buf[0] = 0x12; buf[1] = 0x34; buf[2] = crc2(0x12, 0x34);
                buf[3] = 0x56; buf[4] = 0x78; buf[5] = crc2(0x56, 0x78);
                return true;
            });
    }
};

TEST_F(SHT40Test, InitSuccess) {
    expectInit();
    SHT40 sensor(bus, delay);
    EXPECT_TRUE(sensor.init());
}

TEST_F(SHT40Test, InitFailsBusError) {
    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(false));
    SHT40 sensor(bus, delay);
    EXPECT_FALSE(sensor.init());
}

TEST_F(SHT40Test, ReadSerial) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(1));
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0xAB; buf[1] = 0xCD; buf[2] = crc2(0xAB, 0xCD);
            buf[3] = 0xEF; buf[4] = 0x01; buf[5] = crc2(0xEF, 0x01);
            return true;
        });

    uint32_t serial;
    ASSERT_TRUE(sensor.readSerial(serial));
    EXPECT_EQ(serial, 0xABCDEF01u);
}

TEST_F(SHT40Test, MeasureHighPrecision) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // Expect high precision command (0xFD)
    EXPECT_CALL(bus, rawWrite(ADDR, _, 1))
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0xFD);
            return true;
        });
    EXPECT_CALL(delay, delayMs(9));

    // Response: temp raw = 0x6666, hum raw = 0x6666
    // temp = -45 + 175 * (0x6666 / 65535) = -45 + 175 * 0.4 = 25.0
    // hum = -6 + 125 * (0x6666 / 65535) = -6 + 125 * 0.4 = 44.0
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = crc2(0x66, 0x66);
            return true;
        });

    float temp, hum;
    ASSERT_TRUE(sensor.measure(temp, hum));
    EXPECT_NEAR(temp, 25.003f, 0.1f);
    EXPECT_NEAR(hum, 44.004f, 0.1f);
}

TEST_F(SHT40Test, ReadHumidityPercentUsesMeasurementPath) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(9));
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = crc2(0x66, 0x66);
            return true;
        });

    float hum = 0.0f;
    ASSERT_TRUE(sensor.readHumidityPercent(hum));
    EXPECT_NEAR(hum, 44.004f, 0.1f);
}

TEST_F(SHT40Test, MeasureMediumPrecision) {
    SHT40Config cfg;
    cfg.precision = Sht40Precision::MEDIUM;

    // Init
    EXPECT_CALL(bus, rawWrite(ADDR, _, 1))
        .WillOnce(Return(true))   // soft reset
        .WillOnce(Return(true))   // serial read cmd
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0xF6);  // medium precision
            return true;
        });
    EXPECT_CALL(delay, delayMs(1)).Times(2);
    EXPECT_CALL(delay, delayMs(5));

    // Serial response
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x12; buf[1] = 0x34; buf[2] = crc2(0x12, 0x34);
            buf[3] = 0x56; buf[4] = 0x78; buf[5] = crc2(0x56, 0x78);
            return true;
        })
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = crc2(0x66, 0x66);
            return true;
        });

    SHT40 sensor(bus, delay, cfg);
    ASSERT_TRUE(sensor.init());

    float temp, hum;
    ASSERT_TRUE(sensor.measure(temp, hum));
}

TEST_F(SHT40Test, MeasureLowPrecision) {
    SHT40Config cfg;
    cfg.precision = Sht40Precision::LOW;

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1))
        .WillOnce(Return(true))
        .WillOnce(Return(true))
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0xE0);  // low precision
            return true;
        });
    EXPECT_CALL(delay, delayMs(1)).Times(2);
    EXPECT_CALL(delay, delayMs(2));

    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x12; buf[1] = 0x34; buf[2] = crc2(0x12, 0x34);
            buf[3] = 0x56; buf[4] = 0x78; buf[5] = crc2(0x56, 0x78);
            return true;
        })
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = crc2(0x66, 0x66);
            return true;
        });

    SHT40 sensor(bus, delay, cfg);
    ASSERT_TRUE(sensor.init());

    float temp, hum;
    ASSERT_TRUE(sensor.measure(temp, hum));
}

TEST_F(SHT40Test, MeasureCrcFailTemp) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(9));
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = 0x00; // bad CRC
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = crc2(0x66, 0x66);
            return true;
        });

    float temp, hum;
    EXPECT_FALSE(sensor.measure(temp, hum));
}

TEST_F(SHT40Test, MeasureCrcFailHum) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(9));
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x66; buf[4] = 0x66; buf[5] = 0x00; // bad CRC
            return true;
        });

    float temp, hum;
    EXPECT_FALSE(sensor.measure(temp, hum));
}

TEST_F(SHT40Test, HumidityClampedTo0) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(9));

    // raw hum = 0x0000 → -6 + 125*(0/65535) = -6 → clamped to 0
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0x00; buf[4] = 0x00; buf[5] = crc2(0x00, 0x00);
            return true;
        });

    float temp, hum;
    ASSERT_TRUE(sensor.measure(temp, hum));
    EXPECT_GE(hum, 0.0f);
}

TEST_F(SHT40Test, HumidityClampedTo100) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(9));

    // raw hum = 0xFFFF → -6 + 125*(65535/65535) = 119 → clamped to 100
    EXPECT_CALL(bus, rawRead(ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x66; buf[1] = 0x66; buf[2] = crc2(0x66, 0x66);
            buf[3] = 0xFF; buf[4] = 0xFF; buf[5] = crc2(0xFF, 0xFF);
            return true;
        });

    float temp, hum;
    ASSERT_TRUE(sensor.measure(temp, hum));
    EXPECT_LE(hum, 100.0f);
}

TEST_F(SHT40Test, MeasureBusFail) {
    expectInit();
    SHT40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 1)).WillOnce(Return(false));

    float temp, hum;
    EXPECT_FALSE(sensor.measure(temp, hum));
}

TEST_F(SHT40Test, CustomAddress) {
    constexpr uint8_t ALT_ADDR = 0x45;

    EXPECT_CALL(bus, rawWrite(ALT_ADDR, _, 1))
        .WillOnce(Return(true))   // soft reset
        .WillOnce(Return(true));  // serial read cmd
    EXPECT_CALL(delay, delayMs(1)).Times(2);
    EXPECT_CALL(bus, rawRead(ALT_ADDR, _, 6))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x12; buf[1] = 0x34; buf[2] = crc2(0x12, 0x34);
            buf[3] = 0x56; buf[4] = 0x78; buf[5] = crc2(0x56, 0x78);
            return true;
        });

    SHT40Config cfg;
    cfg.address = ALT_ADDR;
    SHT40 sensor(bus, delay, cfg);
    EXPECT_TRUE(sensor.init());
}

TEST_F(SHT40Test, SizeofIsSmall) {
    EXPECT_LE(sizeof(SHT40), 32);
}
