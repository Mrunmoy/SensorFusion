#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "SGP40.hpp"
#include "Crc8Sensirion.hpp"
#include <type_traits>

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

static constexpr uint8_t ADDR = 0x59;

static_assert(std::is_base_of<IVocSensor, SGP40>::value,
              "SGP40 must satisfy VOC middleware interface");

class SGP40Test : public ::testing::Test {
protected:
    MockI2CBus bus;
    MockDelayProvider delay;

    void expectInit() {
        // heaterOff: 2-byte command 0x3615
        EXPECT_CALL(bus, rawWrite(ADDR, _, 2))
            .WillOnce([](uint8_t, const uint8_t* data, size_t) {
                EXPECT_EQ(data[0], 0x36);
                EXPECT_EQ(data[1], 0x15);
                return true;
            });
    }

    // Provide a VOC response with valid CRC
    void expectVocResponse(uint16_t vocValue) {
        EXPECT_CALL(bus, rawRead(ADDR, _, 3))
            .WillOnce([vocValue](uint8_t, uint8_t* buf, size_t) {
                buf[0] = static_cast<uint8_t>(vocValue >> 8);
                buf[1] = static_cast<uint8_t>(vocValue & 0xFF);
                buf[2] = crc8Sensirion(buf, 2);
                return true;
            });
    }
};

TEST_F(SGP40Test, InitSuccess) {
    expectInit();
    SGP40 sensor(bus, delay);
    EXPECT_TRUE(sensor.init());
}

TEST_F(SGP40Test, InitFailsBusError) {
    EXPECT_CALL(bus, rawWrite(ADDR, _, 2)).WillOnce(Return(false));
    SGP40 sensor(bus, delay);
    EXPECT_FALSE(sensor.init());
}

TEST_F(SGP40Test, HeaterOff) {
    // Standalone heaterOff call
    EXPECT_CALL(bus, rawWrite(ADDR, _, 2))
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x36);
            EXPECT_EQ(data[1], 0x15);
            return true;
        });
    SGP40 sensor(bus, delay);
    EXPECT_TRUE(sensor.heaterOff());
}

TEST_F(SGP40Test, MeasureRawDefault) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // Expect 8-byte measure command with default compensation
    EXPECT_CALL(bus, rawWrite(ADDR, _, 8))
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            // Command: 0x26, 0x0F
            EXPECT_EQ(data[0], 0x26);
            EXPECT_EQ(data[1], 0x0F);
            // Default humidity: 0x80, 0x00
            EXPECT_EQ(data[2], 0x80);
            EXPECT_EQ(data[3], 0x00);
            // CRC of 0x80, 0x00
            EXPECT_EQ(data[4], crc8Sensirion(&data[2], 2));
            // Default temperature: 0x66, 0x66
            EXPECT_EQ(data[5], 0x66);
            EXPECT_EQ(data[6], 0x66);
            // CRC of 0x66, 0x66
            EXPECT_EQ(data[7], crc8Sensirion(&data[5], 2));
            return true;
        });
    EXPECT_CALL(delay, delayMs(30));
    expectVocResponse(12345);

    uint16_t voc;
    ASSERT_TRUE(sensor.measureRaw(voc));
    EXPECT_EQ(voc, 12345);
}

TEST_F(SGP40Test, MeasureRawWithCompensation) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    // 50% RH, 25°C → humTicks = 50*65535/100 = 32767, tempTicks = (25+45)*65535/175 = 26214
    EXPECT_CALL(bus, rawWrite(ADDR, _, 8))
        .WillOnce([](uint8_t, const uint8_t* data, size_t) {
            EXPECT_EQ(data[0], 0x26);
            EXPECT_EQ(data[1], 0x0F);
            // Verify CRCs are correct
            EXPECT_EQ(data[4], crc8Sensirion(&data[2], 2));
            EXPECT_EQ(data[7], crc8Sensirion(&data[5], 2));
            return true;
        });
    EXPECT_CALL(delay, delayMs(30));
    expectVocResponse(5000);

    uint16_t voc;
    ASSERT_TRUE(sensor.measureRaw(voc, 50.0f, 25.0f));
    EXPECT_EQ(voc, 5000);
}

TEST_F(SGP40Test, ReadVocRawUsesDefaultCompensation) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(30));
    expectVocResponse(4242);

    uint16_t voc = 0;
    ASSERT_TRUE(sensor.readVocRaw(voc));
    EXPECT_EQ(voc, 4242);
}

TEST_F(SGP40Test, MeasureRawCrcFail) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(30));

    // Response with bad CRC
    EXPECT_CALL(bus, rawRead(ADDR, _, 3))
        .WillOnce([](uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x30; buf[1] = 0x39; buf[2] = 0x00; // bad CRC
            return true;
        });

    uint16_t voc;
    EXPECT_FALSE(sensor.measureRaw(voc));
}

TEST_F(SGP40Test, MeasureRawBusFail) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(false));

    uint16_t voc;
    EXPECT_FALSE(sensor.measureRaw(voc));
}

TEST_F(SGP40Test, MeasureRawReadFail) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(30));
    EXPECT_CALL(bus, rawRead(ADDR, _, 3)).WillOnce(Return(false));

    uint16_t voc;
    EXPECT_FALSE(sensor.measureRaw(voc));
}

TEST_F(SGP40Test, MeasureRawZeroValue) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(30));
    expectVocResponse(0);

    uint16_t voc;
    ASSERT_TRUE(sensor.measureRaw(voc));
    EXPECT_EQ(voc, 0);
}

TEST_F(SGP40Test, MeasureRawMaxValue) {
    expectInit();
    SGP40 sensor(bus, delay);
    ASSERT_TRUE(sensor.init());

    EXPECT_CALL(bus, rawWrite(ADDR, _, 8)).WillOnce(Return(true));
    EXPECT_CALL(delay, delayMs(30));
    expectVocResponse(65535);

    uint16_t voc;
    ASSERT_TRUE(sensor.measureRaw(voc));
    EXPECT_EQ(voc, 65535);
}

TEST_F(SGP40Test, SizeofIsSmall) {
    EXPECT_LE(sizeof(SGP40), 32);
}
