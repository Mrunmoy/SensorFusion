#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockAdcChannel.hpp"
#include "MockDelayProvider.hpp"
#include "MockGpioInterrupt.hpp"

using namespace sf::test;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArrayArgument;

class HalI2CTest : public ::testing::Test {
protected:
    MockI2CBus bus;
    static constexpr uint8_t ADDR = 0x68;
};

TEST_F(HalI2CTest, Read8DelegatesToReadRegister) {
    EXPECT_CALL(bus, readRegister(ADDR, 0x75, _, 1))
        .WillOnce([](uint8_t, uint8_t, uint8_t* buf, size_t) {
            buf[0] = 0x68;
            return true;
        });
    uint8_t val = 0;
    EXPECT_TRUE(bus.read8(ADDR, 0x75, val));
    EXPECT_EQ(val, 0x68);
}

TEST_F(HalI2CTest, Read8PropagatesFailure) {
    EXPECT_CALL(bus, readRegister(ADDR, 0x75, _, 1))
        .WillOnce(Return(false));
    uint8_t val = 0;
    EXPECT_FALSE(bus.read8(ADDR, 0x75, val));
}

TEST_F(HalI2CTest, Write8DelegatesToWriteRegister) {
    EXPECT_CALL(bus, writeRegister(ADDR, 0x6B, _, 1))
        .WillOnce(Return(true));
    EXPECT_TRUE(bus.write8(ADDR, 0x6B, 0x00));
}

TEST_F(HalI2CTest, ProbeForwardsToMock) {
    EXPECT_CALL(bus, probe(ADDR)).WillOnce(Return(true));
    EXPECT_TRUE(bus.probe(ADDR));
}

// --- ADC Channel Tests ---

TEST(HalAdcTest, ReadRawReturnsValue) {
    MockAdcChannel adc;
    EXPECT_CALL(adc, readRaw(_))
        .WillOnce(DoAll(::testing::SetArgReferee<0>(2048), Return(true)));
    int32_t val;
    EXPECT_TRUE(adc.readRaw(val));
    EXPECT_EQ(val, 2048);
}

TEST(HalAdcTest, ReadMillivoltsReturnsValue) {
    MockAdcChannel adc;
    EXPECT_CALL(adc, readMillivolts(_))
        .WillOnce(DoAll(::testing::SetArgReferee<0>(1650), Return(true)));
    int32_t mv;
    EXPECT_TRUE(adc.readMillivolts(mv));
    EXPECT_EQ(mv, 1650);
}

// --- Delay Provider Tests ---

TEST(HalDelayTest, DelayMsIsCalled) {
    MockDelayProvider delay;
    EXPECT_CALL(delay, delayMs(100));
    delay.delayMs(100);
}

TEST(HalDelayTest, GetTimestampUsReturnsValue) {
    MockDelayProvider delay;
    EXPECT_CALL(delay, getTimestampUs()).WillOnce(Return(123456789ULL));
    EXPECT_EQ(delay.getTimestampUs(), 123456789ULL);
}

// --- GPIO Interrupt Tests ---

TEST(HalGpioTest, EnableDisable) {
    MockGpioInterrupt gpio;
    EXPECT_CALL(gpio, enable(sf::GpioEdge::FALLING, _, _)).WillOnce(Return(true));
    EXPECT_CALL(gpio, disable()).WillOnce(Return(true));
    EXPECT_TRUE(gpio.enable(sf::GpioEdge::FALLING, nullptr, nullptr));
    EXPECT_TRUE(gpio.disable());
}

TEST(HalGpioTest, CaptureAndFireCallback) {
    MockGpioInterrupt gpio;
    gpio.captureCallback();

    bool fired = false;
    auto cb = [](void* ctx) { *static_cast<bool*>(ctx) = true; };

    EXPECT_TRUE(gpio.enable(sf::GpioEdge::FALLING, cb, &fired));
    EXPECT_FALSE(fired);
    gpio.fire();
    EXPECT_TRUE(fired);
}
