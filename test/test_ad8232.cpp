#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockAdcChannel.hpp"
#include "MockDelayProvider.hpp"
#include "AD8232.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

class AD8232Test : public ::testing::Test {
protected:
    MockAdcChannel adc;
    MockDelayProvider delay;
};

TEST_F(AD8232Test, ReadRawReturnsAdcValue) {
    EXPECT_CALL(adc, readRaw(_))
        .WillOnce([](int32_t& out) { out = 2048; return true; });
    AD8232 ecg(adc, delay);
    int32_t val;
    EXPECT_TRUE(ecg.readRaw(val));
    EXPECT_EQ(val, 2048);
}

TEST_F(AD8232Test, ReadMillivoltsReturnsCalibrated) {
    EXPECT_CALL(adc, readMillivolts(_))
        .WillOnce([](int32_t& out) { out = 1650; return true; });
    AD8232 ecg(adc, delay);
    int32_t mv;
    EXPECT_TRUE(ecg.readMillivolts(mv));
    EXPECT_EQ(mv, 1650);
}

TEST_F(AD8232Test, SampleCombinesValueAndTimestamp) {
    EXPECT_CALL(adc, readMillivolts(_))
        .WillOnce([](int32_t& out) { out = 800; return true; });
    EXPECT_CALL(delay, getTimestampUs()).WillOnce(Return(123456ULL));

    AD8232 ecg(adc, delay);
    ECGSample s;
    EXPECT_TRUE(ecg.sample(s));
    EXPECT_EQ(s.millivolts, 800);
    EXPECT_EQ(s.timestampUs, 123456ULL);
}

TEST_F(AD8232Test, ReadRawBusFail) {
    EXPECT_CALL(adc, readRaw(_)).WillOnce(Return(false));
    AD8232 ecg(adc, delay);
    int32_t val;
    EXPECT_FALSE(ecg.readRaw(val));
}

TEST_F(AD8232Test, ReadMillivoltsBusFail) {
    EXPECT_CALL(adc, readMillivolts(_)).WillOnce(Return(false));
    AD8232 ecg(adc, delay);
    int32_t mv;
    EXPECT_FALSE(ecg.readMillivolts(mv));
}

TEST_F(AD8232Test, SampleFailsOnAdcError) {
    EXPECT_CALL(adc, readMillivolts(_)).WillOnce(Return(false));
    AD8232 ecg(adc, delay);
    ECGSample s;
    EXPECT_FALSE(ecg.sample(s));
}
