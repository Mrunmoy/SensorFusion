#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockGpioInput.hpp"
#include "MockGpioOutput.hpp"
#include "BQ25101.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

class BQ25101Test : public ::testing::Test {
protected:
    MockGpioInput chgPin;
    MockGpioOutput tsPin;
};

TEST_F(BQ25101Test, StatusChargingWhenLow) {
    EXPECT_CALL(chgPin, read(_))
        .WillOnce([](bool& level) { level = false; return true; });

    BQ25101 charger(chgPin, tsPin);
    EXPECT_EQ(charger.status(), ChargeStatus::CHARGING);
}

TEST_F(BQ25101Test, StatusNotChargingWhenHigh) {
    EXPECT_CALL(chgPin, read(_))
        .WillOnce([](bool& level) { level = true; return true; });

    BQ25101 charger(chgPin, tsPin);
    EXPECT_EQ(charger.status(), ChargeStatus::NOT_CHARGING);
}

TEST_F(BQ25101Test, StatusUnknownOnReadFail) {
    EXPECT_CALL(chgPin, read(_)).WillOnce(Return(false));

    BQ25101 charger(chgPin, tsPin);
    EXPECT_EQ(charger.status(), ChargeStatus::UNKNOWN);
}

TEST_F(BQ25101Test, IsChargingTrue) {
    EXPECT_CALL(chgPin, read(_))
        .WillOnce([](bool& level) { level = false; return true; });

    BQ25101 charger(chgPin, tsPin);
    EXPECT_TRUE(charger.isCharging());
}

TEST_F(BQ25101Test, IsChargingFalseWhenNotCharging) {
    EXPECT_CALL(chgPin, read(_))
        .WillOnce([](bool& level) { level = true; return true; });

    BQ25101 charger(chgPin, tsPin);
    EXPECT_FALSE(charger.isCharging());
}

TEST_F(BQ25101Test, IsChargingFalseOnReadFail) {
    EXPECT_CALL(chgPin, read(_)).WillOnce(Return(false));

    BQ25101 charger(chgPin, tsPin);
    EXPECT_FALSE(charger.isCharging());
}

TEST_F(BQ25101Test, EnableCharging) {
    // enable=true → TS pin HIGH (normal operation, charging allowed)
    EXPECT_CALL(tsPin, write(true)).WillOnce(Return(true));

    BQ25101 charger(chgPin, tsPin);
    EXPECT_TRUE(charger.setChargeEnable(true));
}

TEST_F(BQ25101Test, DisableCharging) {
    // enable=false → TS pin LOW (charge inhibited)
    EXPECT_CALL(tsPin, write(false)).WillOnce(Return(true));

    BQ25101 charger(chgPin, tsPin);
    EXPECT_TRUE(charger.setChargeEnable(false));
}

TEST_F(BQ25101Test, SetChargeEnableFailsOnGpioError) {
    EXPECT_CALL(tsPin, write(_)).WillOnce(Return(false));

    BQ25101 charger(chgPin, tsPin);
    EXPECT_FALSE(charger.setChargeEnable(true));
}

TEST_F(BQ25101Test, SizeofIsSmall) {
    EXPECT_LE(sizeof(BQ25101), 16);
}
