#include <gtest/gtest.h>
#include "AltitudeEstimator.hpp"
#include <cmath>

using namespace sf;

TEST(AltitudeEstimatorTest, InitialAltitudeIsZero) {
    AltitudeEstimator est;
    EXPECT_FLOAT_EQ(est.altitude(), 0.0f);
    EXPECT_FLOAT_EQ(est.verticalVelocity(), 0.0f);
}

TEST(AltitudeEstimatorTest, SeaLevelPressureGivesZeroAlt) {
    AltitudeEstimator est(0.5f, 1013.25f);

    // Feed sea-level pressure and 1g accel (stationary)
    for (int i = 0; i < 200; ++i) {
        est.update(1013.25f, 1.0f, 0.01f);
    }

    // Should converge near 0 altitude
    EXPECT_NEAR(est.altitude(), 0.0f, 1.0f);
}

TEST(AltitudeEstimatorTest, LowerPressureGivesPositiveAlt) {
    AltitudeEstimator est(0.5f, 1013.25f);

    // ~900 hPa corresponds to ~1000m altitude
    for (int i = 0; i < 500; ++i) {
        est.update(900.0f, 1.0f, 0.01f);
    }

    // Should converge to barometric altitude
    float expected = 44330.0f * (1.0f - std::pow(900.0f / 1013.25f, 1.0f / 5.255f));
    EXPECT_NEAR(est.altitude(), expected, 50.0f);
}

TEST(AltitudeEstimatorTest, ResetClearsState) {
    AltitudeEstimator est;
    est.update(900.0f, 1.0f, 0.01f);
    est.reset();
    EXPECT_FLOAT_EQ(est.altitude(), 0.0f);
    EXPECT_FLOAT_EQ(est.verticalVelocity(), 0.0f);
}

TEST(AltitudeEstimatorTest, ConstantPressureStableAlt) {
    AltitudeEstimator est(0.5f);

    // Run at constant pressure for a long time
    for (int i = 0; i < 1000; ++i) {
        est.update(1013.25f, 1.0f, 0.01f);
    }

    float alt1 = est.altitude();

    // Run more — should remain stable
    for (int i = 0; i < 100; ++i) {
        est.update(1013.25f, 1.0f, 0.01f);
    }

    EXPECT_NEAR(est.altitude(), alt1, 0.1f);
}

TEST(AltitudeEstimatorTest, VelocityDampsToZeroWhenStationary) {
    AltitudeEstimator est(0.5f);

    // Run stationary
    for (int i = 0; i < 1000; ++i) {
        est.update(1013.25f, 1.0f, 0.01f);
    }

    EXPECT_NEAR(est.verticalVelocity(), 0.0f, 0.1f);
}

TEST(AltitudeEstimatorTest, HigherPressureGivesNegativeAlt) {
    AltitudeEstimator est(0.5f, 1013.25f);

    // Higher than sea level = below sea level
    for (int i = 0; i < 500; ++i) {
        est.update(1050.0f, 1.0f, 0.01f);
    }

    EXPECT_LT(est.altitude(), 0.0f);
}

TEST(AltitudeEstimatorSizeTest, SmallFootprint) {
    EXPECT_LE(sizeof(AltitudeEstimator), 20u);
}
