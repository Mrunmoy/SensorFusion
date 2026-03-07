#include <gtest/gtest.h>
#include "CalibrationFitter.hpp"

using namespace sf;

TEST(CalibrationFitterTest, RejectsNullOrTooFewSamples) {
    CalibrationData cal;
    EXPECT_FALSE(CalibrationFitter::fitMagHardSoftIron(nullptr, 6, cal));

    MagData samples[5] = {};
    EXPECT_FALSE(CalibrationFitter::fitMagHardSoftIron(samples, 5, cal));
}

TEST(CalibrationFitterTest, FitsHardIronAndDiagonalSoftIron) {
    // True field radius = 50uT, hard-iron bias = {20,-10,5},
    // soft-iron diagonal distortion = {1.2,0.8,1.0}.
    MagData samples[6] = {
        {20.0f + 60.0f, -10.0f,         5.0f},   // +X
        {20.0f - 60.0f, -10.0f,         5.0f},   // -X
        {20.0f,         -10.0f + 40.0f, 5.0f},   // +Y
        {20.0f,         -10.0f - 40.0f, 5.0f},   // -Y
        {20.0f,         -10.0f,         5.0f + 50.0f}, // +Z
        {20.0f,         -10.0f,         5.0f - 50.0f}, // -Z
    };

    CalibrationData cal;
    ASSERT_TRUE(CalibrationFitter::fitMagHardSoftIron(samples, 6, cal));

    EXPECT_NEAR(cal.offsetX, 20.0f, 1e-4f);
    EXPECT_NEAR(cal.offsetY, -10.0f, 1e-4f);
    EXPECT_NEAR(cal.offsetZ, 5.0f, 1e-4f);
    EXPECT_NEAR(cal.scaleX, 50.0f / 60.0f, 1e-4f);
    EXPECT_NEAR(cal.scaleY, 50.0f / 40.0f, 1e-4f);
    EXPECT_NEAR(cal.scaleZ, 1.0f, 1e-4f);
}

TEST(CalibrationFitterTest, RejectsDegenerateSpan) {
    MagData samples[6] = {
        {1.0f, 2.0f, 3.0f},
        {1.0f, 2.0f, 3.0f},
        {1.0f, 2.0f, 3.0f},
        {1.0f, 2.0f, 3.0f},
        {1.0f, 2.0f, 3.0f},
        {1.0f, 2.0f, 3.0f},
    };
    CalibrationData cal;
    EXPECT_FALSE(CalibrationFitter::fitMagHardSoftIron(samples, 6, cal));
}

TEST(CalibrationFitterTest, FitsAccelerometerOffsetAndScale) {
    // Distorted accel readings:
    // x = 0.10 + 1.2 * ax, y = -0.05 + 0.8 * ay, z = 0.02 + 1.0 * az
    // where ax/ay/az are true +/-1g per-axis sweeps.
    AccelData samples[6] = {
        { 0.10f + 1.2f, -0.05f,        0.02f}, // +X
        { 0.10f - 1.2f, -0.05f,        0.02f}, // -X
        { 0.10f,       -0.05f + 0.8f,  0.02f}, // +Y
        { 0.10f,       -0.05f - 0.8f,  0.02f}, // -Y
        { 0.10f,       -0.05f,         0.02f + 1.0f}, // +Z
        { 0.10f,       -0.05f,         0.02f - 1.0f}, // -Z
    };

    CalibrationData cal;
    ASSERT_TRUE(CalibrationFitter::fitAccelOffsetScale(samples, 6, cal));
    EXPECT_NEAR(cal.offsetX, 0.10f, 1e-4f);
    EXPECT_NEAR(cal.offsetY, -0.05f, 1e-4f);
    EXPECT_NEAR(cal.offsetZ, 0.02f, 1e-4f);
    EXPECT_NEAR(cal.scaleX, 1.0f / 1.2f, 1e-4f);
    EXPECT_NEAR(cal.scaleY, 1.0f / 0.8f, 1e-4f);
    EXPECT_NEAR(cal.scaleZ, 1.0f / 1.0f, 1e-4f);
}

TEST(CalibrationFitterTest, RejectsAccelerometerDegenerateSpan) {
    AccelData samples[6] = {
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f},
    };
    CalibrationData cal;
    EXPECT_FALSE(CalibrationFitter::fitAccelOffsetScale(samples, 6, cal));
}

TEST(CalibrationFitterTest, FitsGyroscopeBias) {
    GyroData samples[4] = {
        {0.10f, -0.20f, 0.30f},
        {0.12f, -0.18f, 0.28f},
        {0.08f, -0.22f, 0.32f},
        {0.10f, -0.20f, 0.30f},
    };
    CalibrationData cal;
    ASSERT_TRUE(CalibrationFitter::fitGyroBias(samples, 4, cal));
    EXPECT_NEAR(cal.offsetX, 0.10f, 1e-4f);
    EXPECT_NEAR(cal.offsetY, -0.20f, 1e-4f);
    EXPECT_NEAR(cal.offsetZ, 0.30f, 1e-4f);
    EXPECT_FLOAT_EQ(cal.scaleX, 1.0f);
    EXPECT_FLOAT_EQ(cal.scaleY, 1.0f);
    EXPECT_FLOAT_EQ(cal.scaleZ, 1.0f);
}
