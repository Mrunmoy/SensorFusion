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
