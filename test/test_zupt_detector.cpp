#include <gtest/gtest.h>
#include "ZuptDetector.hpp"
#include <cmath>

using namespace sf;

TEST(ZuptTest, InitiallyNotStationary) {
    ZuptDetector zupt;
    EXPECT_FALSE(zupt.isStationary());
}

TEST(ZuptTest, ConstantGravityIsStationary) {
    ZuptDetector zupt(0.001f);
    for (int i = 0; i < 32; ++i) {
        zupt.processSample({0.0f, 0.0f, -1.0f});
    }
    EXPECT_TRUE(zupt.isStationary());
    EXPECT_NEAR(zupt.variance(), 0.0f, 0.0001f);
}

TEST(ZuptTest, HighVarianceIsNotStationary) {
    ZuptDetector zupt(0.001f);
    for (int i = 0; i < 32; ++i) {
        float noise = (i % 2 == 0) ? 2.0f : 0.0f;
        zupt.processSample({noise, 0.0f, -1.0f});
    }
    EXPECT_FALSE(zupt.isStationary());
    EXPECT_GT(zupt.variance(), 0.001f);
}

TEST(ZuptTest, TransitionToStationary) {
    ZuptDetector zupt(0.001f);
    // Noisy phase: alternating high/low magnitude
    for (int i = 0; i < 32; ++i) {
        float z = (i % 2 == 0) ? -2.0f : -0.5f;
        zupt.processSample({0.0f, 0.0f, z});
    }
    EXPECT_FALSE(zupt.isStationary());

    // Settle to stationary
    for (int i = 0; i < 32; ++i) {
        zupt.processSample({0.0f, 0.0f, -1.0f});
    }
    EXPECT_TRUE(zupt.isStationary());
}

TEST(ZuptTest, TransitionFromStationary) {
    ZuptDetector zupt(0.001f);
    for (int i = 0; i < 32; ++i) {
        zupt.processSample({0.0f, 0.0f, -1.0f});
    }
    EXPECT_TRUE(zupt.isStationary());

    // Inject motion
    for (int i = 0; i < 32; ++i) {
        zupt.processSample({(i % 2 == 0) ? 2.0f : 0.0f, 0.0f, -1.0f});
    }
    EXPECT_FALSE(zupt.isStationary());
}

TEST(ZuptTest, VarianceComputedCorrectly) {
    ZuptDetector zupt(100.0f);  // high threshold so we can check variance
    // Feed alternating 1.0 and 3.0 magnitudes: mean=2, var=1
    for (int i = 0; i < 32; ++i) {
        float val = (i % 2 == 0) ? 1.0f : 3.0f;
        zupt.processSample({val, 0.0f, 0.0f});  // mag = val
    }
    // Variance of [1,3,1,3,...]: mean=2, var = E[x^2]-E[x]^2 = (5) - (4) = 1.0
    EXPECT_NEAR(zupt.variance(), 1.0f, 0.01f);
}

TEST(ZuptTest, ResetClearsState) {
    ZuptDetector zupt(0.001f);
    for (int i = 0; i < 32; ++i) {
        zupt.processSample({0.0f, 0.0f, -1.0f});
    }
    EXPECT_TRUE(zupt.isStationary());

    zupt.reset();
    EXPECT_FALSE(zupt.isStationary());
    EXPECT_GT(zupt.variance(), 0.5f);  // reset to 1.0
}

TEST(ZuptTest, CustomThreshold) {
    ZuptDetector zupt(0.1f);  // relaxed threshold
    // Small noise that would fail with 0.001 threshold
    for (int i = 0; i < 32; ++i) {
        float small = (i % 2 == 0) ? -0.98f : -1.02f;
        zupt.processSample({0.0f, 0.0f, small});
    }
    EXPECT_TRUE(zupt.isStationary());
}

TEST(ZuptTest, NeedsFullWindowBeforeStationary) {
    ZuptDetector zupt(100.0f);  // very high threshold
    // Even with zero variance, should not be stationary until window full
    for (int i = 0; i < 31; ++i) {
        zupt.processSample({0.0f, 0.0f, -1.0f});
    }
    EXPECT_FALSE(zupt.isStationary());
    zupt.processSample({0.0f, 0.0f, -1.0f});  // 32nd sample
    EXPECT_TRUE(zupt.isStationary());
}

TEST(ZuptSizeTest, LessThan168Bytes) {
    EXPECT_LE(sizeof(ZuptDetector), 168u);
}
