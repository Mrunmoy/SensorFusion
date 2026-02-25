#include <gtest/gtest.h>
#include "LinearAccelExtractor.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 0.02f;

TEST(LinearAccelTest, StationaryFlatNoLinearAccel) {
    LinearAccelExtractor ext;
    Quaternion identity;
    // Sensor flat: reads pure gravity (0, 0, -1g)
    AccelData raw{0.0f, 0.0f, -1.0f};
    auto linear = ext.extract(identity, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, StationaryTiltedNoLinearAccel) {
    LinearAccelExtractor ext;
    // Sensor tilted 90 deg about X: gravity now along +Y in body frame
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    AccelData raw{0.0f, -1.0f, 0.0f};
    auto linear = ext.extract(q, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, LinearAccelInWorldFrame) {
    LinearAccelExtractor ext;
    Quaternion identity;
    // Sensor flat, reading gravity + 0.5g in X
    AccelData raw{0.5f, 0.0f, -1.0f};
    auto linear = ext.extract(identity, raw);
    EXPECT_NEAR(linear.x, 0.5f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, BodyFrameGravityRemoval) {
    LinearAccelExtractor ext;
    Quaternion identity;
    AccelData raw{0.0f, 0.0f, -1.0f};
    auto linear = ext.extractBodyFrame(identity, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, BodyFrameTilted) {
    LinearAccelExtractor ext;
    // 90 deg about X: gravity in body frame is (0, -1, 0)
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    AccelData raw{0.0f, -1.0f, 0.0f};
    auto linear = ext.extractBodyFrame(q, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, CustomGravityMagnitude) {
    LinearAccelExtractor ext(9.81f);
    Quaternion identity;
    AccelData raw{0.0f, 0.0f, -9.81f};
    auto linear = ext.extract(identity, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, 0.0f, TOL);
}

TEST(LinearAccelTest, PureLinearMotionDetected) {
    LinearAccelExtractor ext;
    Quaternion identity;
    // Sensor flat, accelerating upward at 0.3g
    AccelData raw{0.0f, 0.0f, -1.3f};
    auto linear = ext.extract(identity, raw);
    EXPECT_NEAR(linear.x, 0.0f, TOL);
    EXPECT_NEAR(linear.y, 0.0f, TOL);
    EXPECT_NEAR(linear.z, -0.3f, TOL);
}

TEST(LinearAccelTest, Stateless) {
    LinearAccelExtractor ext;
    Quaternion identity;
    AccelData raw{0.1f, 0.2f, -1.0f};
    auto a = ext.extract(identity, raw);
    auto b = ext.extract(identity, raw);
    EXPECT_FLOAT_EQ(a.x, b.x);
    EXPECT_FLOAT_EQ(a.y, b.y);
    EXPECT_FLOAT_EQ(a.z, b.z);
}

TEST(LinearAccelSizeTest, Is4Bytes) {
    EXPECT_EQ(sizeof(LinearAccelExtractor), 4u);
}
