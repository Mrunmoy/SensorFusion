#include <gtest/gtest.h>
#include "Quaternion.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 1e-5f;

TEST(QuaternionTest, DefaultIsIdentity) {
    Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, NormOfIdentity) {
    Quaternion q;
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
}

TEST(QuaternionTest, NormOfArbitrary) {
    Quaternion q{1.0f, 2.0f, 3.0f, 4.0f};
    float expected = std::sqrt(1 + 4 + 9 + 16);
    EXPECT_NEAR(q.norm(), expected, TOL);
}

TEST(QuaternionTest, NormalizeIdentity) {
    Quaternion q;
    q.normalize();
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
}

TEST(QuaternionTest, NormalizeArbitrary) {
    Quaternion q{1.0f, 2.0f, 3.0f, 4.0f};
    q.normalize();
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
}

TEST(QuaternionTest, NormalizeZeroIsNoop) {
    Quaternion q{0.0f, 0.0f, 0.0f, 0.0f};
    q.normalize();
    EXPECT_FLOAT_EQ(q.w, 0.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
}

TEST(QuaternionTest, Conjugate) {
    Quaternion q{1.0f, 2.0f, 3.0f, 4.0f};
    auto c = q.conjugate();
    EXPECT_FLOAT_EQ(c.w, 1.0f);
    EXPECT_FLOAT_EQ(c.x, -2.0f);
    EXPECT_FLOAT_EQ(c.y, -3.0f);
    EXPECT_FLOAT_EQ(c.z, -4.0f);
}

TEST(QuaternionTest, MultiplyByIdentity) {
    Quaternion q{0.5f, 0.5f, 0.5f, 0.5f};
    Quaternion id;
    auto r = q.multiply(id);
    EXPECT_NEAR(r.w, q.w, TOL);
    EXPECT_NEAR(r.x, q.x, TOL);
    EXPECT_NEAR(r.y, q.y, TOL);
    EXPECT_NEAR(r.z, q.z, TOL);
}

TEST(QuaternionTest, IdentityMultiplyByQ) {
    Quaternion q{0.5f, 0.5f, 0.5f, 0.5f};
    Quaternion id;
    auto r = id.multiply(q);
    EXPECT_NEAR(r.w, q.w, TOL);
    EXPECT_NEAR(r.x, q.x, TOL);
    EXPECT_NEAR(r.y, q.y, TOL);
    EXPECT_NEAR(r.z, q.z, TOL);
}

TEST(QuaternionTest, MultiplyByConjugateGivesIdentity) {
    Quaternion q{0.5f, 0.5f, 0.5f, 0.5f};  // already unit
    auto c = q.conjugate();
    auto r = q.multiply(c);
    EXPECT_NEAR(r.w, 1.0f, TOL);
    EXPECT_NEAR(r.x, 0.0f, TOL);
    EXPECT_NEAR(r.y, 0.0f, TOL);
    EXPECT_NEAR(r.z, 0.0f, TOL);
}

TEST(QuaternionTest, MultiplyTwoRotations) {
    // 90-degree rotation about Z: q = cos(45) + sin(45)*k
    float c45 = std::cos(M_PI / 4.0f);
    float s45 = std::sin(M_PI / 4.0f);
    Quaternion q{c45, 0, 0, s45};

    // Two 90-degree rotations about Z = 180-degree rotation
    auto r = q.multiply(q);
    // Should be (0, 0, 0, 1) for 180 about Z
    EXPECT_NEAR(r.w, 0.0f, TOL);
    EXPECT_NEAR(r.x, 0.0f, TOL);
    EXPECT_NEAR(r.y, 0.0f, TOL);
    EXPECT_NEAR(r.z, 1.0f, TOL);
}

TEST(QuaternionTest, ToEulerIdentityIsZero) {
    Quaternion q;
    float r, p, y;
    q.toEuler(r, p, y);
    EXPECT_NEAR(r, 0.0f, TOL);
    EXPECT_NEAR(p, 0.0f, TOL);
    EXPECT_NEAR(y, 0.0f, TOL);
}

TEST(QuaternionTest, ToEuler90Roll) {
    // 90 degrees about X: q = cos(45) + sin(45)*i
    float c = std::cos(M_PI / 4.0f);
    float s = std::sin(M_PI / 4.0f);
    Quaternion q{c, s, 0, 0};
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 90.0f, 0.1f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
    EXPECT_NEAR(yaw, 0.0f, 0.1f);
}

TEST(QuaternionTest, ToEuler90Pitch) {
    // 90 degrees about Y: q = cos(45) + sin(45)*j
    float c = std::cos(M_PI / 4.0f);
    float s = std::sin(M_PI / 4.0f);
    Quaternion q{c, 0, s, 0};
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(pitch, 90.0f, 0.1f);
}

TEST(QuaternionTest, ToEuler90Yaw) {
    // 90 degrees about Z: q = cos(45) + sin(45)*k
    float c = std::cos(M_PI / 4.0f);
    float s = std::sin(M_PI / 4.0f);
    Quaternion q{c, 0, 0, s};
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 90.0f, 0.1f);
    EXPECT_NEAR(roll, 0.0f, 0.1f);
}

TEST(QuaternionTest, ToEulerGimbalLockPositive) {
    // Pitch = +90 (gimbal lock)
    Quaternion q{0.707107f, 0.0f, 0.707107f, 0.0f};
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(pitch, 90.0f, 0.5f);
}

TEST(QuaternionTest, ToEulerGimbalLockNegative) {
    // Pitch = -90
    Quaternion q{0.707107f, 0.0f, -0.707107f, 0.0f};
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(pitch, -90.0f, 0.5f);
}

TEST(QuaternionTest, NormPreservedAfterMultiply) {
    Quaternion a{0.5f, 0.5f, 0.5f, 0.5f};
    Quaternion b{0.707107f, 0.707107f, 0.0f, 0.0f};
    auto r = a.multiply(b);
    EXPECT_NEAR(r.norm(), 1.0f, TOL);
}

TEST(QuaternionTest, SizeIs16Bytes) {
    EXPECT_EQ(sizeof(Quaternion), 16u);
}
