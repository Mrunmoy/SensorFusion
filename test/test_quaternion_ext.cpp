#include <gtest/gtest.h>
#include "Quaternion.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 1e-4f;

// --- inverse ---

TEST(QuaternionExtTest, InverseOfIdentityIsIdentity) {
    Quaternion q;
    auto inv = q.inverse();
    EXPECT_FLOAT_EQ(inv.w, 1.0f);
    EXPECT_FLOAT_EQ(inv.x, 0.0f);
    EXPECT_FLOAT_EQ(inv.y, 0.0f);
    EXPECT_FLOAT_EQ(inv.z, 0.0f);
}

TEST(QuaternionExtTest, InverseEqualsConjugateForUnitQuat) {
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 45.0f);
    auto inv = q.inverse();
    auto conj = q.conjugate();
    EXPECT_NEAR(inv.w, conj.w, TOL);
    EXPECT_NEAR(inv.x, conj.x, TOL);
    EXPECT_NEAR(inv.y, conj.y, TOL);
    EXPECT_NEAR(inv.z, conj.z, TOL);
}

// --- rotateVector ---

TEST(QuaternionExtTest, RotateVectorIdentityNoChange) {
    Quaternion q;  // identity
    Vec3 v{1.0f, 2.0f, 3.0f};
    auto r = q.rotateVector(v);
    EXPECT_NEAR(r.x, 1.0f, TOL);
    EXPECT_NEAR(r.y, 2.0f, TOL);
    EXPECT_NEAR(r.z, 3.0f, TOL);
}

TEST(QuaternionExtTest, RotateVector90AboutZ) {
    // 90 deg about Z: +X -> +Y
    auto q = Quaternion::fromAxisAngle(0, 0, 1, 90.0f);
    Vec3 v{1.0f, 0.0f, 0.0f};
    auto r = q.rotateVector(v);
    EXPECT_NEAR(r.x, 0.0f, TOL);
    EXPECT_NEAR(r.y, 1.0f, TOL);
    EXPECT_NEAR(r.z, 0.0f, TOL);
}

TEST(QuaternionExtTest, RotateVector90AboutX) {
    // 90 deg about X: +Y -> +Z
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    Vec3 v{0.0f, 1.0f, 0.0f};
    auto r = q.rotateVector(v);
    EXPECT_NEAR(r.x, 0.0f, TOL);
    EXPECT_NEAR(r.y, 0.0f, TOL);
    EXPECT_NEAR(r.z, 1.0f, TOL);
}

TEST(QuaternionExtTest, RotateVector180AboutY) {
    // 180 deg about Y: +X -> -X, +Z -> -Z
    auto q = Quaternion::fromAxisAngle(0, 1, 0, 180.0f);
    Vec3 v{1.0f, 0.0f, 1.0f};
    auto r = q.rotateVector(v);
    EXPECT_NEAR(r.x, -1.0f, TOL);
    EXPECT_NEAR(r.y, 0.0f, TOL);
    EXPECT_NEAR(r.z, -1.0f, TOL);
}

TEST(QuaternionExtTest, RotateVectorPreservesLength) {
    auto q = Quaternion::fromAxisAngle(1, 1, 1, 73.0f);
    Vec3 v{3.0f, 4.0f, 5.0f};
    auto r = q.rotateVector(v);
    EXPECT_NEAR(r.length(), v.length(), TOL);
}

// --- toRotationMatrix ---

TEST(QuaternionExtTest, ToRotationMatrixIdentity) {
    Quaternion q;
    float m[9];
    q.toRotationMatrix(m);
    // Identity matrix
    EXPECT_NEAR(m[0], 1.0f, TOL); EXPECT_NEAR(m[1], 0.0f, TOL); EXPECT_NEAR(m[2], 0.0f, TOL);
    EXPECT_NEAR(m[3], 0.0f, TOL); EXPECT_NEAR(m[4], 1.0f, TOL); EXPECT_NEAR(m[5], 0.0f, TOL);
    EXPECT_NEAR(m[6], 0.0f, TOL); EXPECT_NEAR(m[7], 0.0f, TOL); EXPECT_NEAR(m[8], 1.0f, TOL);
}

TEST(QuaternionExtTest, ToRotationMatrix90Z) {
    auto q = Quaternion::fromAxisAngle(0, 0, 1, 90.0f);
    float m[9];
    q.toRotationMatrix(m);
    // [0 -1 0; 1 0 0; 0 0 1]
    EXPECT_NEAR(m[0], 0.0f, TOL); EXPECT_NEAR(m[1], -1.0f, TOL); EXPECT_NEAR(m[2], 0.0f, TOL);
    EXPECT_NEAR(m[3], 1.0f, TOL); EXPECT_NEAR(m[4], 0.0f, TOL);  EXPECT_NEAR(m[5], 0.0f, TOL);
    EXPECT_NEAR(m[6], 0.0f, TOL); EXPECT_NEAR(m[7], 0.0f, TOL);  EXPECT_NEAR(m[8], 1.0f, TOL);
}

TEST(QuaternionExtTest, RotationMatrixConsistentWithRotateVector) {
    auto q = Quaternion::fromAxisAngle(1, 2, 3, 47.0f);
    Vec3 v{0.5f, -1.3f, 2.7f};
    auto rv = q.rotateVector(v);
    float m[9];
    q.toRotationMatrix(m);
    float mx = m[0] * v.x + m[1] * v.y + m[2] * v.z;
    float my = m[3] * v.x + m[4] * v.y + m[5] * v.z;
    float mz = m[6] * v.x + m[7] * v.y + m[8] * v.z;
    EXPECT_NEAR(rv.x, mx, TOL);
    EXPECT_NEAR(rv.y, my, TOL);
    EXPECT_NEAR(rv.z, mz, TOL);
}

// --- slerp ---

TEST(QuaternionExtTest, SlerpAtZeroReturnsA) {
    auto a = Quaternion::fromAxisAngle(1, 0, 0, 0.0f);
    auto b = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    auto r = Quaternion::slerp(a, b, 0.0f);
    EXPECT_NEAR(r.w, a.w, TOL);
    EXPECT_NEAR(r.x, a.x, TOL);
    EXPECT_NEAR(r.y, a.y, TOL);
    EXPECT_NEAR(r.z, a.z, TOL);
}

TEST(QuaternionExtTest, SlerpAtOneReturnsB) {
    auto a = Quaternion::fromAxisAngle(1, 0, 0, 0.0f);
    auto b = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    auto r = Quaternion::slerp(a, b, 1.0f);
    EXPECT_NEAR(r.w, b.w, TOL);
    EXPECT_NEAR(r.x, b.x, TOL);
    EXPECT_NEAR(r.y, b.y, TOL);
    EXPECT_NEAR(r.z, b.z, TOL);
}

TEST(QuaternionExtTest, SlerpMidpoint) {
    auto a = Quaternion::fromAxisAngle(1, 0, 0, 0.0f);
    auto b = Quaternion::fromAxisAngle(1, 0, 0, 90.0f);
    auto mid = Quaternion::slerp(a, b, 0.5f);
    auto expected = Quaternion::fromAxisAngle(1, 0, 0, 45.0f);
    EXPECT_NEAR(mid.w, expected.w, TOL);
    EXPECT_NEAR(mid.x, expected.x, TOL);
}

TEST(QuaternionExtTest, SlerpIdenticalQuaternions) {
    auto a = Quaternion::fromAxisAngle(0, 1, 0, 30.0f);
    auto r = Quaternion::slerp(a, a, 0.5f);
    EXPECT_NEAR(r.w, a.w, TOL);
    EXPECT_NEAR(r.x, a.x, TOL);
    EXPECT_NEAR(r.y, a.y, TOL);
    EXPECT_NEAR(r.z, a.z, TOL);
}

TEST(QuaternionExtTest, SlerpResultIsUnit) {
    auto a = Quaternion::fromAxisAngle(1, 0, 0, 10.0f);
    auto b = Quaternion::fromAxisAngle(0, 1, 0, 80.0f);
    auto r = Quaternion::slerp(a, b, 0.3f);
    EXPECT_NEAR(r.norm(), 1.0f, TOL);
}

// --- fromAxisAngle ---

TEST(QuaternionExtTest, FromAxisAngle90Z) {
    auto q = Quaternion::fromAxisAngle(0, 0, 1, 90.0f);
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 90.0f, 0.1f);
    EXPECT_NEAR(roll, 0.0f, 0.1f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
}

TEST(QuaternionExtTest, FromAxisAngleZeroAngle) {
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 0.0f);
    EXPECT_NEAR(q.w, 1.0f, TOL);
    EXPECT_NEAR(q.x, 0.0f, TOL);
    EXPECT_NEAR(q.y, 0.0f, TOL);
    EXPECT_NEAR(q.z, 0.0f, TOL);
}

TEST(QuaternionExtTest, FromAxisAngleZeroAxisReturnsIdentity) {
    auto q = Quaternion::fromAxisAngle(0, 0, 0, 45.0f);
    EXPECT_NEAR(q.w, 1.0f, TOL);
    EXPECT_NEAR(q.x, 0.0f, TOL);
}

TEST(QuaternionExtTest, SizeStill16Bytes) {
    EXPECT_EQ(sizeof(Quaternion), 16u);
}
