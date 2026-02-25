#include <gtest/gtest.h>
#include "Vec3.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 1e-5f;

TEST(Vec3Test, DefaultIsZero) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, Addition) {
    Vec3 a{1.0f, 2.0f, 3.0f};
    Vec3 b{4.0f, 5.0f, 6.0f};
    Vec3 r = a + b;
    EXPECT_FLOAT_EQ(r.x, 5.0f);
    EXPECT_FLOAT_EQ(r.y, 7.0f);
    EXPECT_FLOAT_EQ(r.z, 9.0f);
}

TEST(Vec3Test, Subtraction) {
    Vec3 a{4.0f, 5.0f, 6.0f};
    Vec3 b{1.0f, 2.0f, 3.0f};
    Vec3 r = a - b;
    EXPECT_FLOAT_EQ(r.x, 3.0f);
    EXPECT_FLOAT_EQ(r.y, 3.0f);
    EXPECT_FLOAT_EQ(r.z, 3.0f);
}

TEST(Vec3Test, ScalarMultiply) {
    Vec3 v{1.0f, -2.0f, 3.0f};
    Vec3 r = v * 2.0f;
    EXPECT_FLOAT_EQ(r.x, 2.0f);
    EXPECT_FLOAT_EQ(r.y, -4.0f);
    EXPECT_FLOAT_EQ(r.z, 6.0f);
}

TEST(Vec3Test, DotProduct) {
    Vec3 a{1.0f, 2.0f, 3.0f};
    Vec3 b{4.0f, 5.0f, 6.0f};
    EXPECT_FLOAT_EQ(a.dot(b), 32.0f);  // 4+10+18
}

TEST(Vec3Test, DotProductOrthogonal) {
    Vec3 a{1.0f, 0.0f, 0.0f};
    Vec3 b{0.0f, 1.0f, 0.0f};
    EXPECT_FLOAT_EQ(a.dot(b), 0.0f);
}

TEST(Vec3Test, LengthSqOfUnitAxis) {
    Vec3 v{1.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(v.lengthSq(), 1.0f);
}

TEST(Vec3Test, LengthOfUnitAxis) {
    Vec3 v{0.0f, 1.0f, 0.0f};
    EXPECT_FLOAT_EQ(v.length(), 1.0f);
}

TEST(Vec3Test, LengthOfArbitrary) {
    Vec3 v{3.0f, 4.0f, 0.0f};
    EXPECT_NEAR(v.length(), 5.0f, TOL);
}

TEST(Vec3Test, ZeroLength) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.length(), 0.0f);
}

TEST(Vec3Test, SubtractSelf) {
    Vec3 v{5.0f, -3.0f, 7.0f};
    Vec3 r = v - v;
    EXPECT_FLOAT_EQ(r.x, 0.0f);
    EXPECT_FLOAT_EQ(r.y, 0.0f);
    EXPECT_FLOAT_EQ(r.z, 0.0f);
}

TEST(Vec3SizeTest, Is12Bytes) {
    EXPECT_EQ(sizeof(Vec3), 12u);
}
