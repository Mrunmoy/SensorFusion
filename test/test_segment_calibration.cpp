#include <gtest/gtest.h>
#include "SegmentCalibration.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 1e-3f;

TEST(SegmentCalTest, InitiallyNotCalibrated) {
    SegmentCalibration cal;
    EXPECT_FALSE(cal.isCalibrated());
}

TEST(SegmentCalTest, PassthroughWhenNotCalibrated) {
    SegmentCalibration cal;
    auto q = Quaternion::fromAxisAngle(0, 0, 1, 45.0f);
    auto seg = cal.segmentOrientation(q);
    EXPECT_NEAR(seg.w, q.w, TOL);
    EXPECT_NEAR(seg.x, q.x, TOL);
    EXPECT_NEAR(seg.y, q.y, TOL);
    EXPECT_NEAR(seg.z, q.z, TOL);
}

TEST(SegmentCalTest, CaptureReferenceMarksCalibrated) {
    SegmentCalibration cal;
    Quaternion q;
    cal.captureReference(q);
    EXPECT_TRUE(cal.isCalibrated());
}

TEST(SegmentCalTest, IdentityRefReturnsCurrentOrientation) {
    SegmentCalibration cal;
    Quaternion identity;
    cal.captureReference(identity);
    auto q = Quaternion::fromAxisAngle(1, 0, 0, 30.0f);
    auto seg = cal.segmentOrientation(q);
    EXPECT_NEAR(seg.w, q.w, TOL);
    EXPECT_NEAR(seg.x, q.x, TOL);
    EXPECT_NEAR(seg.y, q.y, TOL);
    EXPECT_NEAR(seg.z, q.z, TOL);
}

TEST(SegmentCalTest, TPoseCalibrationRemovesOffset) {
    SegmentCalibration cal;
    // Sensor mounted at 30-degree offset about Z
    auto mountOffset = Quaternion::fromAxisAngle(0, 0, 1, 30.0f);
    cal.captureReference(mountOffset);

    // During motion: sensor reads 60 degrees total about Z
    auto current = Quaternion::fromAxisAngle(0, 0, 1, 60.0f);
    auto seg = cal.segmentOrientation(current);

    // Segment should represent 30 degrees of actual rotation
    float roll, pitch, yaw;
    seg.toEuler(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 30.0f, 1.0f);
    EXPECT_NEAR(roll, 0.0f, 1.0f);
    EXPECT_NEAR(pitch, 0.0f, 1.0f);
}

TEST(SegmentCalTest, TwoSegmentsIndependent) {
    SegmentCalibration calA, calB;
    calA.captureReference(Quaternion::fromAxisAngle(0, 0, 1, 10.0f));
    calB.captureReference(Quaternion::fromAxisAngle(0, 0, 1, 50.0f));

    auto current = Quaternion::fromAxisAngle(0, 0, 1, 70.0f);
    auto segA = calA.segmentOrientation(current);
    auto segB = calB.segmentOrientation(current);

    float rA, pA, yA, rB, pB, yB;
    segA.toEuler(rA, pA, yA);
    segB.toEuler(rB, pB, yB);
    EXPECT_NEAR(yA, 60.0f, 1.0f);
    EXPECT_NEAR(yB, 20.0f, 1.0f);
}

TEST(SegmentCalTest, ResetClearsCalibration) {
    SegmentCalibration cal;
    cal.captureReference(Quaternion::fromAxisAngle(1, 0, 0, 45.0f));
    EXPECT_TRUE(cal.isCalibrated());
    cal.reset();
    EXPECT_FALSE(cal.isCalibrated());
}

TEST(SegmentCalSizeTest, Is20Bytes) {
    EXPECT_LE(sizeof(SegmentCalibration), 20u);
}
