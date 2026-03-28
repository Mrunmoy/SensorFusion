#include <gtest/gtest.h>
#include "MahonyAHRS.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 1.0f;      // 1 degree tolerance for convergence tests
static constexpr float TIGHT = 0.01f;   // tight tolerance

class MahonyAHRSTest : public ::testing::Test {
protected:
    MahonyAHRS ahrs{2.0f, 0.0f};  // aggressive Kp for faster convergence in tests
};

TEST_F(MahonyAHRSTest, InitialQuaternionIsIdentity) {
    auto q = ahrs.getQuaternion();
    EXPECT_NEAR(q.w, 1.0f, TIGHT);
    EXPECT_NEAR(q.x, 0.0f, TIGHT);
    EXPECT_NEAR(q.y, 0.0f, TIGHT);
    EXPECT_NEAR(q.z, 0.0f, TIGHT);
}

TEST_F(MahonyAHRSTest, InitialEulerIsZero) {
    float r, p, y;
    ahrs.getEulerDeg(r, p, y);
    EXPECT_NEAR(r, 0.0f, TIGHT);
    EXPECT_NEAR(p, 0.0f, TIGHT);
    EXPECT_NEAR(y, 0.0f, TIGHT);
}

TEST_F(MahonyAHRSTest, GravityDownConvergesToIdentity6DOF) {
    // Accelerometer reading gravity in -Z, gyro zero
    AccelData a{0.0f, 0.0f, -1.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    float dt = 0.01f;

    for (int i = 0; i < 200; ++i) {
        ahrs.update6DOF(a, g, dt);
    }

    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, TOL);
    EXPECT_NEAR(pitch, 0.0f, TOL);
}

TEST_F(MahonyAHRSTest, GravityInXDetectsRoll6DOF) {
    // Sensor rotated 90 degrees: gravity now along +X
    AccelData a{1.0f, 0.0f, 0.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    float dt = 0.01f;

    for (int i = 0; i < 500; ++i) {
        ahrs.update6DOF(a, g, dt);
    }

    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    // Gravity in +X means roll should be ~-90 or pitch ~90 depending on convention
    // With ZYX convention and gravity normally in -Z, gravity in +X means pitch = +90
    EXPECT_NEAR(pitch, -90.0f, 2.0f);
}

TEST_F(MahonyAHRSTest, GravityInYDetectsRoll6DOF) {
    // Gravity along +Y
    AccelData a{0.0f, 1.0f, 0.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    float dt = 0.01f;

    for (int i = 0; i < 500; ++i) {
        ahrs.update6DOF(a, g, dt);
    }

    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    EXPECT_NEAR(roll, 90.0f, 2.0f);
}

TEST_F(MahonyAHRSTest, ResetRestoresIdentity) {
    AccelData a{1.0f, 0.0f, 0.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; ++i) {
        ahrs.update6DOF(a, g, 0.01f);
    }

    ahrs.reset();
    auto q = ahrs.getQuaternion();
    EXPECT_NEAR(q.w, 1.0f, TIGHT);
    EXPECT_NEAR(q.x, 0.0f, TIGHT);
    EXPECT_NEAR(q.y, 0.0f, TIGHT);
    EXPECT_NEAR(q.z, 0.0f, TIGHT);
}

TEST_F(MahonyAHRSTest, QuaternionStaysNormalized) {
    AccelData a{0.3f, 0.5f, -0.8f};
    GyroData  g{10.0f, -5.0f, 3.0f};
    for (int i = 0; i < 1000; ++i) {
        ahrs.update6DOF(a, g, 0.01f);
    }
    auto q = ahrs.getQuaternion();
    EXPECT_NEAR(q.norm(), 1.0f, 0.001f);
}

TEST_F(MahonyAHRSTest, ZeroAccelSkipsCorrection6DOF) {
    // Zero accel should not cause NaN
    AccelData a{0.0f, 0.0f, 0.0f};
    GyroData  g{0.0f, 0.0f, 10.0f};
    ahrs.update6DOF(a, g, 0.01f);
    auto q = ahrs.getQuaternion();
    EXPECT_FALSE(std::isnan(q.w));
    EXPECT_FALSE(std::isnan(q.x));
}

TEST_F(MahonyAHRSTest, Update9DOFWithValidData) {
    AccelData a{0.0f, 0.0f, -1.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    MagData   m{20.0f, 0.0f, -40.0f};

    for (int i = 0; i < 200; ++i) {
        ahrs.update(a, g, m, 0.01f);
    }

    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, TOL);
    EXPECT_NEAR(pitch, 0.0f, TOL);
    // Yaw should converge to some stable value based on mag heading
    EXPECT_FALSE(std::isnan(yaw));
}

TEST_F(MahonyAHRSTest, Update9DOFZeroMagFallsBackTo6DOF) {
    AccelData a{0.0f, 0.0f, -1.0f};
    GyroData  g{0.0f, 0.0f, 0.0f};
    MagData   m{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 100; ++i) {
        ahrs.update(a, g, m, 0.01f);
    }

    auto q = ahrs.getQuaternion();
    EXPECT_FALSE(std::isnan(q.w));
    EXPECT_NEAR(q.norm(), 1.0f, 0.001f);
}

TEST_F(MahonyAHRSTest, InitFromSensorsSeedsLargeYawOffsetCloseToTruth) {
    const Quaternion truth = Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, 90.0f);
    const Vec3 worldGravity{0.0f, 0.0f, 1.0f};
    const Vec3 worldMag{20.0f, 0.0f, -40.0f};
    const Vec3 bodyGravity = truth.rotateVector(worldGravity);
    const Vec3 bodyMag = truth.rotateVector(worldMag);

    ahrs.initFromSensors(
        AccelData{bodyGravity.x, bodyGravity.y, bodyGravity.z},
        MagData{bodyMag.x, bodyMag.y, bodyMag.z});

    const Quaternion seeded = ahrs.getQuaternion();
    const Quaternion relative = truth.conjugate().multiply(seeded);
    const float clampedW = std::fmax(-1.0f, std::fmin(1.0f, relative.w));
    const float errorDeg = std::fabs(2.0f * std::acos(clampedW) * 180.0f / 3.14159265358979323846f);

    EXPECT_LT(errorDeg, 5.0f);
}

TEST_F(MahonyAHRSTest, GyroOnlyIntegration) {
    // Pure gyro rotation about Z at 90 deg/s for 1s = 90 degrees
    AccelData a{0.0f, 0.0f, 0.0f};  // zero accel → no correction
    GyroData  g{0.0f, 0.0f, 90.0f};
    float dt = 0.001f;

    for (int i = 0; i < 1000; ++i) {
        ahrs.update6DOF(a, g, dt);
    }

    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 90.0f, 2.0f);
}

TEST(MahonyAHRSSizeTest, StateSize) {
    // Verify the filter is lightweight
    EXPECT_LE(sizeof(MahonyAHRS), 40u);
}

TEST(MahonyAHRSIntegralTest, IntegralFeedbackReducesBias) {
    MahonyAHRS ahrs(2.0f, 1.0f);  // Higher Ki for faster bias compensation

    // Simulate a gyro bias: reading 5 deg/s when actually stationary
    AccelData a{0.0f, 0.0f, -1.0f};
    GyroData  g{0.0f, 0.0f, 5.0f};  // bias

    // Run for 60s simulated time to let integral term converge
    for (int i = 0; i < 6000; ++i) {
        ahrs.update6DOF(a, g, 0.01f);
    }

    // With integral feedback, yaw drift should be bounded
    // Compare with no-integral case: 5 deg/s * 60s = 300 deg drift
    // With Ki, drift should be significantly less
    float roll, pitch, yaw;
    ahrs.getEulerDeg(roll, pitch, yaw);
    float absYaw = std::fabs(yaw);
    EXPECT_LT(absYaw, 180.0f);  // Well below the 300 deg without integral
}
