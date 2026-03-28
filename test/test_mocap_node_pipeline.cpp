#include "MocapNodePipeline.hpp"
#include "Quaternion.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace sf {
namespace {

class FakeImu final : public IAccelGyroSensor {
public:
    bool accelOk = true;
    bool gyroOk = true;
    AccelData accel{0.0f, 0.0f, 1.0f};
    GyroData gyro{0.0f, 0.0f, 0.0f};

    bool readAccel(AccelData& out) override {
        out = accel;
        return accelOk;
    }

    bool readGyro(GyroData& out) override {
        out = gyro;
        return gyroOk;
    }

    bool readTemperature(float& tempC) override {
        tempC = 25.0f;
        return true;
    }
};

class FakeMag final : public IMagSensor {
public:
    bool ok = true;
    MagData mag{30.0f, 0.0f, 5.0f};

    bool readMag(MagData& out) override {
        out = mag;
        return ok;
    }
};

class FakeBaro final : public IBaroSensor {
public:
    bool ok = true;
    float hPa = 1013.25f;

    bool readPressureHPa(float& out) override {
        out = hPa;
        return ok;
    }
};

TEST(MocapNodePipelineTest, StepReadsAllSensorsWhenAvailable) {
    FakeImu imu;
    FakeMag mag;
    FakeBaro baro;
    MocapNodePipeline pipe(imu, &mag, &baro);

    MocapNodeSample sample{};
    EXPECT_TRUE(pipe.step(sample));
    EXPECT_TRUE(sample.hasMag);
    EXPECT_TRUE(sample.hasPressure);
    EXPECT_NEAR(sample.pressureHPa, 1013.25f, 1e-3f);
    EXPECT_NEAR(sample.orientation.norm(), 1.0f, 1e-3f);
}

TEST(MocapNodePipelineTest, MagFailureFallsBackTo6Dof) {
    FakeImu imu;
    FakeMag mag;
    mag.ok = false;
    FakeBaro baro;
    MocapNodePipeline pipe(imu, &mag, &baro);

    MocapNodeSample sample{};
    EXPECT_TRUE(pipe.step(sample));
    EXPECT_FALSE(sample.hasMag);
    EXPECT_TRUE(sample.hasPressure);
}

TEST(MocapNodePipelineTest, ImuFailureReturnsFalse) {
    FakeImu imu;
    imu.accelOk = false;
    FakeMag mag;
    FakeBaro baro;
    MocapNodePipeline pipe(imu, &mag, &baro);

    MocapNodeSample sample{};
    EXPECT_FALSE(pipe.step(sample));
}

TEST(MocapNodePipelineTest, BaroFailureDoesNotFailStep) {
    FakeImu imu;
    FakeMag mag;
    FakeBaro baro;
    baro.ok = false;
    MocapNodePipeline pipe(imu, &mag, &baro);

    MocapNodeSample sample{};
    EXPECT_TRUE(pipe.step(sample));
    EXPECT_FALSE(sample.hasPressure);
}

TEST(MocapNodePipelineTest, PreferMagFalseSkipsMagRead) {
    FakeImu imu;
    FakeMag mag;
    FakeBaro baro;
    MocapNodePipeline::Config cfg{};
    cfg.preferMag = false;
    MocapNodePipeline pipe(imu, &mag, &baro, cfg);

    MocapNodeSample sample{};
    EXPECT_TRUE(pipe.step(sample));
    EXPECT_FALSE(sample.hasMag);
}

TEST(MocapNodePipelineTest, FirstStepSeedsOrientationFromInitialSensorPose) {
    FakeImu imu;
    FakeMag mag;
    FakeBaro baro;
    const Quaternion truth = Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, 90.0f);
    const Vec3 worldGravity{0.0f, 0.0f, 1.0f};
    const Vec3 worldMag{20.0f, 0.0f, -40.0f};
    const Vec3 bodyGravity = truth.rotateVector(worldGravity);
    const Vec3 bodyMag = truth.rotateVector(worldMag);
    imu.accel = {bodyGravity.x, bodyGravity.y, bodyGravity.z};
    mag.mag = {bodyMag.x, bodyMag.y, bodyMag.z};

    MocapNodePipeline pipe(imu, &mag, &baro);

    MocapNodeSample sample{};
    ASSERT_TRUE(pipe.step(sample));
    ASSERT_TRUE(sample.hasMag);

    const Quaternion relative = truth.conjugate().multiply(sample.orientation);
    const float clampedW = std::fmax(-1.0f, std::fmin(1.0f, relative.w));
    const float errorDeg =
        std::fabs(2.0f * std::acos(clampedW) * 180.0f / 3.14159265358979323846f);

    EXPECT_LT(errorDeg, 10.0f);
}

} // namespace
} // namespace sf
