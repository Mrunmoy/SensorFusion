#include "MocapNodePipeline.hpp"
#include <gtest/gtest.h>

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

} // namespace
} // namespace sf
