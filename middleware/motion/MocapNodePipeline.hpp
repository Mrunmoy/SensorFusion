#pragma once

#include "MahonyAHRS.hpp"
#include "SensorInterface.hpp"

namespace sf {

struct MocapNodeSample {
    Quaternion orientation{};
    AccelData accel{};
    GyroData gyro{};
    MagData mag{};
    float pressureHPa = 0.0f;
    bool hasMag = false;
    bool hasPressure = false;
};

class MocapNodePipeline {
public:
    struct Config {
        float dtSeconds = 0.02f; // 50 Hz output
        float mahonyKp = 1.0f;
        float mahonyKi = 0.0f;
        bool preferMag = true;
    };

    MocapNodePipeline(IAccelGyroSensor& imu,
                      IMagSensor* mag,
                      IBaroSensor* baro);
    MocapNodePipeline(IAccelGyroSensor& imu,
                      IMagSensor* mag,
                      IBaroSensor* baro,
                      const Config& cfg);

    // Not thread-safe: call from a single task or protect externally.
    bool step(MocapNodeSample& out);

private:
    IAccelGyroSensor& imu_;
    IMagSensor* mag_;
    IBaroSensor* baro_;
    Config cfg_;
    MahonyAHRS ahrs_;
};

} // namespace sf
