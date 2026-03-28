#pragma once

#include "SensorTypes.hpp"
#include "Quaternion.hpp"

namespace sf {

class MahonyAHRS {
public:
    MahonyAHRS(float kp = 1.0f, float ki = 0.0f);

    // 9-DOF update: accelerometer + gyroscope + magnetometer
    void update(const AccelData& a, const GyroData& g,
                const MagData& m, float dt);

    // 6-DOF update: accelerometer + gyroscope only
    void update6DOF(const AccelData& a, const GyroData& g, float dt);

    // One-shot initialization from gravity + magnetic field.
    void initFromSensors(const AccelData& a, const MagData& m);

    // One-shot initialization from gravity only. Yaw remains zero.
    void initFromAccel(const AccelData& a);

    Quaternion getQuaternion() const;
    void getEulerDeg(float& roll, float& pitch, float& yaw) const;
    void reset();

private:
    Quaternion q_;
    float integralX_ = 0.0f;
    float integralY_ = 0.0f;
    float integralZ_ = 0.0f;
    float kp_;
    float ki_;

    static float invSqrt(float x);
};

} // namespace sf
