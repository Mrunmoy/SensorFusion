#include "MahonyAHRS.hpp"
#include <cmath>

namespace sf {

namespace {
    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

    Quaternion quaternionFromEulerDeg(float rollDeg, float pitchDeg, float yawDeg) {
        const float halfRoll = rollDeg * DEG_TO_RAD * 0.5f;
        const float halfPitch = pitchDeg * DEG_TO_RAD * 0.5f;
        const float halfYaw = yawDeg * DEG_TO_RAD * 0.5f;

        const float cr = std::cos(halfRoll);
        const float sr = std::sin(halfRoll);
        const float cp = std::cos(halfPitch);
        const float sp = std::sin(halfPitch);
        const float cy = std::cos(halfYaw);
        const float sy = std::sin(halfYaw);

        Quaternion q{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        };
        q.normalize();
        return q;
    }
}

MahonyAHRS::MahonyAHRS(float kp, float ki)
    : kp_(kp), ki_(ki)
{}

void MahonyAHRS::update(const AccelData& a, const GyroData& g,
                         const MagData& m, float dt) {
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

    // Convert gyro from deg/s to rad/s
    float gx = g.x * DEG_TO_RAD;
    float gy = g.y * DEG_TO_RAD;
    float gz = g.z * DEG_TO_RAD;

    // Normalize accelerometer
    float ax = a.x, ay = a.y, az = a.z;
    float normA = invSqrt(ax * ax + ay * ay + az * az);
    if (normA == 0.0f) {
        update6DOF(a, g, dt);
        return;
    }
    ax *= normA; ay *= normA; az *= normA;

    // Normalize magnetometer
    float mx = m.x, my = m.y, mz = m.z;
    float normM = invSqrt(mx * mx + my * my + mz * mz);
    if (normM == 0.0f) {
        update6DOF(a, g, dt);
        return;
    }
    mx *= normM; my *= normM; mz *= normM;

    // Reference direction of Earth's magnetic field (rotate mag into earth frame)
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
                        my * (q1 * q2 - q0 * q3) +
                        mz * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) +
                        my * (0.5f - q1 * q1 - q3 * q3) +
                        mz * (q2 * q3 - q0 * q1));
    float bx = std::sqrt(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) +
                        my * (q2 * q3 + q0 * q1) +
                        mz * (0.5f - q1 * q1 - q2 * q2));

    // Estimated direction of gravity and magnetic field
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
    float wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3));
    float wz = 2.0f * (bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2));

    // Error is cross product between estimated and measured directions
    float ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    float ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    float ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    // Apply integral feedback
    if (ki_ > 0.0f) {
        integralX_ += ki_ * ex * dt;
        integralY_ += ki_ * ey * dt;
        integralZ_ += ki_ * ez * dt;
        gx += integralX_;
        gy += integralY_;
        gz += integralZ_;
    }

    // Apply proportional feedback
    gx += kp_ * ex;
    gy += kp_ * ey;
    gz += kp_ * ez;

    // Integrate quaternion rate
    float halfDt = 0.5f * dt;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

    // Normalize quaternion
    q_.w = q0; q_.x = q1; q_.y = q2; q_.z = q3;
    q_.normalize();
}

void MahonyAHRS::update6DOF(const AccelData& a, const GyroData& g, float dt) {
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

    // Convert gyro from deg/s to rad/s
    float gx = g.x * DEG_TO_RAD;
    float gy = g.y * DEG_TO_RAD;
    float gz = g.z * DEG_TO_RAD;

    // Normalize accelerometer
    float ax = a.x, ay = a.y, az = a.z;
    float normA = invSqrt(ax * ax + ay * ay + az * az);
    if (normA > 0.0f) {
        ax *= normA; ay *= normA; az *= normA;

        // Estimated direction of gravity
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error is cross product between estimated and measured gravity
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // Apply integral feedback
        if (ki_ > 0.0f) {
            integralX_ += ki_ * ex * dt;
            integralY_ += ki_ * ey * dt;
            integralZ_ += ki_ * ez * dt;
            gx += integralX_;
            gy += integralY_;
            gz += integralZ_;
        }

        // Apply proportional feedback
        gx += kp_ * ex;
        gy += kp_ * ey;
        gz += kp_ * ez;
    }

    // Integrate quaternion rate
    float halfDt = 0.5f * dt;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

    // Normalize quaternion
    q_.w = q0; q_.x = q1; q_.y = q2; q_.z = q3;
    q_.normalize();
}

void MahonyAHRS::initFromSensors(const AccelData& a, const MagData& m) {
    const float accelNorm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (accelNorm <= 0.0f) {
        reset();
        return;
    }

    const float ax = a.x / accelNorm;
    const float ay = a.y / accelNorm;
    const float az = a.z / accelNorm;

    // The HelixDrift simulator uses gravity along +Z at identity. These
    // formulas map the measured gravity vector into the filter's ZYX Euler
    // convention.
    const float roll = std::atan2(-ay, az);
    const float pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));

    const float magNorm = std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    if (magNorm <= 0.0f) {
        q_ = quaternionFromEulerDeg(roll * RAD_TO_DEG, pitch * RAD_TO_DEG, 0.0f);
        integralX_ = 0.0f;
        integralY_ = 0.0f;
        integralZ_ = 0.0f;
        return;
    }

    const float mx = m.x / magNorm;
    const float my = m.y / magNorm;
    const float mz = m.z / magNorm;

    const float sinRoll = std::sin(roll);
    const float cosRoll = std::cos(roll);
    const float sinPitch = std::sin(pitch);
    const float cosPitch = std::cos(pitch);

    const float mxLevel = mx * cosPitch + mz * sinPitch;
    const float myLevel =
        mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
    const float yaw = std::atan2(myLevel, mxLevel);

    q_ = quaternionFromEulerDeg(roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);
    integralX_ = 0.0f;
    integralY_ = 0.0f;
    integralZ_ = 0.0f;
}

void MahonyAHRS::initFromAccel(const AccelData& a) {
    const float accelNorm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (accelNorm <= 0.0f) {
        reset();
        return;
    }

    const float ax = a.x / accelNorm;
    const float ay = a.y / accelNorm;
    const float az = a.z / accelNorm;

    const float roll = std::atan2(-ay, az);
    const float pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));

    q_ = quaternionFromEulerDeg(roll * RAD_TO_DEG, pitch * RAD_TO_DEG, 0.0f);
    integralX_ = 0.0f;
    integralY_ = 0.0f;
    integralZ_ = 0.0f;
}

Quaternion MahonyAHRS::getQuaternion() const {
    return q_;
}

void MahonyAHRS::getEulerDeg(float& roll, float& pitch, float& yaw) const {
    q_.toEuler(roll, pitch, yaw);
}

void MahonyAHRS::reset() {
    q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    integralX_ = 0.0f;
    integralY_ = 0.0f;
    integralZ_ = 0.0f;
}

float MahonyAHRS::invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    return 1.0f / std::sqrt(x);
}

} // namespace sf
