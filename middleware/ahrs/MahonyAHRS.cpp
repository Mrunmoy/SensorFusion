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

    Vec3 normalizeOrZero(const Vec3& v) {
        const float len = v.length();
        if (len <= 0.0f) {
            return {};
        }
        const float invLen = 1.0f / len;
        return {v.x * invLen, v.y * invLen, v.z * invLen};
    }

    Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
    }

    Quaternion quaternionFromColumns(const Vec3& xAxis, const Vec3& yAxis, const Vec3& zAxis) {
        const float m00 = xAxis.x;
        const float m10 = xAxis.y;
        const float m20 = xAxis.z;
        const float m01 = yAxis.x;
        const float m11 = yAxis.y;
        const float m21 = yAxis.z;
        const float m02 = zAxis.x;
        const float m12 = zAxis.y;
        const float m22 = zAxis.z;

        Quaternion q{};
        const float trace = m00 + m11 + m22;
        if (trace > 0.0f) {
            const float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        } else if (m11 > m22) {
            const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        } else {
            const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }
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

    // Estimate body-frame gravity and magnetic field using the same quaternion
    // convention as Quaternion::rotateVector().
    const Quaternion q{q0, q1, q2, q3};
    const Vec3 gravityEstimate = q.rotateVector(Vec3{0.0f, 0.0f, 1.0f});

    const Vec3 magBody{mx, my, mz};
    const Vec3 magEarth = q.conjugate().rotateVector(magBody);
    const float bx = std::sqrt(magEarth.x * magEarth.x + magEarth.y * magEarth.y);
    const float bz = magEarth.z;
    const Vec3 magEstimate = q.rotateVector(Vec3{bx, 0.0f, bz});

    const float vx = gravityEstimate.x;
    const float vy = gravityEstimate.y;
    const float vz = gravityEstimate.z;
    const float wx = magEstimate.x;
    const float wy = magEstimate.y;
    const float wz = magEstimate.z;

    // Error is cross product between estimated and measured directions
    float ex = (ay * vz - az * vy) + (mz * wy - my * wz);
    float ey = (az * vx - ax * vz) + (mx * wz - mz * wx);
    float ez = (ax * vy - ay * vx) + (my * wx - mx * wy);

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

        const Quaternion q{q0, q1, q2, q3};
        const Vec3 gravityEstimate = q.rotateVector(Vec3{0.0f, 0.0f, 1.0f});
        const float vx = gravityEstimate.x;
        const float vy = gravityEstimate.y;
        const float vz = gravityEstimate.z;

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

    const Vec3 bodyZ = normalizeOrZero(Vec3{a.x, a.y, a.z});

    const float magNorm = std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    if (magNorm <= 0.0f) {
        initFromAccel(a);
        integralX_ = 0.0f;
        integralY_ = 0.0f;
        integralZ_ = 0.0f;
        return;
    }

    const Vec3 bodyMag = normalizeOrZero(Vec3{m.x, m.y, m.z});
    const Vec3 bodyMagHorizontal = bodyMag - bodyZ * bodyMag.dot(bodyZ);
    const Vec3 bodyX = normalizeOrZero(bodyMagHorizontal);
    if (bodyX.lengthSq() <= 0.0f) {
        initFromAccel(a);
        integralX_ = 0.0f;
        integralY_ = 0.0f;
        integralZ_ = 0.0f;
        return;
    }
    const Vec3 bodyY = normalizeOrZero(cross(bodyZ, bodyX));

    q_ = quaternionFromColumns(bodyX, bodyY, bodyZ);
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
