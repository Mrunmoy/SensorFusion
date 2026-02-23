#pragma once

#include <cmath>

namespace sf {

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    void normalize() {
        float n = norm();
        if (n < 1e-10f) return;
        float inv = 1.0f / n;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
    }

    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }

    Quaternion multiply(const Quaternion& r) const {
        return {
            w * r.w - x * r.x - y * r.y - z * r.z,
            w * r.x + x * r.w + y * r.z - z * r.y,
            w * r.y - x * r.z + y * r.w + z * r.x,
            w * r.z + x * r.y - y * r.x + z * r.w
        };
    }

    // Convert to Euler angles in degrees (ZYX convention)
    // roll  = rotation about X
    // pitch = rotation about Y
    // yaw   = rotation about Z
    void toEuler(float& roll, float& pitch, float& yaw) const {
        constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

        // Roll (X)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

        // Pitch (Y) — clamp for gimbal lock
        float sinp = 2.0f * (w * y - z * x);
        if (sinp >= 1.0f)
            pitch = 90.0f;
        else if (sinp <= -1.0f)
            pitch = -90.0f;
        else
            pitch = std::asin(sinp) * RAD_TO_DEG;

        // Yaw (Z)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    }
};

} // namespace sf
