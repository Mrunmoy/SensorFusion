#pragma once

#include <cmath>
#include "Vec3.hpp"

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

    Quaternion inverse() const {
        return conjugate();
    }

    Vec3 rotateVector(const Vec3& v) const {
        // Optimized q*v*q^-1 using cross-product form:
        // v' = v + 2w*(qv x v) + 2*(qv x (qv x v))
        Vec3 qv{x, y, z};
        Vec3 t{
            2.0f * (qv.y * v.z - qv.z * v.y),
            2.0f * (qv.z * v.x - qv.x * v.z),
            2.0f * (qv.x * v.y - qv.y * v.x)
        };
        return {
            v.x + w * t.x + (qv.y * t.z - qv.z * t.y),
            v.y + w * t.y + (qv.z * t.x - qv.x * t.z),
            v.z + w * t.z + (qv.x * t.y - qv.y * t.x)
        };
    }

    void toRotationMatrix(float mat[9]) const {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        mat[0] = 1.0f - 2.0f * (yy + zz);
        mat[1] = 2.0f * (xy - wz);
        mat[2] = 2.0f * (xz + wy);
        mat[3] = 2.0f * (xy + wz);
        mat[4] = 1.0f - 2.0f * (xx + zz);
        mat[5] = 2.0f * (yz - wx);
        mat[6] = 2.0f * (xz - wy);
        mat[7] = 2.0f * (yz + wx);
        mat[8] = 1.0f - 2.0f * (xx + yy);
    }

    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t) {
        float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        Quaternion b2 = b;
        if (dot < 0.0f) {
            dot = -dot;
            b2 = {-b.w, -b.x, -b.y, -b.z};
        }
        if (dot > 0.9995f) {
            Quaternion r{
                a.w + t * (b2.w - a.w),
                a.x + t * (b2.x - a.x),
                a.y + t * (b2.y - a.y),
                a.z + t * (b2.z - a.z)
            };
            r.normalize();
            return r;
        }
        float theta = std::acos(dot);
        float sinTheta = std::sin(theta);
        float wa = std::sin((1.0f - t) * theta) / sinTheta;
        float wb = std::sin(t * theta) / sinTheta;
        return {
            wa * a.w + wb * b2.w,
            wa * a.x + wb * b2.x,
            wa * a.y + wb * b2.y,
            wa * a.z + wb * b2.z
        };
    }

    static Quaternion fromAxisAngle(float ax, float ay, float az, float angleDeg) {
        constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        float halfRad = angleDeg * DEG_TO_RAD * 0.5f;
        float len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-10f) return {1.0f, 0.0f, 0.0f, 0.0f};
        float inv = 1.0f / len;
        float s = std::sin(halfRad);
        return {std::cos(halfRad), ax * inv * s, ay * inv * s, az * inv * s};
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
