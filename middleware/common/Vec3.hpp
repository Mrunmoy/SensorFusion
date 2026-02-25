#pragma once

#include <cmath>

namespace sf {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    float dot(const Vec3& r) const { return x * r.x + y * r.y + z * r.z; }
    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }
};

} // namespace sf
