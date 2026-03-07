#pragma once

#include "CalibrationStore.hpp"
#include <cmath>
#include <cstddef>

namespace sf {

class CalibrationFitter {
public:
    // Fit hard-iron offsets and diagonal soft-iron scale for magnetometer samples.
    // Returns false when samples are missing, non-finite, or degenerate.
    static bool fitMagHardSoftIron(const MagData* samples, size_t count, CalibrationData& out) {
        if (!samples || count < 6) return false;

        float minX = samples[0].x, maxX = samples[0].x;
        float minY = samples[0].y, maxY = samples[0].y;
        float minZ = samples[0].z, maxZ = samples[0].z;

        for (size_t i = 0; i < count; ++i) {
            const float x = samples[i].x;
            const float y = samples[i].y;
            const float z = samples[i].z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;

            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
        }

        const float hx = 0.5f * (maxX - minX);
        const float hy = 0.5f * (maxY - minY);
        const float hz = 0.5f * (maxZ - minZ);
        constexpr float kMinHalfRange = 1e-3f;
        if (hx < kMinHalfRange || hy < kMinHalfRange || hz < kMinHalfRange) return false;

        out.offsetX = 0.5f * (maxX + minX);
        out.offsetY = 0.5f * (maxY + minY);
        out.offsetZ = 0.5f * (maxZ + minZ);

        const float avgHalf = (hx + hy + hz) / 3.0f;
        out.scaleX = avgHalf / hx;
        out.scaleY = avgHalf / hy;
        out.scaleZ = avgHalf / hz;
        return CalibrationStore::isCalibrationSane(out);
    }
};

} // namespace sf
