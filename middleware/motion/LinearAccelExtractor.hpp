#pragma once

#include "SensorTypes.hpp"
#include "Quaternion.hpp"

namespace sf {

// Thread-safe: const methods are safe for concurrent calls.
// Assumes ENU/Z-up world frame with gravity = (0, 0, -g).
class LinearAccelExtractor {
public:
    explicit LinearAccelExtractor(float gravityMagnitude = 1.0f);

    AccelData extract(const Quaternion& orientation, const AccelData& rawAccel) const;
    AccelData extractBodyFrame(const Quaternion& orientation, const AccelData& rawAccel) const;

private:
    float gravityMag_;
};

} // namespace sf
