#pragma once

#include "SensorTypes.hpp"
#include "Quaternion.hpp"

namespace sf {

class LinearAccelExtractor {
public:
    explicit LinearAccelExtractor(float gravityMagnitude = 1.0f);

    AccelData extract(const Quaternion& orientation, const AccelData& rawAccel) const;
    AccelData extractBodyFrame(const Quaternion& orientation, const AccelData& rawAccel) const;

private:
    float gravityMag_;
};

} // namespace sf
