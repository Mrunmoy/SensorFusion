#pragma once

#include "Quaternion.hpp"

namespace sf {

class SegmentCalibration {
public:
    void captureReference(const Quaternion& currentOrientation) {
        qRef_ = currentOrientation;
        calibrated_ = true;
    }

    Quaternion segmentOrientation(const Quaternion& currentOrientation) const {
        if (!calibrated_) return currentOrientation;
        return qRef_.inverse().multiply(currentOrientation);
    }

    bool isCalibrated() const { return calibrated_; }

    void reset() {
        qRef_ = {1.0f, 0.0f, 0.0f, 0.0f};
        calibrated_ = false;
    }

private:
    Quaternion qRef_{1.0f, 0.0f, 0.0f, 0.0f};
    bool calibrated_ = false;
};

} // namespace sf
