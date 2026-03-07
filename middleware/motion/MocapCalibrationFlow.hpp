#pragma once

#include "Quaternion.hpp"
#include "SegmentCalibration.hpp"
#include <cstdint>

namespace sf {

class MocapCalibrationFlow {
public:
    enum class Command : uint8_t {
        None = 0,
        CaptureStationary = 1,
        CaptureTPose = 2,
        Reset = 3,
    };

    enum class State : uint8_t {
        Uncalibrated = 0,
        StationaryCalibrated = 1,
        FullyCalibrated = 2,
    };

    void issue(Command command);
    bool processSample(const Quaternion& orientation);
    Quaternion apply(const Quaternion& orientation) const;

    State state() const { return state_; }
    bool hasPendingCommand() const { return pending_ != Command::None; }

private:
    Command pending_ = Command::None;
    State state_ = State::Uncalibrated;
    SegmentCalibration stationaryRef_{};
    SegmentCalibration tposeRef_{};
};

} // namespace sf
