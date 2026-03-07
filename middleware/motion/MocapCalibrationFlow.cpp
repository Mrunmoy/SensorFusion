#include "MocapCalibrationFlow.hpp"

namespace sf {

void MocapCalibrationFlow::issue(Command command) {
    if (command == Command::Reset) {
        pending_ = Command::None;
        state_ = State::Uncalibrated;
        stationaryRef_.reset();
        tposeRef_.reset();
        return;
    }

    if (command == Command::CaptureTPose && state_ == State::Uncalibrated) {
        // Require stationary capture first so operator workflow is deterministic.
        return;
    }
    pending_ = command;
}

bool MocapCalibrationFlow::processSample(const Quaternion& orientation) {
    if (pending_ == Command::None) {
        return false;
    }

    if (pending_ == Command::CaptureStationary) {
        stationaryRef_.captureReference(orientation);
        tposeRef_.reset();
        state_ = State::StationaryCalibrated;
    } else if (pending_ == Command::CaptureTPose && state_ != State::Uncalibrated) {
        tposeRef_.captureReference(orientation);
        state_ = State::FullyCalibrated;
    } else {
        pending_ = Command::None;
        return false;
    }

    pending_ = Command::None;
    return true;
}

Quaternion MocapCalibrationFlow::apply(const Quaternion& orientation) const {
    if (state_ == State::FullyCalibrated) {
        return tposeRef_.segmentOrientation(orientation);
    }
    if (state_ == State::StationaryCalibrated) {
        return stationaryRef_.segmentOrientation(orientation);
    }
    return orientation;
}

} // namespace sf
