#include "TimestampSync.hpp"

namespace sf {

void TimestampSync::observeAnchor(uint64_t localTimestampUs, uint64_t remoteTimestampUs) {
    const int64_t measuredOffset =
        static_cast<int64_t>(remoteTimestampUs) - static_cast<int64_t>(localTimestampUs);

    if (!locked_) {
        offsetUs_ = measuredOffset;
        locked_ = true;
        return;
    }

    const int64_t delta = measuredOffset - offsetUs_;
    const int64_t absDelta = (delta >= 0) ? delta : -delta;
    if (absDelta > cfg_.maxDriftStepUs) {
        offsetUs_ = measuredOffset;
        return;
    }

    offsetUs_ += static_cast<int64_t>(cfg_.alpha * static_cast<float>(delta));
}

uint64_t TimestampSync::toRemoteTimeUs(uint64_t localTimestampUs) const {
    if (!locked_) {
        return localTimestampUs;
    }
    const int64_t mapped = static_cast<int64_t>(localTimestampUs) + offsetUs_;
    return (mapped <= 0) ? 0ull : static_cast<uint64_t>(mapped);
}

} // namespace sf
