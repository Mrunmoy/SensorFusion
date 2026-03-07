#pragma once

#include <cstdint>

namespace sf {

class TimestampSync {
public:
    struct Config {
        float alpha = 0.2f;
        int64_t maxDriftStepUs = 500000;
    };

    TimestampSync() = default;
    explicit TimestampSync(const Config& cfg)
        : cfg_(cfg)
    {}

    void observeAnchor(uint64_t localTimestampUs, uint64_t remoteTimestampUs);
    uint64_t toRemoteTimeUs(uint64_t localTimestampUs) const;

    bool isLocked() const { return locked_; }
    int64_t offsetUs() const { return offsetUs_; }

private:
    Config cfg_{};
    bool locked_ = false;
    int64_t offsetUs_ = 0;
};

} // namespace sf
