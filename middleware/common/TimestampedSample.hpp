#pragma once

#include <cstdint>

namespace sf {

template<typename T>
struct TimestampedSample {
    T data;
    uint64_t timestampUs;  // microseconds since boot
};

} // namespace sf
