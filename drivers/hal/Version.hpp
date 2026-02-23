#pragma once

#include <cstdint>

namespace sf {

struct Version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

// Driver library version
constexpr Version DRIVER_VERSION = {1, 0, 0};

// Middleware library version
constexpr Version MIDDLEWARE_VERSION = {1, 0, 0};

// Firmware version (updated by application)
constexpr Version FIRMWARE_VERSION = {0, 1, 0};

} // namespace sf
