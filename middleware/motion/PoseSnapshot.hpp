#pragma once

#include "Quaternion.hpp"
#include "Vec3.hpp"
#include <cstdint>
#include <cstddef>

namespace sf {

static constexpr size_t MAX_JOINTS = 16;

struct PoseSnapshot {
    Vec3 positions[MAX_JOINTS]{};
    Quaternion orientations[MAX_JOINTS]{};
    uint64_t timestampUs = 0;
    uint8_t activeJoints = 0;
};

} // namespace sf
