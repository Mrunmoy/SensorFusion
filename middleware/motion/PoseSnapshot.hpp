#pragma once

#include "Quaternion.hpp"
#include "Vec3.hpp"
#include "ForwardKinematics.hpp"
#include <cstdint>
#include <cstddef>

namespace sf {

struct PoseSnapshot {
    static constexpr size_t MAX_JOINTS = ForwardKinematics::MAX_BONES;

    Vec3 positions[MAX_JOINTS]{};
    Quaternion orientations[MAX_JOINTS]{};
    uint64_t timestampUs = 0;
    uint8_t activeJoints = 0;
};

} // namespace sf
