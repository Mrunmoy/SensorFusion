#pragma once

#include "Quaternion.hpp"
#include "Vec3.hpp"
#include <cstdint>
#include <cstddef>

namespace sf {

struct Bone {
    uint8_t parentIdx;  // 0xFF = root (no parent)
    uint8_t nodeId;     // which IMU node drives this bone
    // 2 bytes padding before float (total sizeof(Bone) == 8)
    float length;       // bone length in meters
};

struct JointPose {
    Vec3 position;
    Quaternion orientation;
};

// Not thread-safe. All methods must be called from a single task/thread.
class ForwardKinematics {
public:
    static constexpr size_t MAX_BONES = 16;
    static constexpr uint8_t ROOT = 0xFF;

    bool configure(const Bone* bones, size_t count);
    bool setNodeOrientation(uint8_t nodeId, const Quaternion& orientation);
    void solve();
    const JointPose& joint(size_t index) const;
    size_t boneCount() const;
    void reset();

private:
    Bone bones_[MAX_BONES]{};
    Quaternion nodeOrientations_[MAX_BONES]{};
    JointPose joints_[MAX_BONES]{};
    size_t count_ = 0;
};

} // namespace sf
