#include "ForwardKinematics.hpp"

namespace sf {

bool ForwardKinematics::configure(const Bone* bones, size_t count) {
    if (count > MAX_BONES) return false;
    if (count > 0 && bones == nullptr) return false;
    for (size_t i = 0; i < count; ++i) {
        if (bones[i].parentIdx != ROOT && bones[i].parentIdx >= i)
            return false;  // must be in topological order
        bones_[i] = bones[i];
    }
    count_ = count;
    return true;
}

bool ForwardKinematics::setNodeOrientation(uint8_t nodeId,
                                            const Quaternion& orientation) {
    for (size_t i = 0; i < count_; ++i) {
        if (bones_[i].nodeId == nodeId) {
            nodeOrientations_[i] = orientation;
            return true;
        }
    }
    return false;
}

void ForwardKinematics::solve() {
    for (size_t i = 0; i < count_; ++i) {
        joints_[i].orientation = nodeOrientations_[i];

        if (bones_[i].parentIdx == ROOT) {
            joints_[i].position = {0.0f, 0.0f, 0.0f};
        } else {
            uint8_t p = bones_[i].parentIdx;
            // Parent bone extends along local +Y axis from parent to this joint
            Vec3 boneVec{0.0f, bones_[p].length, 0.0f};
            Vec3 rotated = joints_[p].orientation.rotateVector(boneVec);
            joints_[i].position = joints_[p].position + rotated;
        }
    }
}

const JointPose& ForwardKinematics::joint(size_t index) const {
    if (index >= count_) index = (count_ > 0) ? count_ - 1 : 0;
    return joints_[index];
}

size_t ForwardKinematics::boneCount() const {
    return count_;
}

void ForwardKinematics::reset() {
    count_ = 0;
    for (size_t i = 0; i < MAX_BONES; ++i) {
        bones_[i] = {};
        nodeOrientations_[i] = {1.0f, 0.0f, 0.0f, 0.0f};
        joints_[i] = {};
    }
}

} // namespace sf
