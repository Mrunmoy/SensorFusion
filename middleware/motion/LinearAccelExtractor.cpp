#include "LinearAccelExtractor.hpp"

namespace sf {

LinearAccelExtractor::LinearAccelExtractor(float gravityMagnitude)
    : gravityMag_(gravityMagnitude) {}

AccelData LinearAccelExtractor::extract(const Quaternion& orientation,
                                         const AccelData& rawAccel) const {
    // Gravity in world frame: (0, 0, -g)
    // Rotate to body frame using inverse orientation
    Vec3 gravityWorld{0.0f, 0.0f, -gravityMag_};
    Vec3 gravityBody = orientation.inverse().rotateVector(gravityWorld);

    // Linear accel in body frame
    Vec3 linearBody{rawAccel.x - gravityBody.x,
                    rawAccel.y - gravityBody.y,
                    rawAccel.z - gravityBody.z};

    // Rotate to world frame
    Vec3 linearWorld = orientation.rotateVector(linearBody);
    return {linearWorld.x, linearWorld.y, linearWorld.z};
}

AccelData LinearAccelExtractor::extractBodyFrame(const Quaternion& orientation,
                                                  const AccelData& rawAccel) const {
    Vec3 gravityWorld{0.0f, 0.0f, -gravityMag_};
    Vec3 gravityBody = orientation.inverse().rotateVector(gravityWorld);

    return {rawAccel.x - gravityBody.x,
            rawAccel.y - gravityBody.y,
            rawAccel.z - gravityBody.z};
}

} // namespace sf
