#include "MocapBleTransport.hpp"

namespace sf {

MocapBleTransport::MocapBleTransport(NotifyFn notifyFn, void* context)
    : MocapBleTransport(notifyFn, context, Config{})
{}

MocapBleTransport::MocapBleTransport(NotifyFn notifyFn, void* context, const Config& cfg)
    : impl_(RuntimeNotifier{notifyFn, context}, cfg)
{}

bool MocapBleTransport::canSendQuaternion() const {
    return impl_.canSendQuaternion();
}

bool MocapBleTransport::sendQuaternion(uint8_t nodeId,
                                       uint64_t timestampUs,
                                       const Quaternion& q) {
    return impl_.sendQuaternion(nodeId, timestampUs, q);
}

uint32_t MocapBleTransport::sentCount() const {
    return impl_.sentCount();
}

uint32_t MocapBleTransport::droppedCount() const {
    return impl_.droppedCount();
}

uint32_t MocapBleTransport::retryCount() const {
    return impl_.retryCount();
}

} // namespace sf
