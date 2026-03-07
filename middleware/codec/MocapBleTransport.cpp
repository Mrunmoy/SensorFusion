#include "MocapBleTransport.hpp"

namespace sf {

MocapBleTransport::MocapBleTransport(NotifyFn notifyFn, void* context)
    : MocapBleTransport(notifyFn, context, Config{})
{}

MocapBleTransport::MocapBleTransport(NotifyFn notifyFn,
                                     void* context,
                                     const Config& cfg)
    : notifyFn_(notifyFn),
      context_(context),
      cfg_(cfg)
{}

bool MocapBleTransport::canSendQuaternion() const {
    return notifyFn_ != nullptr && cfg_.attMtu >= REQUIRED_ATT_MTU;
}

bool MocapBleTransport::sendQuaternion(uint8_t nodeId,
                                       uint64_t timestampUs,
                                       const Quaternion& q) {
    if (!canSendQuaternion()) {
        ++droppedCount_;
        return false;
    }

    const size_t frameLen = FrameCodec::encodeQuaternion(
        nodeId, timestampUs, q, frameBuf_, sizeof(frameBuf_));
    if (frameLen == 0) {
        ++droppedCount_;
        return false;
    }

    for (uint8_t attempt = 0; attempt <= cfg_.maxRetries; ++attempt) {
        if (notifyFn_(frameBuf_, frameLen, context_)) {
            sentCount_++;
            retryCount_ += attempt;
            return true;
        }
    }

    retryCount_ += cfg_.maxRetries;
    ++droppedCount_;
    return false;
}

} // namespace sf
