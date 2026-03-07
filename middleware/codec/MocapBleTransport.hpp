#pragma once

#include "FrameCodec.hpp"

namespace sf {

class MocapBleTransport {
public:
    using NotifyFn = bool (*)(const uint8_t* data, size_t len, void* context);

    struct Config {
        size_t attMtu = 185;
        uint8_t maxRetries = 2;
    };

    static constexpr size_t ATT_OVERHEAD_BYTES = 3;
    static constexpr size_t QUATERNION_FRAME_BYTES =
        FrameCodec::HEADER_SIZE + 16 + FrameCodec::CRC_SIZE;
    static constexpr size_t REQUIRED_ATT_MTU =
        QUATERNION_FRAME_BYTES + ATT_OVERHEAD_BYTES;

    explicit MocapBleTransport(NotifyFn notifyFn, void* context);
    MocapBleTransport(NotifyFn notifyFn, void* context, const Config& cfg);

    bool canSendQuaternion() const;
    bool sendQuaternion(uint8_t nodeId, uint64_t timestampUs, const Quaternion& q);

    uint32_t sentCount() const { return sentCount_; }
    uint32_t droppedCount() const { return droppedCount_; }
    uint32_t retryCount() const { return retryCount_; }

private:
    NotifyFn notifyFn_;
    void* context_;
    Config cfg_;
    uint8_t frameBuf_[FrameCodec::MAX_FRAME_SIZE]{};

    uint32_t sentCount_ = 0;
    uint32_t droppedCount_ = 0;
    uint32_t retryCount_ = 0;
};

} // namespace sf
