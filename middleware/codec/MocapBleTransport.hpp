#pragma once

#include "FrameCodec.hpp"
#include <utility>

namespace sf {

struct MocapBleTransportConfig {
    size_t attMtu = 185;
    uint8_t maxRetries = 2;
};

constexpr size_t MOCAP_ATT_OVERHEAD_BYTES = 3;
constexpr size_t MOCAP_QUATERNION_FRAME_BYTES =
    FrameCodec::HEADER_SIZE + 16 + FrameCodec::CRC_SIZE;
constexpr size_t MOCAP_REQUIRED_ATT_MTU =
    MOCAP_QUATERNION_FRAME_BYTES + MOCAP_ATT_OVERHEAD_BYTES;

inline bool branchLikely(bool condition) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_expect(condition, 1) != 0;
#else
    return condition;
#endif
}

template <typename Notifier>
class MocapBleTransportT {
public:
    using Config = MocapBleTransportConfig;

    static constexpr size_t ATT_OVERHEAD_BYTES = MOCAP_ATT_OVERHEAD_BYTES;
    static constexpr size_t QUATERNION_FRAME_BYTES = MOCAP_QUATERNION_FRAME_BYTES;
    static constexpr size_t REQUIRED_ATT_MTU = MOCAP_REQUIRED_ATT_MTU;

    explicit MocapBleTransportT(Notifier notifier)
        : notifier_(std::move(notifier))
    {}

    MocapBleTransportT(Notifier notifier, const Config& cfg)
        : notifier_(std::move(notifier)),
          cfg_(cfg)
    {}

    bool canSendQuaternion() const {
        return notifier_.valid() && cfg_.attMtu >= REQUIRED_ATT_MTU;
    }

    bool sendQuaternion(uint8_t nodeId, uint64_t timestampUs, const Quaternion& q) {
        if (!branchLikely(canSendQuaternion())) {
            ++droppedCount_;
            return false;
        }

        const size_t frameLen = FrameCodec::encodeQuaternion(
            nodeId, timestampUs, q, frameBuf_, sizeof(frameBuf_));
        if (!branchLikely(frameLen > 0)) {
            ++droppedCount_;
            return false;
        }

        for (uint8_t attempt = 0; attempt <= cfg_.maxRetries; ++attempt) {
            if (branchLikely(notifier_(frameBuf_, frameLen))) {
                ++sentCount_;
                retryCount_ += attempt;
                return true;
            }
        }

        retryCount_ += cfg_.maxRetries;
        ++droppedCount_;
        return false;
    }

    uint32_t sentCount() const { return sentCount_; }
    uint32_t droppedCount() const { return droppedCount_; }
    uint32_t retryCount() const { return retryCount_; }

private:
    Notifier notifier_;
    Config cfg_{};
    uint8_t frameBuf_[FrameCodec::MAX_FRAME_SIZE]{};

    uint32_t sentCount_ = 0;
    uint32_t droppedCount_ = 0;
    uint32_t retryCount_ = 0;
};

class MocapBleTransport {
public:
    using NotifyFn = bool (*)(const uint8_t* data, size_t len, void* context);
    using Config = MocapBleTransportConfig;

    static constexpr size_t ATT_OVERHEAD_BYTES = MOCAP_ATT_OVERHEAD_BYTES;
    static constexpr size_t QUATERNION_FRAME_BYTES = MOCAP_QUATERNION_FRAME_BYTES;
    static constexpr size_t REQUIRED_ATT_MTU = MOCAP_REQUIRED_ATT_MTU;

    explicit MocapBleTransport(NotifyFn notifyFn, void* context);
    MocapBleTransport(NotifyFn notifyFn, void* context, const Config& cfg);

    bool canSendQuaternion() const;
    bool sendQuaternion(uint8_t nodeId, uint64_t timestampUs, const Quaternion& q);

    uint32_t sentCount() const;
    uint32_t droppedCount() const;
    uint32_t retryCount() const;

private:
    struct RuntimeNotifier {
        NotifyFn fn = nullptr;
        void* context = nullptr;

        bool valid() const { return fn != nullptr; }
        bool operator()(const uint8_t* data, size_t len) const {
            return fn(data, len, context);
        }
    };

    MocapBleTransportT<RuntimeNotifier> impl_;
};

} // namespace sf
