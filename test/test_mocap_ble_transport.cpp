#include "MocapBleTransport.hpp"
#include <gtest/gtest.h>

namespace sf {
namespace {

struct CallbackContext {
    int calls = 0;
    int failBeforeSuccess = 0;
    size_t lastLen = 0;
    uint8_t lastFrame[FrameCodec::MAX_FRAME_SIZE]{};
};

bool notifyStub(const uint8_t* data, size_t len, void* context) {
    auto* ctx = static_cast<CallbackContext*>(context);
    ctx->calls++;
    ctx->lastLen = len;
    for (size_t i = 0; i < len && i < sizeof(ctx->lastFrame); ++i) {
        ctx->lastFrame[i] = data[i];
    }

    if (ctx->failBeforeSuccess > 0) {
        ctx->failBeforeSuccess--;
        return false;
    }
    return true;
}

TEST(MocapBleTransportTest, RequiredMtuMatchesQuaternionFrame) {
    EXPECT_EQ(MocapBleTransport::QUATERNION_FRAME_BYTES, 30u);
    EXPECT_EQ(MocapBleTransport::REQUIRED_ATT_MTU, 33u);
}

TEST(MocapBleTransportTest, SendQuaternionSuccess) {
    CallbackContext ctx{};
    MocapBleTransport tx(&notifyStub, &ctx);
    Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};

    EXPECT_TRUE(tx.sendQuaternion(7, 123456u, q));
    EXPECT_EQ(tx.sentCount(), 1u);
    EXPECT_EQ(tx.droppedCount(), 0u);
    EXPECT_EQ(tx.retryCount(), 0u);
    EXPECT_EQ(ctx.calls, 1);
    EXPECT_EQ(ctx.lastLen, MocapBleTransport::QUATERNION_FRAME_BYTES);

    FrameCodec::FrameHeader hdr{};
    uint8_t payload[32]{};
    size_t payloadLen = 0;
    ASSERT_TRUE(FrameCodec::decode(ctx.lastFrame, ctx.lastLen, hdr, payload, payloadLen));
    EXPECT_EQ(hdr.nodeId, 7u);
    EXPECT_EQ(hdr.timestampUs, 123456u);
    EXPECT_EQ(hdr.type, SensorType::QUATERNION);
}

TEST(MocapBleTransportTest, RetryThenSuccess) {
    CallbackContext ctx{};
    ctx.failBeforeSuccess = 1;
    MocapBleTransport::Config cfg{};
    cfg.maxRetries = 2;
    MocapBleTransport tx(&notifyStub, &ctx, cfg);

    EXPECT_TRUE(tx.sendQuaternion(1, 42u, Quaternion{}));
    EXPECT_EQ(ctx.calls, 2);
    EXPECT_EQ(tx.sentCount(), 1u);
    EXPECT_EQ(tx.retryCount(), 1u);
    EXPECT_EQ(tx.droppedCount(), 0u);
}

TEST(MocapBleTransportTest, DropWhenRetriesExhausted) {
    CallbackContext ctx{};
    ctx.failBeforeSuccess = 10;
    MocapBleTransport::Config cfg{};
    cfg.maxRetries = 2;
    MocapBleTransport tx(&notifyStub, &ctx, cfg);

    EXPECT_FALSE(tx.sendQuaternion(1, 42u, Quaternion{}));
    EXPECT_EQ(ctx.calls, 3);
    EXPECT_EQ(tx.sentCount(), 0u);
    EXPECT_EQ(tx.retryCount(), 2u);
    EXPECT_EQ(tx.droppedCount(), 1u);
}

TEST(MocapBleTransportTest, TooSmallMtuDropsWithoutSend) {
    CallbackContext ctx{};
    MocapBleTransport::Config cfg{};
    cfg.attMtu = 32;
    MocapBleTransport tx(&notifyStub, &ctx, cfg);

    EXPECT_FALSE(tx.canSendQuaternion());
    EXPECT_FALSE(tx.sendQuaternion(1, 1u, Quaternion{}));
    EXPECT_EQ(ctx.calls, 0);
    EXPECT_EQ(tx.droppedCount(), 1u);
}

TEST(MocapBleTransportTest, NullCallbackDrops) {
    MocapBleTransport tx(nullptr, nullptr);
    EXPECT_FALSE(tx.canSendQuaternion());
    EXPECT_FALSE(tx.sendQuaternion(1, 1u, Quaternion{}));
    EXPECT_EQ(tx.sentCount(), 0u);
    EXPECT_EQ(tx.droppedCount(), 1u);
}

} // namespace
} // namespace sf
