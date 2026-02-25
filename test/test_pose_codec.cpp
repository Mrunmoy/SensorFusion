#include <gtest/gtest.h>
#include "FrameCodec.hpp"
#include <cstring>

using namespace sf;

TEST(PoseCodecTest, PoseRoundTrip) {
    Quaternion q{0.707f, 0.0f, 0.707f, 0.0f};
    uint8_t buf[64];
    size_t len = FrameCodec::encodePose(0x03, 99999ULL,
                                         1.5f, -0.3f, 0.8f, q,
                                         buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::POSE);
    EXPECT_EQ(hdr.nodeId, 0x03);
    EXPECT_EQ(hdr.timestampUs, 99999ULL);
    EXPECT_EQ(payloadLen, 28u);

    // Verify payload floats
    auto readF = [&](size_t offset) {
        float v;
        uint32_t bits = static_cast<uint32_t>(payload[offset]) |
                        (static_cast<uint32_t>(payload[offset+1]) << 8) |
                        (static_cast<uint32_t>(payload[offset+2]) << 16) |
                        (static_cast<uint32_t>(payload[offset+3]) << 24);
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    };

    EXPECT_FLOAT_EQ(readF(0), 1.5f);     // posX
    EXPECT_FLOAT_EQ(readF(4), -0.3f);    // posY
    EXPECT_FLOAT_EQ(readF(8), 0.8f);     // posZ
    EXPECT_FLOAT_EQ(readF(12), 0.707f);  // q.w
    EXPECT_FLOAT_EQ(readF(16), 0.0f);    // q.x
    EXPECT_FLOAT_EQ(readF(20), 0.707f);  // q.y
    EXPECT_FLOAT_EQ(readF(24), 0.0f);    // q.z
}

TEST(PoseCodecTest, PoseFrameSize) {
    Quaternion q{1, 0, 0, 0};
    uint8_t buf[64];
    size_t len = FrameCodec::encodePose(0x01, 0, 0, 0, 0, q, buf, sizeof(buf));
    EXPECT_EQ(len, 42u);  // header(12) + payload(28) + crc(2)
}

TEST(PoseCodecTest, PoseBufferTooSmall) {
    Quaternion q{1, 0, 0, 0};
    uint8_t buf[10];  // too small
    size_t len = FrameCodec::encodePose(0x01, 0, 0, 0, 0, q, buf, sizeof(buf));
    EXPECT_EQ(len, 0u);
}

TEST(PoseCodecTest, PoseNodeIdPreserved) {
    Quaternion q{1, 0, 0, 0};
    uint8_t buf[64];
    size_t len = FrameCodec::encodePose(0xAB, 0, 0, 0, 0, q, buf, sizeof(buf));
    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
    EXPECT_EQ(hdr.nodeId, 0xAB);
}

TEST(PoseCodecTest, PoseTimestampPreserved) {
    Quaternion q{1, 0, 0, 0};
    uint8_t buf[64];
    uint64_t ts = 0xDEADBEEFCAFE;
    size_t len = FrameCodec::encodePose(0x01, ts, 0, 0, 0, q, buf, sizeof(buf));
    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
    EXPECT_EQ(hdr.timestampUs, ts);
}
