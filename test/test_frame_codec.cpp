#include <gtest/gtest.h>
#include "FrameCodec.hpp"
#include <cstring>

using namespace sf;

static constexpr size_t BUF_SIZE = 64;

// --- CRC-16 tests ---

TEST(FrameCodecCRCTest, EmptyData) {
    uint16_t crc = FrameCodec::crc16(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFF);  // init value, no data
}

TEST(FrameCodecCRCTest, KnownString) {
    // CRC-16 CCITT of "123456789" = 0x29B1
    const uint8_t data[] = "123456789";
    uint16_t crc = FrameCodec::crc16(data, 9);
    EXPECT_EQ(crc, 0x29B1);
}

TEST(FrameCodecCRCTest, DifferentDataDifferentCRC) {
    uint8_t a[] = {0x01, 0x02};
    uint8_t b[] = {0x01, 0x03};
    EXPECT_NE(FrameCodec::crc16(a, 2), FrameCodec::crc16(b, 2));
}

// --- Accel round-trip ---

TEST(FrameCodecTest, AccelRoundTrip) {
    AccelData accel{1.5f, -2.3f, 9.81f};
    uint8_t buf[BUF_SIZE];

    size_t len = FrameCodec::encodeAccel(0x01, 12345678ULL, accel, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.nodeId, 0x01);
    EXPECT_EQ(hdr.type, SensorType::ACCEL);
    EXPECT_EQ(hdr.timestampUs, 12345678ULL);
    EXPECT_EQ(payloadLen, 12u);

    // Verify payload floats
    float x, y, z;
    std::memcpy(&x, &payload[0], 4);
    std::memcpy(&y, &payload[4], 4);
    std::memcpy(&z, &payload[8], 4);
    EXPECT_FLOAT_EQ(x, 1.5f);
    EXPECT_FLOAT_EQ(y, -2.3f);
    EXPECT_FLOAT_EQ(z, 9.81f);
}

// --- Gyro round-trip ---

TEST(FrameCodecTest, GyroRoundTrip) {
    GyroData gyro{100.0f, -50.0f, 25.0f};
    uint8_t buf[BUF_SIZE];

    size_t len = FrameCodec::encodeGyro(0x02, 9999ULL, gyro, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.nodeId, 0x02);
    EXPECT_EQ(hdr.type, SensorType::GYRO);
    EXPECT_EQ(payloadLen, 12u);
}

// --- Mag round-trip ---

TEST(FrameCodecTest, MagRoundTrip) {
    MagData mag{25.5f, -10.0f, 42.0f};
    uint8_t buf[BUF_SIZE];

    size_t len = FrameCodec::encodeMag(0x03, 1000000ULL, mag, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::MAG);
    EXPECT_EQ(payloadLen, 12u);
}

// --- Quaternion round-trip ---

TEST(FrameCodecTest, QuaternionRoundTrip) {
    Quaternion q{0.707f, 0.0f, 0.707f, 0.0f};
    uint8_t buf[BUF_SIZE];

    size_t len = FrameCodec::encodeQuaternion(0x01, 5000ULL, q, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);
    EXPECT_EQ(len, FrameCodec::HEADER_SIZE + 16 + FrameCodec::CRC_SIZE);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::QUATERNION);
    EXPECT_EQ(payloadLen, 16u);

    float w, x, y, z;
    std::memcpy(&w, &payload[0], 4);
    std::memcpy(&x, &payload[4], 4);
    std::memcpy(&y, &payload[8], 4);
    std::memcpy(&z, &payload[12], 4);
    EXPECT_FLOAT_EQ(w, 0.707f);
    EXPECT_FLOAT_EQ(y, 0.707f);
}

// --- Baro round-trip ---

TEST(FrameCodecTest, BaroRoundTrip) {
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeBaro(0x05, 7777ULL, 1013.25f, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::BARO);
    EXPECT_EQ(payloadLen, 4u);

    float hPa;
    std::memcpy(&hPa, payload, 4);
    EXPECT_FLOAT_EQ(hPa, 1013.25f);
}

// --- ECG round-trip ---

TEST(FrameCodecTest, ECGRoundTrip) {
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeECG(0x01, 3333ULL, -150, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::ECG);
    EXPECT_EQ(payloadLen, 4u);

    int32_t mv;
    std::memcpy(&mv, payload, 4);
    EXPECT_EQ(mv, -150);
}

// --- IMU_ALL round-trip ---

TEST(FrameCodecTest, IMUAllRoundTrip) {
    AccelData accel{0.1f, 0.2f, -1.0f};
    GyroData gyro{1.0f, 2.0f, 3.0f};
    float tempC = 25.5f;
    uint8_t buf[BUF_SIZE];

    size_t len = FrameCodec::encodeIMUAll(0x01, 8888ULL, accel, gyro, tempC, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);
    EXPECT_EQ(len, FrameCodec::HEADER_SIZE + 28 + FrameCodec::CRC_SIZE);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));

    EXPECT_EQ(hdr.type, SensorType::IMU_ALL);
    EXPECT_EQ(payloadLen, 28u);

    // Verify all 7 floats
    float vals[7];
    std::memcpy(vals, payload, 28);
    EXPECT_FLOAT_EQ(vals[0], 0.1f);   // accel.x
    EXPECT_FLOAT_EQ(vals[1], 0.2f);   // accel.y
    EXPECT_FLOAT_EQ(vals[2], -1.0f);  // accel.z
    EXPECT_FLOAT_EQ(vals[3], 1.0f);   // gyro.x
    EXPECT_FLOAT_EQ(vals[4], 2.0f);   // gyro.y
    EXPECT_FLOAT_EQ(vals[5], 3.0f);   // gyro.z
    EXPECT_FLOAT_EQ(vals[6], 25.5f);  // temp
}

// --- Error cases ---

TEST(FrameCodecTest, BufferTooSmallReturnsZero) {
    AccelData a{0, 0, 0};
    uint8_t buf[4];
    EXPECT_EQ(FrameCodec::encodeAccel(0, 0, a, buf, 4), 0u);
}

TEST(FrameCodecTest, CorruptedCRCFailsDecode) {
    AccelData a{1.0f, 2.0f, 3.0f};
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeAccel(0x01, 100ULL, a, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    // Corrupt last byte (CRC)
    buf[len - 1] ^= 0xFF;

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    EXPECT_FALSE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
}

TEST(FrameCodecTest, CorruptedSyncFailsDecode) {
    AccelData a{1.0f, 2.0f, 3.0f};
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeAccel(0x01, 100ULL, a, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    buf[0] = 0x00;  // corrupt sync

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    EXPECT_FALSE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
}

TEST(FrameCodecTest, TooShortFrameFailsDecode) {
    uint8_t buf[10] = {0xAA, 0x55, 0, 0, 0, 0, 0, 0, 0, 0};
    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    EXPECT_FALSE(FrameCodec::decode(buf, 10, hdr, payload, payloadLen));
}

TEST(FrameCodecTest, CorruptedPayloadFailsDecode) {
    AccelData a{1.0f, 2.0f, 3.0f};
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeAccel(0x01, 100ULL, a, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    // Corrupt a payload byte
    buf[FrameCodec::HEADER_SIZE] ^= 0xFF;

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    EXPECT_FALSE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
}

TEST(FrameCodecTest, LargeTimestamp) {
    AccelData a{0, 0, 0};
    uint8_t buf[BUF_SIZE];
    uint64_t bigTs = 0xDEADBEEFCAFEBABEULL;
    size_t len = FrameCodec::encodeAccel(0xFF, bigTs, a, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    uint8_t payload[32];
    size_t payloadLen;
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, payload, payloadLen));
    EXPECT_EQ(hdr.timestampUs, bigTs);
    EXPECT_EQ(hdr.nodeId, 0xFF);
}

TEST(FrameCodecTest, MaxFrameSizeConstant) {
    EXPECT_EQ(FrameCodec::MAX_FRAME_SIZE, 42u);
}

TEST(FrameCodecTest, NullPayloadDecode) {
    AccelData a{1.0f, 2.0f, 3.0f};
    uint8_t buf[BUF_SIZE];
    size_t len = FrameCodec::encodeAccel(0x01, 0ULL, a, buf, BUF_SIZE);
    ASSERT_GT(len, 0u);

    FrameCodec::FrameHeader hdr;
    size_t payloadLen;
    // Pass nullptr for payload — should still decode header
    ASSERT_TRUE(FrameCodec::decode(buf, len, hdr, nullptr, payloadLen));
    EXPECT_EQ(payloadLen, 12u);
}
