#include "FrameCodec.hpp"
#include <cstring>

namespace sf {

// --- Helper: write header (sync + nodeId + type + timestamp) ---
size_t FrameCodec::writeHeader(uint8_t nodeId, SensorType type,
                                uint64_t ts, uint8_t* buf) {
    buf[0] = SYNC_BYTE_1;
    buf[1] = SYNC_BYTE_2;
    buf[2] = nodeId;
    buf[3] = static_cast<uint8_t>(type);
    writeU64LE(ts, &buf[4]);
    return HEADER_SIZE;
}

void FrameCodec::writeFloat(float val, uint8_t* buf) {
    uint32_t bits;
    std::memcpy(&bits, &val, sizeof(bits));
    buf[0] = static_cast<uint8_t>(bits);
    buf[1] = static_cast<uint8_t>(bits >> 8);
    buf[2] = static_cast<uint8_t>(bits >> 16);
    buf[3] = static_cast<uint8_t>(bits >> 24);
}

float FrameCodec::readFloat(const uint8_t* buf) {
    uint32_t bits = static_cast<uint32_t>(buf[0]) |
                    (static_cast<uint32_t>(buf[1]) << 8) |
                    (static_cast<uint32_t>(buf[2]) << 16) |
                    (static_cast<uint32_t>(buf[3]) << 24);
    float val;
    std::memcpy(&val, &bits, sizeof(val));
    return val;
}

void FrameCodec::writeU64LE(uint64_t val, uint8_t* buf) {
    for (int i = 0; i < 8; ++i) {
        buf[i] = static_cast<uint8_t>(val >> (i * 8));
    }
}

uint64_t FrameCodec::readU64LE(const uint8_t* buf) {
    uint64_t val = 0;
    for (int i = 0; i < 8; ++i) {
        val |= static_cast<uint64_t>(buf[i]) << (i * 8);
    }
    return val;
}

void FrameCodec::writeI32LE(int32_t val, uint8_t* buf) {
    uint32_t u;
    std::memcpy(&u, &val, sizeof(u));
    buf[0] = static_cast<uint8_t>(u);
    buf[1] = static_cast<uint8_t>(u >> 8);
    buf[2] = static_cast<uint8_t>(u >> 16);
    buf[3] = static_cast<uint8_t>(u >> 24);
}

int32_t FrameCodec::readI32LE(const uint8_t* buf) {
    uint32_t u = static_cast<uint32_t>(buf[0]) |
                 (static_cast<uint32_t>(buf[1]) << 8) |
                 (static_cast<uint32_t>(buf[2]) << 16) |
                 (static_cast<uint32_t>(buf[3]) << 24);
    int32_t val;
    std::memcpy(&val, &u, sizeof(val));
    return val;
}

void FrameCodec::writeU16LE(uint16_t val, uint8_t* buf) {
    buf[0] = static_cast<uint8_t>(val);
    buf[1] = static_cast<uint8_t>(val >> 8);
}

uint16_t FrameCodec::readU16LE(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0]) |
           (static_cast<uint16_t>(buf[1]) << 8);
}

// --- CRC-16 CCITT (polynomial 0x1021, init 0xFFFF) ---
uint16_t FrameCodec::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// --- Encode helpers ---

size_t FrameCodec::finalize(uint8_t* buf, size_t pos, size_t bufLen) {
    if (pos + CRC_SIZE > bufLen) return 0;
    uint16_t crc = crc16(buf, pos);
    buf[pos]     = static_cast<uint8_t>(crc);
    buf[pos + 1] = static_cast<uint8_t>(crc >> 8);
    return pos + CRC_SIZE;
}

size_t FrameCodec::encode3Float(uint8_t nodeId, SensorType type, uint64_t ts,
                                 float v1, float v2, float v3,
                                 uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 12;
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, type, ts, buf);
    writeFloat(v1, &buf[pos]); pos += 4;
    writeFloat(v2, &buf[pos]); pos += 4;
    writeFloat(v3, &buf[pos]); pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodeAccel(uint8_t nodeId, uint64_t ts,
                                const AccelData& data,
                                uint8_t* buf, size_t bufLen) {
    return encode3Float(nodeId, SensorType::ACCEL, ts,
                        data.x, data.y, data.z, buf, bufLen);
}

size_t FrameCodec::encodeGyro(uint8_t nodeId, uint64_t ts,
                               const GyroData& data,
                               uint8_t* buf, size_t bufLen) {
    return encode3Float(nodeId, SensorType::GYRO, ts,
                        data.x, data.y, data.z, buf, bufLen);
}

size_t FrameCodec::encodeMag(uint8_t nodeId, uint64_t ts,
                              const MagData& data,
                              uint8_t* buf, size_t bufLen) {
    return encode3Float(nodeId, SensorType::MAG, ts,
                        data.x, data.y, data.z, buf, bufLen);
}

size_t FrameCodec::encodeQuaternion(uint8_t nodeId, uint64_t ts,
                                     const Quaternion& data,
                                     uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 16;  // 4 * 4 bytes
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::QUATERNION, ts, buf);
    writeFloat(data.w, &buf[pos]); pos += 4;
    writeFloat(data.x, &buf[pos]); pos += 4;
    writeFloat(data.y, &buf[pos]); pos += 4;
    writeFloat(data.z, &buf[pos]); pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodeBaro(uint8_t nodeId, uint64_t ts,
                               float hPa,
                               uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 4;
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::BARO, ts, buf);
    writeFloat(hPa, &buf[pos]); pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodeECG(uint8_t nodeId, uint64_t ts,
                              int32_t millivolts,
                              uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 4;
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::ECG, ts, buf);
    writeI32LE(millivolts, &buf[pos]); pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodeIMUAll(uint8_t nodeId, uint64_t ts,
                                 const AccelData& accel, const GyroData& gyro,
                                 float tempC,
                                 uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 28;  // 3*4 + 3*4 + 4 = 28
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::IMU_ALL, ts, buf);
    writeFloat(accel.x, &buf[pos]); pos += 4;
    writeFloat(accel.y, &buf[pos]); pos += 4;
    writeFloat(accel.z, &buf[pos]); pos += 4;
    writeFloat(gyro.x, &buf[pos]);  pos += 4;
    writeFloat(gyro.y, &buf[pos]);  pos += 4;
    writeFloat(gyro.z, &buf[pos]);  pos += 4;
    writeFloat(tempC, &buf[pos]);   pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodePose(uint8_t nodeId, uint64_t ts,
                               float posX, float posY, float posZ,
                               const Quaternion& orientation,
                               uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 28;  // 3*4 pos + 4*4 quat
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::POSE, ts, buf);
    writeFloat(posX, &buf[pos]); pos += 4;
    writeFloat(posY, &buf[pos]); pos += 4;
    writeFloat(posZ, &buf[pos]); pos += 4;
    writeFloat(orientation.w, &buf[pos]); pos += 4;
    writeFloat(orientation.x, &buf[pos]); pos += 4;
    writeFloat(orientation.y, &buf[pos]); pos += 4;
    writeFloat(orientation.z, &buf[pos]); pos += 4;
    return finalize(buf, pos, bufLen);
}

size_t FrameCodec::encodeNodeHealth(uint8_t nodeId, uint64_t ts,
                                    uint16_t batteryMv,
                                    uint8_t batteryPercent,
                                    uint8_t linkQuality,
                                    uint16_t droppedFrames,
                                    uint8_t calibrationState,
                                    uint8_t flags,
                                    uint8_t* buf, size_t bufLen) {
    constexpr size_t PAYLOAD_SIZE = 8;
    constexpr size_t FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE;
    if (bufLen < FRAME_SIZE) return 0;

    size_t pos = writeHeader(nodeId, SensorType::NODE_HEALTH, ts, buf);
    writeU16LE(batteryMv, &buf[pos]); pos += 2;
    buf[pos++] = batteryPercent;
    buf[pos++] = linkQuality;
    writeU16LE(droppedFrames, &buf[pos]); pos += 2;
    buf[pos++] = calibrationState;
    buf[pos++] = flags;
    return finalize(buf, pos, bufLen);
}

// --- Decode ---
bool FrameCodec::decode(const uint8_t* buf, size_t len,
                         FrameHeader& hdr,
                         uint8_t* payload, size_t& payloadLen) {
    // Minimum frame: header(12) + CRC(2) = 14
    if (len < HEADER_SIZE + CRC_SIZE) return false;

    // Check sync bytes
    if (buf[0] != SYNC_BYTE_1 || buf[1] != SYNC_BYTE_2) return false;

    // Verify CRC over everything except the last 2 bytes
    uint16_t expected = crc16(buf, len - CRC_SIZE);
    uint16_t received = readU16LE(&buf[len - CRC_SIZE]);
    if (expected != received) return false;

    // Parse header
    hdr.nodeId = buf[2];
    hdr.type = static_cast<SensorType>(buf[3]);
    hdr.timestampUs = readU64LE(&buf[4]);

    // Copy payload
    payloadLen = len - HEADER_SIZE - CRC_SIZE;
    if (payloadLen > 0 && payload != nullptr) {
        std::memcpy(payload, &buf[HEADER_SIZE], payloadLen);
    }

    return true;
}

} // namespace sf
