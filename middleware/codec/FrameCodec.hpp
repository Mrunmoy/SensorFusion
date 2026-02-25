#pragma once

#include "SensorTypes.hpp"
#include "Quaternion.hpp"
#include <cstdint>
#include <cstddef>

namespace sf {

enum class SensorType : uint8_t {
    ACCEL      = 0x01,
    GYRO       = 0x02,
    MAG        = 0x03,
    QUATERNION = 0x04,
    BARO       = 0x05,
    ECG        = 0x06,
    IMU_ALL    = 0x07,
    POSE       = 0x08,
};

class FrameCodec {
public:
    static constexpr uint8_t SYNC_BYTE_1 = 0xAA;
    static constexpr uint8_t SYNC_BYTE_2 = 0x55;
    static constexpr size_t HEADER_SIZE = 12;  // sync(2) + nodeId(1) + type(1) + timestamp(8)
    static constexpr size_t CRC_SIZE = 2;
    static constexpr size_t MAX_FRAME_SIZE = 42;  // IMU_ALL: header(12) + 28 + crc(2)

    struct FrameHeader {
        uint8_t nodeId;
        SensorType type;
        uint64_t timestampUs;
    };

    // Encode functions — return bytes written, 0 on error
    static size_t encodeAccel(uint8_t nodeId, uint64_t ts,
                              const AccelData& data,
                              uint8_t* buf, size_t bufLen);

    static size_t encodeGyro(uint8_t nodeId, uint64_t ts,
                             const GyroData& data,
                             uint8_t* buf, size_t bufLen);

    static size_t encodeMag(uint8_t nodeId, uint64_t ts,
                            const MagData& data,
                            uint8_t* buf, size_t bufLen);

    static size_t encodeQuaternion(uint8_t nodeId, uint64_t ts,
                                   const Quaternion& data,
                                   uint8_t* buf, size_t bufLen);

    static size_t encodeBaro(uint8_t nodeId, uint64_t ts,
                             float hPa,
                             uint8_t* buf, size_t bufLen);

    static size_t encodeECG(uint8_t nodeId, uint64_t ts,
                            int32_t millivolts,
                            uint8_t* buf, size_t bufLen);

    static size_t encodeIMUAll(uint8_t nodeId, uint64_t ts,
                               const AccelData& accel, const GyroData& gyro,
                               float tempC,
                               uint8_t* buf, size_t bufLen);

    static size_t encodePose(uint8_t nodeId, uint64_t ts,
                             float posX, float posY, float posZ,
                             const Quaternion& orientation,
                             uint8_t* buf, size_t bufLen);

    // Decode — fills header and copies payload bytes
    static bool decode(const uint8_t* buf, size_t len,
                       FrameHeader& hdr,
                       uint8_t* payload, size_t& payloadLen);

    static uint16_t crc16(const uint8_t* data, size_t len);

private:
    static size_t finalize(uint8_t* buf, size_t pos, size_t bufLen);
    static size_t encode3Float(uint8_t nodeId, SensorType type, uint64_t ts,
                               float v1, float v2, float v3,
                               uint8_t* buf, size_t bufLen);
    static size_t writeHeader(uint8_t nodeId, SensorType type,
                              uint64_t ts, uint8_t* buf);
    static void writeFloat(float val, uint8_t* buf);
    static float readFloat(const uint8_t* buf);
    static void writeU64LE(uint64_t val, uint8_t* buf);
    static uint64_t readU64LE(const uint8_t* buf);
    static void writeI32LE(int32_t val, uint8_t* buf);
    static int32_t readI32LE(const uint8_t* buf);
    static void writeU16LE(uint16_t val, uint8_t* buf);
    static uint16_t readU16LE(const uint8_t* buf);
};

} // namespace sf
