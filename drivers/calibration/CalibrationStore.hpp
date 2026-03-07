#pragma once

#include "INvStore.hpp"
#include "SensorTypes.hpp"
#include <cstdint>

namespace sf {

enum class SensorId : uint8_t {
    ACCEL = 0,
    GYRO  = 1,
    MAG   = 2,
    BARO  = 3,
};

struct CalibrationData {
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    float offsetZ = 0.0f;
    float scaleX  = 1.0f;
    float scaleY  = 1.0f;
    float scaleZ  = 1.0f;
};

class CalibrationStore {
public:
    static constexpr uint32_t MAGIC = 0x43414C42; // "CALB"
    static constexpr uint32_t SLOT_SIZE = 4u + static_cast<uint32_t>(sizeof(CalibrationData)) + 4u;

    CalibrationStore(INvStore& nv, uint32_t baseAddress = 0);

    bool save(SensorId id, const CalibrationData& data);
    bool load(SensorId id, CalibrationData& data);
    bool loadOrDefault(SensorId id, CalibrationData& data, const CalibrationData& defaults);
    bool isValid(SensorId id);
    bool reset(SensorId id);

    void apply(const CalibrationData& cal, AccelData& a) const;
    void apply(const CalibrationData& cal, GyroData& g) const;
    void apply(const CalibrationData& cal, MagData& m) const;

    static uint32_t crc32(const uint8_t* data, size_t len);
    static bool isCalibrationSane(const CalibrationData& data);

private:
    INvStore& nv_;
    uint32_t baseAddr_;

    uint32_t slotAddress(SensorId id) const;
};

} // namespace sf
