#include "CalibrationStore.hpp"
#include <cstring>

namespace sf {

CalibrationStore::CalibrationStore(INvStore& nv, uint32_t baseAddress)
    : nv_(nv), baseAddr_(baseAddress)
{}

uint32_t CalibrationStore::slotAddress(SensorId id) const {
    return baseAddr_ + static_cast<uint32_t>(id) * SLOT_SIZE;
}

uint32_t CalibrationStore::crc32(const uint8_t* data, size_t len) {
    // CRC-32 (ISO 3309 / ITU-T V.42)
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

bool CalibrationStore::save(SensorId id, const CalibrationData& data) {
    uint8_t buf[SLOT_SIZE];

    // Magic
    std::memcpy(buf, &MAGIC, 4);

    // Calibration data
    std::memcpy(buf + 4, &data, sizeof(CalibrationData));

    // CRC over magic + data
    uint32_t crc = crc32(buf, 4 + sizeof(CalibrationData));
    std::memcpy(buf + 4 + sizeof(CalibrationData), &crc, 4);

    return nv_.write(slotAddress(id), buf, SLOT_SIZE);
}

bool CalibrationStore::load(SensorId id, CalibrationData& data) {
    uint8_t buf[SLOT_SIZE];

    if (!nv_.read(slotAddress(id), buf, SLOT_SIZE)) return false;

    // Check magic
    uint32_t magic;
    std::memcpy(&magic, buf, 4);
    if (magic != MAGIC) return false;

    // Verify CRC
    uint32_t storedCrc;
    std::memcpy(&storedCrc, buf + 4 + sizeof(CalibrationData), 4);
    uint32_t computedCrc = crc32(buf, 4 + sizeof(CalibrationData));
    if (storedCrc != computedCrc) return false;

    std::memcpy(&data, buf + 4, sizeof(CalibrationData));
    return true;
}

bool CalibrationStore::isValid(SensorId id) {
    CalibrationData tmp;
    return load(id, tmp);
}

bool CalibrationStore::reset(SensorId id) {
    // Write all 0xFF to invalidate slot
    uint8_t buf[SLOT_SIZE];
    std::memset(buf, 0xFF, SLOT_SIZE);
    return nv_.write(slotAddress(id), buf, SLOT_SIZE);
}

void CalibrationStore::apply(const CalibrationData& cal, AccelData& a) const {
    a.x = (a.x - cal.offsetX) * cal.scaleX;
    a.y = (a.y - cal.offsetY) * cal.scaleY;
    a.z = (a.z - cal.offsetZ) * cal.scaleZ;
}

void CalibrationStore::apply(const CalibrationData& cal, GyroData& g) const {
    g.x = (g.x - cal.offsetX) * cal.scaleX;
    g.y = (g.y - cal.offsetY) * cal.scaleY;
    g.z = (g.z - cal.offsetZ) * cal.scaleZ;
}

void CalibrationStore::apply(const CalibrationData& cal, MagData& m) const {
    m.x = (m.x - cal.offsetX) * cal.scaleX;
    m.y = (m.y - cal.offsetY) * cal.scaleY;
    m.z = (m.z - cal.offsetZ) * cal.scaleZ;
}

} // namespace sf
