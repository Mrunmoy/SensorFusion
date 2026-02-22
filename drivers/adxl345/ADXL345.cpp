#include "ADXL345.hpp"

namespace sf {

namespace reg {
    constexpr uint8_t DEVID       = 0x00;
    constexpr uint8_t BW_RATE     = 0x2C;
    constexpr uint8_t POWER_CTL   = 0x2D;
    constexpr uint8_t DATA_FORMAT = 0x31;
    constexpr uint8_t DATAX0      = 0x32;
}

static constexpr uint8_t EXPECTED_DEVID = 0xE5;
static constexpr float MG_PER_LSB = 0.004f; // 4 mg/LSB in full-resolution mode

ADXL345::ADXL345(II2CBus& bus, const ADXL345Config& cfg)
    : bus_(bus), cfg_(cfg)
{}

int16_t ADXL345::toInt16LE(const uint8_t* buf) {
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

bool ADXL345::init() {
    const uint8_t addr = cfg_.address;

    // Verify DEVID
    uint8_t id;
    if (!bus_.read8(addr, reg::DEVID, id)) return false;
    if (id != EXPECTED_DEVID) return false;

    // Enable measurement mode
    if (!bus_.write8(addr, reg::POWER_CTL, 0x08)) return false;

    // Data format: range + full-resolution bit
    uint8_t fmt = static_cast<uint8_t>(cfg_.range);
    if (cfg_.fullRes) fmt |= 0x08;
    if (!bus_.write8(addr, reg::DATA_FORMAT, fmt)) return false;

    // Bandwidth rate: 100 Hz
    if (!bus_.write8(addr, reg::BW_RATE, 0x0A)) return false;

    return true;
}

bool ADXL345::readAccel(AccelData& out) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, reg::DATAX0, buf, 6)) return false;

    out.x = static_cast<float>(toInt16LE(&buf[0])) * MG_PER_LSB;
    out.y = static_cast<float>(toInt16LE(&buf[2])) * MG_PER_LSB;
    out.z = static_cast<float>(toInt16LE(&buf[4])) * MG_PER_LSB;
    return true;
}

} // namespace sf
