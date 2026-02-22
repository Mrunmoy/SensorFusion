#include "QMC5883L.hpp"
#include <cmath>

namespace sf {

namespace reg {
    constexpr uint8_t DATA_X_LSB = 0x00;
    constexpr uint8_t STATUS     = 0x06;
    constexpr uint8_t CTRL1      = 0x09;
    constexpr uint8_t CTRL2      = 0x0A;
    constexpr uint8_t SET_RST    = 0x0B;
}

QMC5883L::QMC5883L(II2CBus& bus, IDelayProvider& delay, const QMC5883LConfig& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg),
      lsbPerMicroTesla_(lsbPerUT(cfg.range))
{}

float QMC5883L::lsbPerUT(MagRange r) {
    // 2G: 12000 LSB/Gauss = 1200 LSB/µT
    // 8G: 3000 LSB/Gauss = 300 LSB/µT
    return (r == MagRange::GAUSS_2) ? 1200.0f : 300.0f;
}

int16_t QMC5883L::sensorToHost16(const uint8_t* buf) {
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

bool QMC5883L::init() {
    const uint8_t addr = cfg_.address;

    // Soft reset
    if (!bus_.write8(addr, reg::CTRL2, 0x80)) return false;
    delay_.delayMs(10);

    // Recommended SET/RESET period
    if (!bus_.write8(addr, reg::SET_RST, 0x01)) return false;

    // CTRL1: OSR[7:6] | RNG[5:4] | ODR[3:2] | MODE[1:0]
    uint8_t ctrl1 = (static_cast<uint8_t>(cfg_.osr) << 6) |
                    (static_cast<uint8_t>(cfg_.range) << 4) |
                    (static_cast<uint8_t>(cfg_.odr) << 2) |
                    static_cast<uint8_t>(cfg_.mode);
    if (!bus_.write8(addr, reg::CTRL1, ctrl1)) return false;
    delay_.delayMs(5);

    return true;
}

bool QMC5883L::isDataReady(bool& ready) {
    uint8_t status;
    if (!bus_.read8(cfg_.address, reg::STATUS, status)) return false;
    ready = (status & 0x01) != 0;
    return true;
}

bool QMC5883L::readRaw(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, reg::DATA_X_LSB, buf, 6)) return false;
    x = sensorToHost16(&buf[0]);
    y = sensorToHost16(&buf[2]);
    z = sensorToHost16(&buf[4]);
    return true;
}

bool QMC5883L::readMicroTesla(MagData& out) {
    int16_t rx, ry, rz;
    if (!readRaw(rx, ry, rz)) return false;
    out.x = static_cast<float>(rx) / lsbPerMicroTesla_;
    out.y = static_cast<float>(ry) / lsbPerMicroTesla_;
    out.z = static_cast<float>(rz) / lsbPerMicroTesla_;
    return true;
}

float QMC5883L::headingDegrees(float mx, float my) {
    float heading = std::atan2(-my, mx) * (180.0f / static_cast<float>(M_PI));
    if (heading < 0.0f) heading += 360.0f;
    return heading;
}

} // namespace sf
