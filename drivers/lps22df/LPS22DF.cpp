#include "LPS22DF.hpp"
#include <cmath>

namespace sf {

namespace reg {
    constexpr uint8_t WHO_AM_I     = 0x0F;
    constexpr uint8_t CTRL_REG1    = 0x10;
    constexpr uint8_t CTRL_REG2    = 0x11;
    constexpr uint8_t CTRL_REG4    = 0x13;
    constexpr uint8_t STATUS       = 0x27;
    constexpr uint8_t PRESS_OUT_XL = 0x28;
    constexpr uint8_t TEMP_OUT_L   = 0x2B;
}

static constexpr uint8_t EXPECTED_WHO_AM_I = 0xB4;
static constexpr float PRESS_SENSITIVITY = 4096.0f;  // LSB/hPa
static constexpr float TEMP_SENSITIVITY  = 100.0f;   // LSB/°C

LPS22DF::LPS22DF(II2CBus& bus, IDelayProvider& delay, const LPS22DFConfig& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg)
{}

int16_t LPS22DF::sensorToHost16(const uint8_t* buf) {
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

int32_t LPS22DF::sensorToHost24(const uint8_t* buf) {
    // Little-endian: XL, L, H
    uint32_t raw = static_cast<uint32_t>(buf[0]) |
                   (static_cast<uint32_t>(buf[1]) << 8) |
                   (static_cast<uint32_t>(buf[2]) << 16);
    // Sign-extend 24-bit
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
    return static_cast<int32_t>(raw);
}

bool LPS22DF::init() {
    const uint8_t addr = cfg_.address;

    // Software reset
    if (!bus_.write8(addr, reg::CTRL_REG2, 0x04)) return false;
    delay_.delayMs(10);

    // Verify WHO_AM_I
    uint8_t id;
    if (!bus_.read8(addr, reg::WHO_AM_I, id)) return false;
    if (id != EXPECTED_WHO_AM_I) return false;

    // CTRL_REG1: ODR[6:3] | AVG[2:0]
    uint8_t ctrl1 = (static_cast<uint8_t>(cfg_.odr) << 3) |
                     static_cast<uint8_t>(cfg_.avg);
    if (!bus_.write8(addr, reg::CTRL_REG1, ctrl1)) return false;

    // CTRL_REG2: BDU=1 (bit 3)
    if (!bus_.write8(addr, reg::CTRL_REG2, 0x08)) return false;

    return true;
}

bool LPS22DF::readPressure(float& hPa) {
    uint8_t buf[3];
    if (!bus_.readRegister(cfg_.address, reg::PRESS_OUT_XL, buf, 3)) return false;
    int32_t raw = sensorToHost24(buf);
    hPa = static_cast<float>(raw) / PRESS_SENSITIVITY;
    return true;
}

bool LPS22DF::readTemperature(float& tempC) {
    uint8_t buf[2];
    if (!bus_.readRegister(cfg_.address, reg::TEMP_OUT_L, buf, 2)) return false;
    int16_t raw = sensorToHost16(buf);
    tempC = static_cast<float>(raw) / TEMP_SENSITIVITY;
    return true;
}

bool LPS22DF::readAll(float& hPa, float& tempC) {
    // Pressure (3 bytes at 0x28) + temperature (2 bytes at 0x2B) = 5 contiguous bytes
    uint8_t buf[5];
    if (!bus_.readRegister(cfg_.address, reg::PRESS_OUT_XL, buf, 5)) return false;

    int32_t rawP = sensorToHost24(&buf[0]);
    hPa = static_cast<float>(rawP) / PRESS_SENSITIVITY;

    int16_t rawT = sensorToHost16(&buf[3]);
    tempC = static_cast<float>(rawT) / TEMP_SENSITIVITY;

    return true;
}

float LPS22DF::altitudeFromPressure(float hPa, float seaLevelHPa) {
    return 44330.0f * (1.0f - std::pow(hPa / seaLevelHPa, 1.0f / 5.255f));
}

bool LPS22DF::enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                       IGpioInterrupt::Callback cb, void* ctx) {
    if (!intPin) return false;
    const uint8_t addr = cfg_.address;

    // CTRL_REG4: INT_EN (bit 4) + DRDY (bit 3)
    if (!bus_.write8(addr, reg::CTRL_REG4, 0x18)) return false;

    // Default: active-high, push-pull
    if (!intPin->enable(GpioEdge::RISING, cb, ctx)) return false;

    intPin_ = intPin;
    return true;
}

bool LPS22DF::disableDataReadyInterrupt() {
    const uint8_t addr = cfg_.address;

    // Clear CTRL_REG4
    if (!bus_.write8(addr, reg::CTRL_REG4, 0x00)) return false;

    if (intPin_) {
        intPin_->disable();
        intPin_ = nullptr;
    }
    return true;
}

} // namespace sf
