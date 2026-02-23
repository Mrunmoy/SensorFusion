#include "BMP180.hpp"
#include <cmath>

namespace sf {

namespace {
    constexpr uint8_t CALIB_START = 0xAA;
    constexpr uint8_t CHIP_ID    = 0xD0;
    constexpr uint8_t CTRL_MEAS  = 0xF4;
    constexpr uint8_t OUT_MSB    = 0xF6;
}

static constexpr uint8_t CMD_TEMP = 0x2E;
static constexpr uint8_t EXPECTED_ID = 0x55;

// Delay in ms for each oversampling setting
static constexpr uint8_t ossDelay[] = {5, 8, 14, 26};

BMP180::BMP180(II2CBus& bus, IDelayProvider& delay, const BMP180Config& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg),
      ac1_(0), ac2_(0), ac3_(0),
      ac4_(0), ac5_(0), ac6_(0),
      b1_(0), b2_(0), mb_(0), mc_(0), md_(0)
{}

bool BMP180::init() {
    const uint8_t addr = cfg_.address;

    // Verify chip ID
    uint8_t id;
    if (!bus_.read8(addr, CHIP_ID, id)) return false;
    if (id != EXPECTED_ID) return false;

    // Read calibration data (22 bytes, big-endian)
    uint8_t calib[22];
    if (!bus_.readRegister(addr, CALIB_START, calib, 22)) return false;

    auto get16 = [&](int idx) -> int16_t {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(calib[idx * 2]) << 8) | static_cast<uint16_t>(calib[idx * 2 + 1]));
    };
    auto getu16 = [&](int idx) -> uint16_t {
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(calib[idx * 2]) << 8) | static_cast<uint16_t>(calib[idx * 2 + 1]));
    };

    ac1_ = get16(0);
    ac2_ = get16(1);
    ac3_ = get16(2);
    ac4_ = getu16(3);
    ac5_ = getu16(4);
    ac6_ = getu16(5);
    b1_  = get16(6);
    b2_  = get16(7);
    mb_  = get16(8);
    mc_  = get16(9);
    md_  = get16(10);

    return true;
}

bool BMP180::readRawTemp(int32_t& ut) {
    if (!bus_.write8(cfg_.address, CTRL_MEAS, CMD_TEMP)) return false;
    delay_.delayMs(5);
    uint8_t buf[2];
    if (!bus_.readRegister(cfg_.address, OUT_MSB, buf, 2)) return false;
    ut = static_cast<int32_t>(
        (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
    return true;
}

bool BMP180::readRawPressure(int32_t& up) {
    uint8_t oss = static_cast<uint8_t>(cfg_.oss);
    uint8_t cmd = 0x34 | (oss << 6);
    if (!bus_.write8(cfg_.address, CTRL_MEAS, cmd)) return false;
    delay_.delayMs(ossDelay[oss]);
    uint8_t buf[3];
    if (!bus_.readRegister(cfg_.address, OUT_MSB, buf, 3)) return false;
    up = ((static_cast<int32_t>(buf[0]) << 16) |
          (static_cast<int32_t>(buf[1]) << 8) |
          static_cast<int32_t>(buf[2])) >> (8 - oss);
    return true;
}

int32_t BMP180::computeB5(int32_t ut) {
    int32_t x1 = (static_cast<int32_t>(ut - ac6_) * ac5_) >> 15;
    int32_t divisor = x1 + md_;
    if (divisor == 0) return 0; // bad calibration data
    int32_t x2 = (static_cast<int32_t>(mc_) << 11) / divisor;
    return x1 + x2;
}

int32_t BMP180::computeTruePressure(int32_t up, int32_t b5) {
    uint8_t oss = static_cast<uint8_t>(cfg_.oss);

    int32_t b6 = b5 - 4000;
    int32_t x1 = (static_cast<int32_t>(b2_) * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (static_cast<int32_t>(ac2_) * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((static_cast<int32_t>(ac1_) * 4 + x3) << oss) + 2) / 4;

    x1 = (static_cast<int32_t>(ac3_) * b6) >> 13;
    x2 = (static_cast<int32_t>(b1_) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (static_cast<uint32_t>(ac4_) * static_cast<uint32_t>(x3 + 32768)) >> 15;
    if (b4 == 0) return 0; // bad calibration data
    uint32_t b7 = (static_cast<uint32_t>(up - b3)) * (50000u >> oss);

    int32_t p;
    if (b7 < 0x80000000u)
        p = static_cast<int32_t>((b7 * 2) / b4);
    else
        p = static_cast<int32_t>((b7 / b4) * 2);

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}

bool BMP180::readTemperature(float& tempC) {
    int32_t ut;
    if (!readRawTemp(ut)) return false;
    int32_t b5 = computeB5(ut);
    tempC = static_cast<float>((b5 + 8) >> 4) / 10.0f;
    return true;
}

bool BMP180::readPressure(int32_t& pressurePa) {
    int32_t ut, up;
    if (!readRawTemp(ut)) return false;
    if (!readRawPressure(up)) return false;
    int32_t b5 = computeB5(ut);
    pressurePa = computeTruePressure(up, b5);
    return true;
}

bool BMP180::readTempAndPressure(float& tempC, int32_t& pressurePa) {
    int32_t ut, up;
    if (!readRawTemp(ut)) return false;
    if (!readRawPressure(up)) return false;
    int32_t b5 = computeB5(ut);
    tempC = static_cast<float>((b5 + 8) >> 4) / 10.0f;
    pressurePa = computeTruePressure(up, b5);
    return true;
}

float BMP180::pressureToAltitude(int32_t pressurePa, float seaLevelPa) {
    return 44330.0f * (1.0f - std::pow(static_cast<float>(pressurePa) / seaLevelPa,
                                        1.0f / 5.255f));
}

} // namespace sf
