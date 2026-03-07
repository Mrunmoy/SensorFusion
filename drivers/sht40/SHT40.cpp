#include "SHT40.hpp"
#include "Crc8Sensirion.hpp"
#include <algorithm>

namespace sf {

namespace {
    constexpr uint8_t CMD_SOFT_RESET  = 0x94;
    constexpr uint8_t CMD_READ_SERIAL = 0x89;
}

SHT40::SHT40(II2CBus& bus, IDelayProvider& delay, const SHT40Config& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg)
{}

uint32_t SHT40::delayForPrecision() const {
    switch (cfg_.precision) {
        case Sht40Precision::HIGH:   return 9;
        case Sht40Precision::MEDIUM: return 5;
        case Sht40Precision::LOW:    return 2;
    }
    return 9;
}

bool SHT40::sendCommand(uint8_t cmd) {
    return bus_.rawWrite(cfg_.address, &cmd, 1);
}

bool SHT40::readResponse(uint8_t* buf, size_t len) {
    return bus_.rawRead(cfg_.address, buf, len);
}

bool SHT40::checkCrc(const uint8_t* data) {
    return crc8Sensirion(data, 2) == data[2];
}

bool SHT40::init() {
    // Soft reset
    if (!sendCommand(CMD_SOFT_RESET)) return false;
    delay_.delayMs(1);

    // Verify communication by reading serial
    uint32_t serial;
    return readSerial(serial);
}

bool SHT40::readSerial(uint32_t& serial) {
    if (!sendCommand(CMD_READ_SERIAL)) return false;
    delay_.delayMs(1);

    uint8_t buf[6];
    if (!readResponse(buf, 6)) return false;

    if (!checkCrc(&buf[0]) || !checkCrc(&buf[3])) return false;

    serial = (static_cast<uint32_t>(buf[0]) << 24) |
             (static_cast<uint32_t>(buf[1]) << 16) |
             (static_cast<uint32_t>(buf[3]) << 8) |
             static_cast<uint32_t>(buf[4]);
    return true;
}

bool SHT40::measure(float& temperatureC, float& humidityPercent) {
    uint8_t cmd = static_cast<uint8_t>(cfg_.precision);
    if (!sendCommand(cmd)) return false;
    delay_.delayMs(delayForPrecision());

    uint8_t buf[6];
    if (!readResponse(buf, 6)) return false;

    if (!checkCrc(&buf[0]) || !checkCrc(&buf[3])) return false;

    uint16_t rawTemp = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    uint16_t rawHum  = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

    temperatureC    = -45.0f + 175.0f * (static_cast<float>(rawTemp) / 65535.0f);
    humidityPercent = -6.0f + 125.0f * (static_cast<float>(rawHum) / 65535.0f);
    humidityPercent = std::max(0.0f, std::min(100.0f, humidityPercent));

    return true;
}

bool SHT40::readHumidityPercent(float& humidityPercent) {
    float temperatureC;
    return measure(temperatureC, humidityPercent);
}

} // namespace sf
