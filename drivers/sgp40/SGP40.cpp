#include "SGP40.hpp"
#include "Crc8Sensirion.hpp"

namespace sf {

namespace {
    constexpr uint16_t CMD_MEASURE_RAW = 0x260F;
    constexpr uint16_t CMD_HEATER_OFF  = 0x3615;

    // Default compensation: 50% RH, 25°C
    constexpr uint16_t DEFAULT_HUM_TICKS  = 0x8000;
    constexpr uint16_t DEFAULT_TEMP_TICKS = 0x6666;
}

SGP40::SGP40(II2CBus& bus, IDelayProvider& delay, uint8_t address)
    : bus_(bus), delay_(delay), address_(address)
{}

bool SGP40::init() {
    // Turn off heater to start in a known state
    return heaterOff();
}

bool SGP40::heaterOff() {
    uint8_t cmd[2] = {
        static_cast<uint8_t>(CMD_HEATER_OFF >> 8),
        static_cast<uint8_t>(CMD_HEATER_OFF & 0xFF)
    };
    return bus_.rawWrite(address_, cmd, 2);
}

bool SGP40::sendMeasureCmd(uint16_t humTicks, uint16_t tempTicks) {
    uint8_t buf[8];

    // Command bytes
    buf[0] = static_cast<uint8_t>(CMD_MEASURE_RAW >> 8);
    buf[1] = static_cast<uint8_t>(CMD_MEASURE_RAW & 0xFF);

    // Humidity parameter + CRC
    buf[2] = static_cast<uint8_t>(humTicks >> 8);
    buf[3] = static_cast<uint8_t>(humTicks & 0xFF);
    buf[4] = crc8Sensirion(&buf[2], 2);

    // Temperature parameter + CRC
    buf[5] = static_cast<uint8_t>(tempTicks >> 8);
    buf[6] = static_cast<uint8_t>(tempTicks & 0xFF);
    buf[7] = crc8Sensirion(&buf[5], 2);

    return bus_.rawWrite(address_, buf, 8);
}

bool SGP40::readVocResponse(uint16_t& vocRaw) {
    uint8_t buf[3];
    if (!bus_.rawRead(address_, buf, 3)) return false;

    if (crc8Sensirion(buf, 2) != buf[2]) return false;

    vocRaw = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool SGP40::measureRaw(uint16_t& vocRaw) {
    if (!sendMeasureCmd(DEFAULT_HUM_TICKS, DEFAULT_TEMP_TICKS)) return false;
    delay_.delayMs(30);
    return readVocResponse(vocRaw);
}

bool SGP40::measureRaw(uint16_t& vocRaw, float humidityPercent, float temperatureC) {
    uint16_t humTicks = static_cast<uint16_t>(humidityPercent * 65535.0f / 100.0f + 0.5f);
    uint16_t tempTicks = static_cast<uint16_t>((temperatureC + 45.0f) * 65535.0f / 175.0f + 0.5f);

    if (!sendMeasureCmd(humTicks, tempTicks)) return false;
    delay_.delayMs(30);
    return readVocResponse(vocRaw);
}

} // namespace sf
