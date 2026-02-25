#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include <cstdint>

namespace sf {

enum class Sht40Precision : uint8_t {
    HIGH   = 0xFD,  // 8.3 ms
    MEDIUM = 0xF6,  // 4.5 ms
    LOW    = 0xE0   // 1.7 ms
};

struct SHT40Config {
    Sht40Precision precision = Sht40Precision::HIGH;
    uint8_t        address   = 0x44;
};

class SHT40 {
public:
    SHT40(II2CBus& bus, IDelayProvider& delay, const SHT40Config& cfg = {});

    bool init();
    bool readSerial(uint32_t& serial);
    bool measure(float& temperatureC, float& humidityPercent);

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    SHT40Config cfg_;

    uint32_t delayForPrecision() const;
    bool sendCommand(uint8_t cmd);
    bool readResponse(uint8_t* buf, size_t len);
    static bool checkCrc(const uint8_t* data);
};

} // namespace sf
