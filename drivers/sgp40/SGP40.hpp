#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include <cstdint>

namespace sf {

class SGP40 {
public:
    SGP40(II2CBus& bus, IDelayProvider& delay, uint8_t address = 0x59);

    bool init();
    bool measureRaw(uint16_t& vocRaw);
    bool measureRaw(uint16_t& vocRaw, float humidityPercent, float temperatureC);
    bool heaterOff();

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    uint8_t address_;

    bool sendMeasureCmd(uint16_t humTicks, uint16_t tempTicks);
    bool readVocResponse(uint16_t& vocRaw);
};

} // namespace sf
