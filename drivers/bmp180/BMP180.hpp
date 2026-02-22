#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include <cstdint>

namespace sf {

enum class BMP180Oss : uint8_t {
    ULTRA_LOW = 0, STANDARD = 1, HIGH_RES = 2, ULTRA_HIGH = 3
};

struct BMP180Config {
    BMP180Oss oss     = BMP180Oss::STANDARD;
    uint8_t   address = 0x77;
};

class BMP180 {
public:
    BMP180(II2CBus& bus, IDelayProvider& delay, const BMP180Config& cfg = {});

    bool init();
    bool readTemperature(float& tempC);
    bool readPressure(int32_t& pressurePa);
    bool readTempAndPressure(float& tempC, int32_t& pressurePa);
    static float pressureToAltitude(int32_t pressurePa, float seaLevelPa = 101325.0f);

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    BMP180Config cfg_;

    // Calibration coefficients
    int16_t  ac1_, ac2_, ac3_;
    uint16_t ac4_, ac5_, ac6_;
    int16_t  b1_, b2_, mb_, mc_, md_;

    bool readRawTemp(int32_t& ut);
    bool readRawPressure(int32_t& up);
    int32_t computeB5(int32_t ut);
    int32_t computeTruePressure(int32_t up, int32_t b5);
};

} // namespace sf
