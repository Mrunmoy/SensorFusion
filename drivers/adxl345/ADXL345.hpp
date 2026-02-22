#pragma once

#include "II2CBus.hpp"
#include "SensorTypes.hpp"
#include <cstdint>

namespace sf {

enum class AdxlRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };

struct ADXL345Config {
    AdxlRange range   = AdxlRange::G4;
    bool      fullRes = true;
    uint8_t   address = 0x53;
};

class ADXL345 {
public:
    explicit ADXL345(II2CBus& bus, const ADXL345Config& cfg = {});

    bool init();
    bool readAccel(AccelData& out);

private:
    II2CBus& bus_;
    ADXL345Config cfg_;

    static int16_t toInt16LE(const uint8_t* buf);
};

} // namespace sf
