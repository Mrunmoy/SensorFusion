#pragma once

#include "II2CBus.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorInterface.hpp"
#include <cstdint>

namespace sf {

enum class AdxlRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };

struct ADXL345Config {
    AdxlRange range   = AdxlRange::G4;
    bool      fullRes = true;
    uint8_t   address = 0x53;
};

class ADXL345 : public IAccelSensor {
public:
    explicit ADXL345(II2CBus& bus, const ADXL345Config& cfg = {});

    bool init();
    bool readAccel(AccelData& out) override;

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();
    bool clearInterrupt(uint8_t& source);

private:
    II2CBus& bus_;
    ADXL345Config cfg_;
    float mgPerLsb_;

    IGpioInterrupt* intPin_ = nullptr;

    static float computeMgPerLsb(AdxlRange range, bool fullRes);
    static int16_t sensorToHost16(const uint8_t* buf);
};

} // namespace sf
