#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorInterface.hpp"
#include <cstdint>

namespace sf {

enum class LpsOdr : uint8_t {
    ONE_SHOT = 0x00,
    HZ_1     = 0x01,
    HZ_4     = 0x02,
    HZ_10    = 0x03,
    HZ_25    = 0x04,
    HZ_50    = 0x05,
    HZ_75    = 0x06,
    HZ_100   = 0x07,
    HZ_200   = 0x08,
};

enum class LpsAvg : uint8_t {
    AVG_4   = 0x00,
    AVG_8   = 0x01,
    AVG_16  = 0x02,
    AVG_32  = 0x03,
    AVG_64  = 0x04,
    AVG_128 = 0x05,
    AVG_256 = 0x06,
    AVG_512 = 0x07,
};

struct LPS22DFConfig {
    LpsOdr  odr     = LpsOdr::HZ_25;
    LpsAvg  avg     = LpsAvg::AVG_4;
    uint8_t address = 0x5D;
};

class LPS22DF : public IBaroSensor {
public:
    LPS22DF(II2CBus& bus, IDelayProvider& delay, const LPS22DFConfig& cfg = {});

    bool init();
    bool readPressure(float& hPa);
    bool readPressureHPa(float& hPa) override { return readPressure(hPa); }
    bool readTemperature(float& tempC);
    bool readAll(float& hPa, float& tempC);

    static float altitudeFromPressure(float hPa, float seaLevelHPa = 1013.25f);

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    LPS22DFConfig cfg_;

    IGpioInterrupt* intPin_ = nullptr;

    static int16_t sensorToHost16(const uint8_t* buf);
    static int32_t sensorToHost24(const uint8_t* buf);
};

} // namespace sf
