#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorInterface.hpp"
#include <cstdint>

namespace sf {

enum class Lis3mdlScale : uint8_t {
    GAUSS_4  = 0x00,
    GAUSS_8  = 0x01,
    GAUSS_12 = 0x02,
    GAUSS_16 = 0x03
};

enum class Lis3mdlOdr : uint8_t {
    HZ_0_625 = 0x00,
    HZ_1_25  = 0x01,
    HZ_2_5   = 0x02,
    HZ_5     = 0x03,
    HZ_10    = 0x04,
    HZ_20    = 0x05,
    HZ_40    = 0x06,
    HZ_80    = 0x07
};

struct LIS3MDLConfig {
    Lis3mdlScale scale   = Lis3mdlScale::GAUSS_4;
    Lis3mdlOdr   odr     = Lis3mdlOdr::HZ_80;
    uint8_t      address = 0x1E;
};

class LIS3MDL : public IMagSensor {
public:
    LIS3MDL(II2CBus& bus, IDelayProvider& delay, const LIS3MDLConfig& cfg = {});

    bool init();
    bool readRaw(int16_t& x, int16_t& y, int16_t& z);
    bool readMicroTesla(MagData& out);
    bool readMag(MagData& out) override { return readMicroTesla(out); }
    bool readTemperature(float& tempC);

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    LIS3MDLConfig cfg_;
    float lsbPerGauss_;

    IGpioInterrupt* intPin_ = nullptr;

    static float lsbPerG(Lis3mdlScale s);
    static int16_t sensorToHost16(const uint8_t* buf);
};

} // namespace sf
