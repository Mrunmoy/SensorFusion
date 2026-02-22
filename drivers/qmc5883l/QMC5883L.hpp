#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "SensorTypes.hpp"
#include <cstdint>

namespace sf {

enum class MagOsr   : uint8_t { OSR_512 = 0, OSR_256 = 1, OSR_128 = 2, OSR_64 = 3 };
enum class MagOdr   : uint8_t { HZ_10 = 0, HZ_50 = 1, HZ_100 = 2, HZ_200 = 3 };
enum class MagRange : uint8_t { GAUSS_2 = 0, GAUSS_8 = 1 };
enum class MagMode  : uint8_t { STANDBY = 0, CONTINUOUS = 1 };

struct QMC5883LConfig {
    MagOsr   osr     = MagOsr::OSR_512;
    MagRange range   = MagRange::GAUSS_8;
    MagOdr   odr     = MagOdr::HZ_200;
    MagMode  mode    = MagMode::CONTINUOUS;
    uint8_t  address = 0x0D;
};

class QMC5883L {
public:
    QMC5883L(II2CBus& bus, IDelayProvider& delay, const QMC5883LConfig& cfg = {});

    bool init();
    bool isDataReady(bool& ready);
    bool readRaw(int16_t& x, int16_t& y, int16_t& z);
    bool readMicroTesla(MagData& out);
    float headingDegrees(float mx, float my);

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    QMC5883LConfig cfg_;
    float lsbPerMicroTesla_;

    static float lsbPerUT(MagRange r);
    static int16_t toInt16LE(const uint8_t* buf);
};

} // namespace sf
