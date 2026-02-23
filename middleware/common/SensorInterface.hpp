#pragma once

#include "SensorTypes.hpp"

namespace sf {

class IAccelGyroSensor {
public:
    virtual ~IAccelGyroSensor() = default;
    virtual bool readAccel(AccelData& out) = 0;
    virtual bool readGyro(GyroData& out) = 0;
    virtual bool readTemperature(float& tempC) = 0;
};

class IMagSensor {
public:
    virtual ~IMagSensor() = default;
    virtual bool readMag(MagData& out) = 0;
};

class IBaroSensor {
public:
    virtual ~IBaroSensor() = default;
    virtual bool readPressureHPa(float& hPa) = 0;
};

} // namespace sf
