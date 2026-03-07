#pragma once

#include "SensorTypes.hpp"

namespace sf {

class IAccelSensor {
public:
    virtual ~IAccelSensor() = default;
    virtual bool readAccel(AccelData& out) = 0;
};

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

class IHumiditySensor {
public:
    virtual ~IHumiditySensor() = default;
    virtual bool readHumidityPercent(float& humidityPercent) = 0;
};

class IVocSensor {
public:
    virtual ~IVocSensor() = default;
    virtual bool readVocRaw(uint16_t& vocRaw) = 0;
};

} // namespace sf
