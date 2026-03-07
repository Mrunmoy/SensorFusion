#pragma once

#include "SensorInterface.hpp"
#include "CalibrationStore.hpp"
#include "AD8232.hpp"

namespace sf {

class SensorHub {
public:
    explicit SensorHub(CalibrationStore* calStore = nullptr);

    // Register available sensors (nullptr = not present)
    void setAccel(IAccelSensor* accel);
    void setIMU(IAccelGyroSensor* imu);
    void setMag(IMagSensor* mag);
    void setBaro(IBaroSensor* baro);
    void setHumidity(IHumiditySensor* humidity);
    void setVoc(IVocSensor* voc);
    void setECG(AD8232* ecg);

    // Read calibrated data (returns false if sensor not registered)
    bool readAccel(AccelData& out);
    bool readGyro(GyroData& out);
    bool readMag(MagData& out);
    bool readPressure(float& hPa);
    bool readHumidity(float& humidityPercent);
    bool readVocRaw(uint16_t& vocRaw);
    bool readECG(ECGSample& out);

    // Query capability
    bool hasAccel() const;
    bool hasIMU() const;
    bool hasMag() const;
    bool hasBaro() const;
    bool hasHumidity() const;
    bool hasVoc() const;
    bool hasECG() const;

private:
    IAccelSensor* accel_ = nullptr;
    IAccelGyroSensor* imu_ = nullptr;
    IMagSensor* mag_ = nullptr;
    IBaroSensor* baro_ = nullptr;
    IHumiditySensor* humidity_ = nullptr;
    IVocSensor* voc_ = nullptr;
    AD8232* ecg_ = nullptr;
    CalibrationStore* cal_ = nullptr;
};

} // namespace sf
