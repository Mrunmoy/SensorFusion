#include "SensorHub.hpp"

namespace sf {

SensorHub::SensorHub(CalibrationStore* calStore)
    : cal_(calStore)
{}

void SensorHub::setAccel(IAccelSensor* accel) { accel_ = accel; }
void SensorHub::setIMU(IAccelGyroSensor* imu) { imu_ = imu; }
void SensorHub::setMag(IMagSensor* mag) { mag_ = mag; }
void SensorHub::setBaro(IBaroSensor* baro) { baro_ = baro; }
void SensorHub::setECG(AD8232* ecg) { ecg_ = ecg; }

bool SensorHub::readAccel(AccelData& out) {
    bool ok = false;
    if (accel_) {
        ok = accel_->readAccel(out);
    } else if (imu_) {
        ok = imu_->readAccel(out);
    }
    if (!ok) return false;
    if (cal_) {
        CalibrationData cal;
        if (cal_->load(SensorId::ACCEL, cal)) {
            cal_->apply(cal, out);
        }
    }
    return true;
}

bool SensorHub::readGyro(GyroData& out) {
    if (!imu_) return false;
    if (!imu_->readGyro(out)) return false;
    if (cal_) {
        CalibrationData cal;
        if (cal_->load(SensorId::GYRO, cal)) {
            cal_->apply(cal, out);
        }
    }
    return true;
}

bool SensorHub::readMag(MagData& out) {
    if (!mag_) return false;
    if (!mag_->readMag(out)) return false;
    if (cal_) {
        CalibrationData cal;
        if (cal_->load(SensorId::MAG, cal)) {
            cal_->apply(cal, out);
        }
    }
    return true;
}

bool SensorHub::readPressure(float& hPa) {
    if (!baro_) return false;
    return baro_->readPressureHPa(hPa);
}

bool SensorHub::readECG(ECGSample& out) {
    if (!ecg_) return false;
    return ecg_->sample(out);
}

bool SensorHub::hasAccel() const { return accel_ != nullptr || imu_ != nullptr; }
bool SensorHub::hasIMU()  const { return imu_ != nullptr; }
bool SensorHub::hasMag()  const { return mag_ != nullptr; }
bool SensorHub::hasBaro() const { return baro_ != nullptr; }
bool SensorHub::hasECG()  const { return ecg_ != nullptr; }

} // namespace sf
