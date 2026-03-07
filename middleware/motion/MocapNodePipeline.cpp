#include "MocapNodePipeline.hpp"

namespace sf {

MocapNodePipeline::MocapNodePipeline(IAccelGyroSensor& imu,
                                     IMagSensor* mag,
                                     IBaroSensor* baro)
    : MocapNodePipeline(imu, mag, baro, Config{})
{}

MocapNodePipeline::MocapNodePipeline(IAccelGyroSensor& imu,
                                     IMagSensor* mag,
                                     IBaroSensor* baro,
                                     const Config& cfg)
    : imu_(imu),
      mag_(mag),
      baro_(baro),
      cfg_(cfg),
      ahrs_(cfg.mahonyKp, cfg.mahonyKi)
{}

bool MocapNodePipeline::step(MocapNodeSample& out) {
    if (!imu_.readAccel(out.accel)) return false;
    if (!imu_.readGyro(out.gyro)) return false;

    out.hasMag = false;
    if (cfg_.preferMag && mag_ != nullptr && mag_->readMag(out.mag)) {
        out.hasMag = true;
        ahrs_.update(out.accel, out.gyro, out.mag, cfg_.dtSeconds);
    } else {
        ahrs_.update6DOF(out.accel, out.gyro, cfg_.dtSeconds);
    }

    out.hasPressure = false;
    if (baro_ != nullptr && baro_->readPressureHPa(out.pressureHPa)) {
        out.hasPressure = true;
    }

    out.orientation = ahrs_.getQuaternion();
    return true;
}

} // namespace sf
