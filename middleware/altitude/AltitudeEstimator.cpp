#include "AltitudeEstimator.hpp"
#include <cmath>

namespace sf {

AltitudeEstimator::AltitudeEstimator(float tau, float seaLevelHPa)
    : tau_(tau), seaLevelHPa_(seaLevelHPa)
{}

void AltitudeEstimator::update(float pressureHPa, float accelZ_g, float dt) {
    // Barometric altitude estimate
    float baroAlt = pressureToAltitude(pressureHPa, seaLevelHPa_);

    // Vertical acceleration in m/s^2 (subtract 1g for gravity)
    float accelMs2 = (accelZ_g - 1.0f) * 9.80665f;

    // Complementary filter:
    // High-pass accel (short-term), low-pass baro (long-term)
    float alpha = tau_ / (tau_ + dt);

    // Integrate acceleration for velocity
    vel_ += accelMs2 * dt;

    // Fuse: altitude = alpha * (prev_alt + vel*dt) + (1-alpha) * baroAlt
    alt_ = alpha * (alt_ + vel_ * dt) + (1.0f - alpha) * baroAlt;

    // Damp velocity toward baro-derived rate (prevents drift)
    vel_ *= alpha;
}

float AltitudeEstimator::altitude() const {
    return alt_;
}

float AltitudeEstimator::verticalVelocity() const {
    return vel_;
}

void AltitudeEstimator::reset() {
    alt_ = 0.0f;
    vel_ = 0.0f;
}

float AltitudeEstimator::pressureToAltitude(float hPa, float seaLevelHPa) {
    return 44330.0f * (1.0f - std::pow(hPa / seaLevelHPa, 1.0f / 5.255f));
}

} // namespace sf
