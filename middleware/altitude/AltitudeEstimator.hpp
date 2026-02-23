#pragma once

namespace sf {

class AltitudeEstimator {
public:
    AltitudeEstimator(float tau = 0.5f, float seaLevelHPa = 1013.25f);

    // Update with barometric pressure and vertical acceleration
    void update(float pressureHPa, float accelZ_g, float dt);

    float altitude() const;
    float verticalVelocity() const;
    void reset();

private:
    float alt_ = 0.0f;
    float vel_ = 0.0f;
    float tau_;
    float seaLevelHPa_;

    static float pressureToAltitude(float hPa, float seaLevelHPa);
};

} // namespace sf
