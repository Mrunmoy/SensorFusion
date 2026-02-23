#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorInterface.hpp"
#include <cstdint>

namespace sf {

enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };
enum class GyroRange  : uint8_t { DPS250 = 0, DPS500 = 1, DPS1000 = 2, DPS2000 = 3 };
enum class DlpfBandwidth : uint8_t {
    BW260 = 0, BW184 = 1, BW94 = 2, BW44 = 3,
    BW21 = 4, BW10 = 5, BW5 = 6
};

struct MPU6050Config {
    AccelRange    accelRange    = AccelRange::G2;
    GyroRange     gyroRange     = GyroRange::DPS250;
    DlpfBandwidth dlpf          = DlpfBandwidth::BW44;
    uint8_t       sampleRateDiv = 4;
    bool          i2cBypass     = true;
    uint8_t       address       = 0x68;
};

class MPU6050 : public IAccelGyroSensor {
public:
    MPU6050(II2CBus& bus, IDelayProvider& delay, const MPU6050Config& cfg = {});

    bool init();
    bool readAccel(AccelData& out) override;
    bool readGyro(GyroData& out) override;
    bool readTemperature(float& tempC) override;
    bool readAll(AccelData& accel, GyroData& gyro, float& tempC);

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();
    bool clearInterrupt(uint8_t& status);

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    MPU6050Config cfg_;
    float accelScale_;
    float gyroScale_;

    IGpioInterrupt* intPin_ = nullptr;

    static float accelLsbPerG(AccelRange r);
    static float gyroLsbPerDps(GyroRange r);
    static int16_t sensorToHost16(const uint8_t* buf);
};

} // namespace sf
