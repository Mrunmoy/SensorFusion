#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorTypes.hpp"
#include <cstdint>

namespace sf {

enum class LsmAccelRange : uint8_t { G2 = 0, G4 = 2, G8 = 3, G16 = 1 };
enum class LsmGyroRange  : uint8_t { DPS125 = 1, DPS250 = 0, DPS500 = 2, DPS1000 = 4, DPS2000 = 6 };

enum class LsmOdr : uint8_t {
    POWER_DOWN = 0x00,
    HZ_12_5    = 0x01,
    HZ_26      = 0x02,
    HZ_52      = 0x03,
    HZ_104     = 0x04,
    HZ_208     = 0x05,
    HZ_416     = 0x06,
    HZ_833     = 0x07,
    HZ_1666    = 0x08,
    HZ_3332    = 0x09,
    HZ_6664    = 0x0A,
};

struct LSM6DSOConfig {
    LsmAccelRange accelRange = LsmAccelRange::G2;
    LsmGyroRange  gyroRange  = LsmGyroRange::DPS250;
    LsmOdr        accelOdr   = LsmOdr::HZ_104;
    LsmOdr        gyroOdr    = LsmOdr::HZ_104;
    uint8_t       address    = 0x6A;
};

class LSM6DSO {
public:
    LSM6DSO(II2CBus& bus, IDelayProvider& delay, const LSM6DSOConfig& cfg = {});

    bool init();
    bool readAccel(AccelData& out);
    bool readGyro(GyroData& out);
    bool readTemperature(float& tempC);
    bool readAll(AccelData& accel, GyroData& gyro, float& tempC);

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    LSM6DSOConfig cfg_;
    float accelSens_;   // mg per LSB
    float gyroSens_;    // mdps per LSB

    IGpioInterrupt* intPin_ = nullptr;

    static float accelSensitivity(LsmAccelRange r);
    static float gyroSensitivity(LsmGyroRange r);
    static int16_t sensorToHost16(const uint8_t* buf);
};

} // namespace sf
