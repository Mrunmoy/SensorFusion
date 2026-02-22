#include "LSM6DSO.hpp"

namespace sf {

namespace reg {
    constexpr uint8_t INT1_CTRL   = 0x0D;
    constexpr uint8_t INT2_CTRL   = 0x0E;
    constexpr uint8_t WHO_AM_I    = 0x0F;
    constexpr uint8_t CTRL1_XL    = 0x10;
    constexpr uint8_t CTRL2_G     = 0x11;
    constexpr uint8_t CTRL3_C     = 0x12;
    constexpr uint8_t STATUS_REG  = 0x1E;
    constexpr uint8_t OUT_TEMP_L  = 0x20;
    constexpr uint8_t OUTX_L_G   = 0x22;
    constexpr uint8_t OUTX_L_A   = 0x28;
}

static constexpr uint8_t EXPECTED_WHO_AM_I = 0x6C;

LSM6DSO::LSM6DSO(II2CBus& bus, IDelayProvider& delay, const LSM6DSOConfig& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg),
      accelSens_(accelSensitivity(cfg.accelRange)),
      gyroSens_(gyroSensitivity(cfg.gyroRange))
{}

float LSM6DSO::accelSensitivity(LsmAccelRange r) {
    // mg per LSB
    switch (r) {
        case LsmAccelRange::G2:  return 0.061f;
        case LsmAccelRange::G4:  return 0.122f;
        case LsmAccelRange::G8:  return 0.244f;
        case LsmAccelRange::G16: return 0.488f;
    }
    return 0.061f;
}

float LSM6DSO::gyroSensitivity(LsmGyroRange r) {
    // mdps per LSB
    switch (r) {
        case LsmGyroRange::DPS125:  return 4.375f;
        case LsmGyroRange::DPS250:  return 8.750f;
        case LsmGyroRange::DPS500:  return 17.50f;
        case LsmGyroRange::DPS1000: return 35.0f;
        case LsmGyroRange::DPS2000: return 70.0f;
    }
    return 8.750f;
}

int16_t LSM6DSO::sensorToHost16(const uint8_t* buf) {
    // Little-endian: L, H
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

bool LSM6DSO::init() {
    const uint8_t addr = cfg_.address;

    // Software reset: CTRL3_C bit 0
    if (!bus_.write8(addr, reg::CTRL3_C, 0x01)) return false;
    delay_.delayMs(10);

    // Verify WHO_AM_I
    uint8_t id;
    if (!bus_.read8(addr, reg::WHO_AM_I, id)) return false;
    if (id != EXPECTED_WHO_AM_I) return false;

    // CTRL3_C: BDU (bit 6) + IF_INC (bit 2)
    if (!bus_.write8(addr, reg::CTRL3_C, 0x44)) return false;

    // CTRL1_XL: ODR_XL[7:4] | FS_XL[3:2]
    uint8_t ctrl1 = (static_cast<uint8_t>(cfg_.accelOdr) << 4) |
                     (static_cast<uint8_t>(cfg_.accelRange) << 2);
    if (!bus_.write8(addr, reg::CTRL1_XL, ctrl1)) return false;

    // CTRL2_G: ODR_G[7:4] | FS_G[3:1]
    uint8_t ctrl2 = (static_cast<uint8_t>(cfg_.gyroOdr) << 4) |
                     (static_cast<uint8_t>(cfg_.gyroRange) << 1);
    if (!bus_.write8(addr, reg::CTRL2_G, ctrl2)) return false;

    return true;
}

bool LSM6DSO::readAccel(AccelData& out) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, reg::OUTX_L_A, buf, 6)) return false;

    out.x = static_cast<float>(sensorToHost16(&buf[0])) * accelSens_ * 0.001f;
    out.y = static_cast<float>(sensorToHost16(&buf[2])) * accelSens_ * 0.001f;
    out.z = static_cast<float>(sensorToHost16(&buf[4])) * accelSens_ * 0.001f;
    return true;
}

bool LSM6DSO::readGyro(GyroData& out) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, reg::OUTX_L_G, buf, 6)) return false;

    out.x = static_cast<float>(sensorToHost16(&buf[0])) * gyroSens_ * 0.001f;
    out.y = static_cast<float>(sensorToHost16(&buf[2])) * gyroSens_ * 0.001f;
    out.z = static_cast<float>(sensorToHost16(&buf[4])) * gyroSens_ * 0.001f;
    return true;
}

bool LSM6DSO::readTemperature(float& tempC) {
    uint8_t buf[2];
    if (!bus_.readRegister(cfg_.address, reg::OUT_TEMP_L, buf, 2)) return false;
    int16_t raw = sensorToHost16(buf);
    tempC = static_cast<float>(raw) / 256.0f + 25.0f;
    return true;
}

bool LSM6DSO::readAll(AccelData& accel, GyroData& gyro, float& tempC) {
    // Read 14 bytes: temp(2) + gyro(6) + accel(6) from OUT_TEMP_L (0x20)
    uint8_t buf[14];
    if (!bus_.readRegister(cfg_.address, reg::OUT_TEMP_L, buf, 14)) return false;

    // Temperature: bytes 0-1
    int16_t rawTemp = sensorToHost16(&buf[0]);
    tempC = static_cast<float>(rawTemp) / 256.0f + 25.0f;

    // Gyro: bytes 2-7
    gyro.x = static_cast<float>(sensorToHost16(&buf[2])) * gyroSens_ * 0.001f;
    gyro.y = static_cast<float>(sensorToHost16(&buf[4])) * gyroSens_ * 0.001f;
    gyro.z = static_cast<float>(sensorToHost16(&buf[6])) * gyroSens_ * 0.001f;

    // Accel: bytes 8-13
    accel.x = static_cast<float>(sensorToHost16(&buf[8])) * accelSens_ * 0.001f;
    accel.y = static_cast<float>(sensorToHost16(&buf[10])) * accelSens_ * 0.001f;
    accel.z = static_cast<float>(sensorToHost16(&buf[12])) * accelSens_ * 0.001f;

    return true;
}

bool LSM6DSO::enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                       IGpioInterrupt::Callback cb, void* ctx) {
    if (!intPin) return false;
    const uint8_t addr = cfg_.address;

    // INT1_CTRL: INT1_DRDY_XL (bit 0) + INT1_DRDY_G (bit 1)
    if (!bus_.write8(addr, reg::INT1_CTRL, 0x03)) return false;

    // Default: active-high, push-pull (CTRL3_C H_LACTIVE=0, PP_OD=0)
    if (!intPin->enable(GpioEdge::RISING, cb, ctx)) return false;

    intPin_ = intPin;
    return true;
}

bool LSM6DSO::disableDataReadyInterrupt() {
    const uint8_t addr = cfg_.address;

    // Disable all INT1 sources
    if (!bus_.write8(addr, reg::INT1_CTRL, 0x00)) return false;

    if (intPin_) {
        intPin_->disable();
        intPin_ = nullptr;
    }
    return true;
}

} // namespace sf
