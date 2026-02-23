#include "MPU6050.hpp"

namespace sf {

namespace {
    constexpr uint8_t SMPLRT_DIV   = 0x19;
    constexpr uint8_t CONFIG       = 0x1A;
    constexpr uint8_t GYRO_CONFIG  = 0x1B;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t INT_PIN_CFG  = 0x37;
    constexpr uint8_t INT_ENABLE   = 0x38;
    constexpr uint8_t INT_STATUS   = 0x3A;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t TEMP_OUT_H   = 0x41;
    constexpr uint8_t GYRO_XOUT_H  = 0x43;
    constexpr uint8_t USER_CTRL    = 0x6A;
    constexpr uint8_t PWR_MGMT_1   = 0x6B;
    constexpr uint8_t WHO_AM_I     = 0x75;
}

MPU6050::MPU6050(II2CBus& bus, IDelayProvider& delay, const MPU6050Config& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg),
      accelScale_(accelLsbPerG(cfg.accelRange)),
      gyroScale_(gyroLsbPerDps(cfg.gyroRange))
{}

float MPU6050::accelLsbPerG(AccelRange r) {
    constexpr float table[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    return table[static_cast<uint8_t>(r)];
}

float MPU6050::gyroLsbPerDps(GyroRange r) {
    constexpr float table[] = {131.0f, 65.5f, 32.8f, 16.4f};
    return table[static_cast<uint8_t>(r)];
}

int16_t MPU6050::sensorToHost16(const uint8_t* buf) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
}

bool MPU6050::init() {
    const uint8_t addr = cfg_.address;

    // Device reset
    if (!bus_.write8(addr, PWR_MGMT_1, 0x80)) return false;
    delay_.delayMs(100);

    // Clock source: PLL with X gyro reference
    if (!bus_.write8(addr, PWR_MGMT_1, 0x01)) return false;

    // Verify WHO_AM_I
    uint8_t id = 0;
    if (!bus_.read8(addr, WHO_AM_I, id)) return false;
    if (id != 0x68) return false;

    // Sample rate divider
    if (!bus_.write8(addr, SMPLRT_DIV, cfg_.sampleRateDiv)) return false;

    // DLPF config
    if (!bus_.write8(addr, CONFIG, static_cast<uint8_t>(cfg_.dlpf))) return false;

    // Gyro range
    if (!bus_.write8(addr, GYRO_CONFIG,
                     static_cast<uint8_t>(cfg_.gyroRange) << 3)) return false;

    // Accel range
    if (!bus_.write8(addr, ACCEL_CONFIG,
                     static_cast<uint8_t>(cfg_.accelRange) << 3)) return false;

    // I2C bypass mode (allows direct access to aux I2C devices like magnetometer)
    if (cfg_.i2cBypass) {
        if (!bus_.write8(addr, USER_CTRL, 0x00)) return false;   // disable I2C master
        if (!bus_.write8(addr, INT_PIN_CFG, 0x02)) return false; // enable bypass
    }

    return true;
}

bool MPU6050::readAccel(AccelData& out) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, ACCEL_XOUT_H, buf, 6)) return false;
    out.x = static_cast<float>(sensorToHost16(&buf[0])) / accelScale_;
    out.y = static_cast<float>(sensorToHost16(&buf[2])) / accelScale_;
    out.z = static_cast<float>(sensorToHost16(&buf[4])) / accelScale_;
    return true;
}

bool MPU6050::readGyro(GyroData& out) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, GYRO_XOUT_H, buf, 6)) return false;
    out.x = static_cast<float>(sensorToHost16(&buf[0])) / gyroScale_;
    out.y = static_cast<float>(sensorToHost16(&buf[2])) / gyroScale_;
    out.z = static_cast<float>(sensorToHost16(&buf[4])) / gyroScale_;
    return true;
}

bool MPU6050::readTemperature(float& tempC) {
    uint8_t buf[2];
    if (!bus_.readRegister(cfg_.address, TEMP_OUT_H, buf, 2)) return false;
    int16_t raw = sensorToHost16(buf);
    tempC = static_cast<float>(raw) / 340.0f + 36.53f;
    return true;
}

bool MPU6050::readAll(AccelData& accel, GyroData& gyro, float& tempC) {
    uint8_t buf[14];
    if (!bus_.readRegister(cfg_.address, ACCEL_XOUT_H, buf, 14)) return false;

    accel.x = static_cast<float>(sensorToHost16(&buf[0])) / accelScale_;
    accel.y = static_cast<float>(sensorToHost16(&buf[2])) / accelScale_;
    accel.z = static_cast<float>(sensorToHost16(&buf[4])) / accelScale_;

    int16_t rawTemp = sensorToHost16(&buf[6]);
    tempC = static_cast<float>(rawTemp) / 340.0f + 36.53f;

    gyro.x = static_cast<float>(sensorToHost16(&buf[8])) / gyroScale_;
    gyro.y = static_cast<float>(sensorToHost16(&buf[10])) / gyroScale_;
    gyro.z = static_cast<float>(sensorToHost16(&buf[12])) / gyroScale_;

    return true;
}

bool MPU6050::enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                       IGpioInterrupt::Callback cb, void* ctx) {
    if (!intPin) return false;
    const uint8_t addr = cfg_.address;

    // INT_PIN_CFG: active low (bit 7), latch until cleared (bit 5),
    // clear on any read (bit 4), preserve bypass if enabled (bit 1)
    uint8_t pinCfg = 0xB0; // active-low | latch | clear-on-read
    if (cfg_.i2cBypass) pinCfg |= 0x02;
    if (!bus_.write8(addr, INT_PIN_CFG, pinCfg)) return false;

    // Enable data ready interrupt
    if (!bus_.write8(addr, INT_ENABLE, 0x01)) return false;

    // Attach MCU-side GPIO interrupt (active low → falling edge)
    if (!intPin->enable(GpioEdge::FALLING, cb, ctx)) return false;

    intPin_ = intPin;
    return true;
}

bool MPU6050::disableDataReadyInterrupt() {
    const uint8_t addr = cfg_.address;

    // Disable all interrupts
    if (!bus_.write8(addr, INT_ENABLE, 0x00)) return false;

    if (intPin_) {
        intPin_->disable();
        intPin_ = nullptr;
    }
    return true;
}

bool MPU6050::clearInterrupt(uint8_t& status) {
    return bus_.read8(cfg_.address, INT_STATUS, status);
}

} // namespace sf
