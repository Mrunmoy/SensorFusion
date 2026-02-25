#include "LIS3MDL.hpp"

namespace sf {

namespace {
    constexpr uint8_t WHO_AM_I   = 0x0F;
    constexpr uint8_t WHO_AM_I_VAL = 0x3D;
    constexpr uint8_t CTRL_REG1  = 0x20;
    constexpr uint8_t CTRL_REG2  = 0x21;
    constexpr uint8_t CTRL_REG3  = 0x22;
    constexpr uint8_t CTRL_REG4  = 0x23;
    constexpr uint8_t CTRL_REG5  = 0x24;
    constexpr uint8_t OUT_X_L    = 0x28;
    constexpr uint8_t TEMP_OUT_L = 0x2E;

    // 1 Gauss = 100 microtesla
    constexpr float GAUSS_TO_UT = 100.0f;
}

LIS3MDL::LIS3MDL(II2CBus& bus, IDelayProvider& delay, const LIS3MDLConfig& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg),
      lsbPerGauss_(lsbPerG(cfg.scale))
{}

float LIS3MDL::lsbPerG(Lis3mdlScale s) {
    switch (s) {
        case Lis3mdlScale::GAUSS_4:  return 6842.0f;
        case Lis3mdlScale::GAUSS_8:  return 3421.0f;
        case Lis3mdlScale::GAUSS_12: return 2281.0f;
        case Lis3mdlScale::GAUSS_16: return 1711.0f;
    }
    return 6842.0f;
}

int16_t LIS3MDL::sensorToHost16(const uint8_t* buf) {
    // Little-endian: LSB first
    return static_cast<int16_t>(
        (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[0]));
}

bool LIS3MDL::init() {
    const uint8_t addr = cfg_.address;

    // Verify WHO_AM_I
    uint8_t whoami;
    if (!bus_.read8(addr, WHO_AM_I, whoami)) return false;
    if (whoami != WHO_AM_I_VAL) return false;

    // CTRL_REG1: TEMP_EN[7] | OM[6:5] | DO[4:2] | FAST_ODR[1] | ST[0]
    // DO=000..111 maps to 0.625..80 Hz. FAST_ODR is only for rates > 80 Hz.
    uint8_t ctrl1 = 0x80 | (0x03 << 5) |
                    (static_cast<uint8_t>(cfg_.odr) << 2);
    if (!bus_.write8(addr, CTRL_REG1, ctrl1)) return false;

    // CTRL_REG2: FS[1:0] (full-scale selection)
    uint8_t ctrl2 = static_cast<uint8_t>(cfg_.scale) << 5;
    if (!bus_.write8(addr, CTRL_REG2, ctrl2)) return false;

    // CTRL_REG3: continuous-conversion mode (MD=00)
    if (!bus_.write8(addr, CTRL_REG3, 0x00)) return false;

    // CTRL_REG4: Z-axis ultra-high performance (OMZ=11), LSB at lower address
    if (!bus_.write8(addr, CTRL_REG4, 0x0C)) return false;

    // CTRL_REG5: BDU=1 (block data update)
    if (!bus_.write8(addr, CTRL_REG5, 0x40)) return false;

    delay_.delayMs(5);
    return true;
}

bool LIS3MDL::readRaw(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buf[6];
    if (!bus_.readRegister(cfg_.address, OUT_X_L, buf, 6)) return false;
    x = sensorToHost16(&buf[0]);
    y = sensorToHost16(&buf[2]);
    z = sensorToHost16(&buf[4]);
    return true;
}

bool LIS3MDL::readMicroTesla(MagData& out) {
    int16_t rx, ry, rz;
    if (!readRaw(rx, ry, rz)) return false;
    float gaussToUt = GAUSS_TO_UT / lsbPerGauss_;
    out.x = static_cast<float>(rx) * gaussToUt;
    out.y = static_cast<float>(ry) * gaussToUt;
    out.z = static_cast<float>(rz) * gaussToUt;
    return true;
}

bool LIS3MDL::readTemperature(float& tempC) {
    uint8_t buf[2];
    if (!bus_.readRegister(cfg_.address, TEMP_OUT_L, buf, 2)) return false;
    int16_t raw = sensorToHost16(buf);
    // 8 LSB/degC, offset at ~25°C
    tempC = 25.0f + static_cast<float>(raw) / 8.0f;
    return true;
}

bool LIS3MDL::enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                       IGpioInterrupt::Callback cb, void* ctx) {
    if (!intPin) return false;

    // DRDY is a dedicated pin (separate from INT) that goes low when new
    // data is available. It requires no register configuration — just
    // attach the GPIO edge callback.
    if (!intPin->enable(GpioEdge::FALLING, cb, ctx)) return false;

    intPin_ = intPin;
    return true;
}

bool LIS3MDL::disableDataReadyInterrupt() {
    if (intPin_) {
        intPin_->disable();
        intPin_ = nullptr;
    }
    return true;
}

} // namespace sf
