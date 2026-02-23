#include "BMM350.hpp"

namespace sf {

namespace reg {
    constexpr uint8_t CHIP_ID         = 0x00;
    constexpr uint8_t PMU_CMD_AGGR    = 0x04;
    constexpr uint8_t PMU_CMD_AXIS_EN = 0x05;
    constexpr uint8_t PMU_CMD         = 0x06;
    constexpr uint8_t PMU_CMD_STATUS  = 0x07;
    constexpr uint8_t INT_CTRL        = 0x2E;
    constexpr uint8_t INT_STATUS      = 0x30;
    constexpr uint8_t MAG_X_XLSB     = 0x31;
    constexpr uint8_t TEMP_XLSB      = 0x3A;
    constexpr uint8_t OTP_CMD         = 0x50;
    constexpr uint8_t OTP_DATA_MSB    = 0x52;
    constexpr uint8_t OTP_DATA_LSB    = 0x53;
    constexpr uint8_t OTP_STATUS      = 0x55;
    constexpr uint8_t CMD             = 0x7E;
}

static constexpr uint8_t EXPECTED_CHIP_ID = 0x33;
static constexpr uint8_t PMU_SUSPEND = 0x00;
static constexpr uint8_t PMU_NORMAL  = 0x01;

// OTP word addresses for calibration data (simplified subset)
static constexpr uint8_t OTP_WORD_OFF_X   = 0x0E;
static constexpr uint8_t OTP_WORD_OFF_Y   = 0x0F;
static constexpr uint8_t OTP_WORD_OFF_Z   = 0x10; // packs offsetZ (high byte) + sensX (low byte)
static constexpr uint8_t OTP_WORD_SENS_Y  = 0x11;
static constexpr uint8_t OTP_WORD_TCO_X   = 0x12;
static constexpr uint8_t OTP_WORD_TCO_Y   = 0x13;
static constexpr uint8_t OTP_WORD_T0      = 0x0D;

// Conversion constants from Bosch API
static constexpr float TEMP_SCALE  = 0.00204f;
static constexpr float TEMP_OFFSET = -25.49f;
static constexpr float UT_SCALE_XY = 14.55f;
static constexpr float UT_SCALE_Z  = 9.0f;

BMM350::BMM350(II2CBus& bus, IDelayProvider& delay, const BMM350Config& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg)
{}

int16_t BMM350::sensorToHost16(const uint8_t* buf) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[0]));
}

int32_t BMM350::sensorToHost24(const uint8_t* buf) {
    // Little-endian: XLSB, LSB, MSB
    uint32_t raw = static_cast<uint32_t>(buf[0]) |
                   (static_cast<uint32_t>(buf[1]) << 8) |
                   (static_cast<uint32_t>(buf[2]) << 16);
    return signExtend21(raw);
}

int32_t BMM350::signExtend21(uint32_t raw) {
    // Mask to 21 bits and sign-extend
    raw &= 0x1FFFFF;
    if (raw & 0x100000) {
        return static_cast<int32_t>(raw | 0xFFE00000);
    }
    return static_cast<int32_t>(raw);
}

bool BMM350::readOtpWord(uint8_t wordAddr, uint16_t& out) {
    const uint8_t addr = cfg_.address;

    // Write OTP word address with read command (bit 5 = DIR_READ, bits 4:0 = word addr)
    uint8_t cmd = 0x20 | (wordAddr & 0x1F);
    if (!bus_.write8(addr, reg::OTP_CMD, cmd)) return false;

    delay_.delayUs(300);

    // Check OTP status
    uint8_t status;
    if (!bus_.read8(addr, reg::OTP_STATUS, status)) return false;
    if ((status & 0x01) == 0) return false; // OTP error

    // Read data
    uint8_t msb, lsb;
    if (!bus_.read8(addr, reg::OTP_DATA_MSB, msb)) return false;
    if (!bus_.read8(addr, reg::OTP_DATA_LSB, lsb)) return false;

    out = static_cast<uint16_t>((msb << 8) | lsb);
    return true;
}

bool BMM350::readOtp() {
    // Read key OTP words and populate compensation structure
    uint16_t w;

    // T0 reference temperature
    if (!readOtpWord(OTP_WORD_T0, w)) return false;
    otp_.t0 = static_cast<float>(static_cast<int16_t>(w)) * 0.01f;

    // Offsets
    if (!readOtpWord(OTP_WORD_OFF_X, w)) return false;
    otp_.offsetX = static_cast<float>(static_cast<int16_t>(w));
    if (!readOtpWord(OTP_WORD_OFF_Y, w)) return false;
    otp_.offsetY = static_cast<float>(static_cast<int16_t>(w));

    // Z offset and X sensitivity share a word
    if (!readOtpWord(OTP_WORD_OFF_Z, w)) return false;
    otp_.offsetZ = static_cast<float>(static_cast<int16_t>(w & 0xFF00)) / 256.0f;
    otp_.sensX = static_cast<float>(static_cast<int8_t>(w & 0xFF)) / 256.0f;

    // Y sensitivity
    if (!readOtpWord(OTP_WORD_SENS_Y, w)) return false;
    otp_.sensY = static_cast<float>(static_cast<int8_t>(w & 0xFF)) / 256.0f;
    otp_.sensZ = 0.0f; // Z sensitivity not separately stored

    // TCO (temperature coefficient of offset)
    if (!readOtpWord(OTP_WORD_TCO_X, w)) return false;
    otp_.tcoX = static_cast<float>(static_cast<int8_t>(w & 0xFF)) / 32.0f;
    otp_.tcoY = static_cast<float>(static_cast<int8_t>((w >> 8) & 0xFF)) / 32.0f;
    otp_.tcoZ = 0.0f;

    // TCS and cross-axis default to 0 (simplified)
    otp_.tcsX = 0.0f;
    otp_.tcsY = 0.0f;
    otp_.tcsZ = 0.0f;
    otp_.crossXY = 0.0f;
    otp_.crossYX = 0.0f;

    return true;
}

bool BMM350::setNormalMode() {
    const uint8_t addr = cfg_.address;

    // Enable all axes
    if (!bus_.write8(addr, reg::PMU_CMD_AXIS_EN, 0x07)) return false;

    // Set ODR and averaging
    uint8_t aggr = (static_cast<uint8_t>(cfg_.avg) << 4) |
                    static_cast<uint8_t>(cfg_.odr);
    if (!bus_.write8(addr, reg::PMU_CMD_AGGR, aggr)) return false;

    // Command: normal mode
    if (!bus_.write8(addr, reg::PMU_CMD, PMU_NORMAL)) return false;
    delay_.delayMs(38); // Wait for mode transition

    // Verify mode
    uint8_t status;
    if (!bus_.read8(addr, reg::PMU_CMD_STATUS, status)) return false;
    if ((status & 0x03) != PMU_NORMAL) return false;

    return true;
}

bool BMM350::init() {
    const uint8_t addr = cfg_.address;

    // Soft reset
    if (!bus_.write8(addr, reg::CMD, 0xB6)) return false;
    delay_.delayMs(24);

    // Verify CHIP_ID
    uint8_t id;
    if (!bus_.read8(addr, reg::CHIP_ID, id)) return false;
    if (id != EXPECTED_CHIP_ID) return false;

    // Read OTP calibration data
    if (!readOtp()) return false;

    // Set normal mode
    if (!setNormalMode()) return false;

    return true;
}

bool BMM350::readMag(MagData& out) {
    // Read 9 bytes: X(3) + Y(3) + Z(3) starting at MAG_X_XLSB
    uint8_t buf[9];
    if (!bus_.readRegister(cfg_.address, reg::MAG_X_XLSB, buf, 9)) return false;

    int32_t rawX = sensorToHost24(&buf[0]);
    int32_t rawY = sensorToHost24(&buf[3]);
    int32_t rawZ = sensorToHost24(&buf[6]);

    // Apply compensation: offset + sensitivity correction
    float fx = (static_cast<float>(rawX) + otp_.offsetX) * (1.0f + otp_.sensX);
    float fy = (static_cast<float>(rawY) + otp_.offsetY) * (1.0f + otp_.sensY);
    float fz = (static_cast<float>(rawZ) + otp_.offsetZ) * (1.0f + otp_.sensZ);

    // Convert to µT
    out.x = fx / UT_SCALE_XY;
    out.y = fy / UT_SCALE_XY;
    out.z = fz / UT_SCALE_Z;

    return true;
}

bool BMM350::readTemperature(float& tempC) {
    // Temperature is 3 bytes starting at MAG_X_XLSB + 9
    uint8_t buf[3];
    if (!bus_.readRegister(cfg_.address, reg::TEMP_XLSB, buf, 3)) return false;

    int32_t raw = sensorToHost24(buf);
    tempC = static_cast<float>(raw) * TEMP_SCALE + TEMP_OFFSET;
    return true;
}

bool BMM350::enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                      IGpioInterrupt::Callback cb, void* ctx) {
    if (!intPin) return false;
    const uint8_t addr = cfg_.address;

    // INT_CTRL: enable DRDY interrupt, push-pull, active high
    if (!bus_.write8(addr, reg::INT_CTRL, 0x07)) return false;

    if (!intPin->enable(GpioEdge::RISING, cb, ctx)) return false;

    intPin_ = intPin;
    return true;
}

bool BMM350::disableDataReadyInterrupt() {
    const uint8_t addr = cfg_.address;

    // Disable interrupts
    if (!bus_.write8(addr, reg::INT_CTRL, 0x00)) return false;

    if (intPin_) {
        intPin_->disable();
        intPin_ = nullptr;
    }
    return true;
}

} // namespace sf
