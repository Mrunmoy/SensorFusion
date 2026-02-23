#pragma once

#include "II2CBus.hpp"
#include "IDelayProvider.hpp"
#include "IGpioInterrupt.hpp"
#include "SensorTypes.hpp"
#include <cstdint>

namespace sf {

enum class Bmm350Odr : uint8_t {
    HZ_400   = 0x02,
    HZ_200   = 0x03,
    HZ_100   = 0x04,
    HZ_50    = 0x05,
    HZ_25    = 0x06,
    HZ_12_5  = 0x07,
    HZ_6_25  = 0x08,
    HZ_3_125 = 0x09,
    HZ_1_5625 = 0x0A,
};

enum class Bmm350Avg : uint8_t {
    NO_AVG = 0x00,
    AVG_2  = 0x01,
    AVG_4  = 0x02,
    AVG_8  = 0x03,
};

struct BMM350Config {
    Bmm350Odr odr     = Bmm350Odr::HZ_100;
    Bmm350Avg avg     = Bmm350Avg::NO_AVG;
    uint8_t   address = 0x14;
};

struct Bmm350OtpData {
    float offsetX;
    float offsetY;
    float offsetZ;
    float sensX;
    float sensY;
    float sensZ;
    float tcoX;
    float tcoY;
    float tcoZ;
    float tcsX;
    float tcsY;
    float tcsZ;
    float crossXY;
    float crossYX;
    float t0;
};

class BMM350 {
public:
    BMM350(II2CBus& bus, IDelayProvider& delay, const BMM350Config& cfg = {});

    bool init();
    bool readMag(MagData& out);
    bool readTemperature(float& tempC);

    bool enableDataReadyInterrupt(IGpioInterrupt* intPin,
                                  IGpioInterrupt::Callback cb, void* ctx);
    bool disableDataReadyInterrupt();

    const Bmm350OtpData& otpData() const { return otp_; }

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    BMM350Config cfg_;
    Bmm350OtpData otp_{};

    IGpioInterrupt* intPin_ = nullptr;

    bool readOtp();
    bool readOtpWord(uint8_t wordAddr, uint16_t& out);
    bool setNormalMode();

    static int32_t signExtend21(uint32_t raw);
    static int16_t sensorToHost16(const uint8_t* buf);
    static int32_t sensorToHost24(const uint8_t* buf);
};

} // namespace sf
