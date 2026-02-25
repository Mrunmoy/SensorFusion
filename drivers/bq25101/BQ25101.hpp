#pragma once

#include "IGpioInput.hpp"
#include "IGpioOutput.hpp"
#include <cstdint>

namespace sf {

enum class ChargeStatus : uint8_t {
    CHARGING,
    NOT_CHARGING,
    UNKNOWN
};

class BQ25101 {
public:
    // chgPin: open-drain CHG output (LOW = charging, HI-Z = not charging)
    // tsPin:  TS input (LOW = charge inhibited, HIGH/float = normal operation)
    BQ25101(IGpioInput& chgPin, IGpioOutput& tsPin);

    ChargeStatus status() const;
    bool setChargeEnable(bool enable);
    bool isCharging() const;

private:
    IGpioInput& chgPin_;
    IGpioOutput& tsPin_;
};

} // namespace sf
