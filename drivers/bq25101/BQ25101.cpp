#include "BQ25101.hpp"

namespace sf {

BQ25101::BQ25101(IGpioInput& chgPin, IGpioOutput& tsPin)
    : chgPin_(chgPin), tsPin_(tsPin)
{}

ChargeStatus BQ25101::status() const {
    bool level;
    if (!chgPin_.read(level)) return ChargeStatus::UNKNOWN;
    // CHG pin: open-drain, LOW = charging, HIGH/HI-Z = not charging
    return level ? ChargeStatus::NOT_CHARGING : ChargeStatus::CHARGING;
}

bool BQ25101::setChargeEnable(bool enable) {
    // TS pin: LOW (<92mV) = charge inhibited, HIGH/float = normal operation
    return tsPin_.write(enable);
}

bool BQ25101::isCharging() const {
    return status() == ChargeStatus::CHARGING;
}

} // namespace sf
