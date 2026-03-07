#pragma once

#include "ISPIBus.hpp"
#include "NrfSdk.hpp"

namespace sf {

class NrfSpimBus : public ISPIBus {
public:
    NrfSpimBus(const nrfx_spim_t& spim, uint8_t readMask = 0x80)
        : spim_(spim), readMask_(readMask) {}

    bool readRegister(uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t reg, const uint8_t* data, size_t len) override;

private:
    nrfx_spim_t spim_;
    uint8_t readMask_;
};

} // namespace sf
