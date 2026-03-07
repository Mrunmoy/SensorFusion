#pragma once

#include "II2CBus.hpp"
#include "NrfSdk.hpp"

namespace sf {

class NrfTwimBus : public II2CBus {
public:
    explicit NrfTwimBus(const nrfx_twim_t& twim) : twim_(twim) {}

    bool readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) override;
    bool probe(uint8_t devAddr) override;
    bool rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) override;
    bool rawRead(uint8_t devAddr, uint8_t* buf, size_t len) override;

private:
    nrfx_twim_t twim_;
};

} // namespace sf
