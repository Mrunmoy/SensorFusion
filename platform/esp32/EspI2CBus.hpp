#pragma once

#include "II2CBus.hpp"
#include "driver/i2c.h"
#include <cstdint>

namespace sf {

class EspI2CBus : public II2CBus {
public:
    EspI2CBus(i2c_port_t port, uint32_t timeoutMs = 100);

    bool readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) override;
    bool probe(uint8_t devAddr) override;
    bool rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) override;
    bool rawRead(uint8_t devAddr, uint8_t* buf, size_t len) override;

private:
    i2c_port_t port_;
    TickType_t timeoutTicks_;
};

} // namespace sf
