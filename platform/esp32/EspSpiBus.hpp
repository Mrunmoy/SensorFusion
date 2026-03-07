#pragma once

#include "ISPIBus.hpp"
#include "driver/spi_master.h"

namespace sf {

class EspSpiBus : public ISPIBus {
public:
    EspSpiBus(spi_device_handle_t handle, uint8_t readMask = 0x80);

    bool readRegister(uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t reg, const uint8_t* data, size_t len) override;

private:
    spi_device_handle_t handle_;
    uint8_t readMask_;
};

} // namespace sf
