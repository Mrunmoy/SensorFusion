#pragma once

#include <cstddef>
#include <cstdint>

namespace sf {

/// SPI bus abstraction for a single device.
/// Each ISPIBus instance manages chip-select for one device.
/// The platform implementation must assert CS before and deassert CS after
/// each readRegister/writeRegister call.
class ISPIBus {
public:
    virtual ~ISPIBus() = default;

    virtual bool readRegister(uint8_t reg, uint8_t* buf, size_t len) = 0;

    virtual bool writeRegister(uint8_t reg, const uint8_t* data, size_t len) = 0;

    // --- Non-virtual convenience wrappers ---

    bool read8(uint8_t reg, uint8_t& out) {
        return readRegister(reg, &out, 1);
    }

    bool write8(uint8_t reg, uint8_t val) {
        return writeRegister(reg, &val, 1);
    }
};

} // namespace sf
