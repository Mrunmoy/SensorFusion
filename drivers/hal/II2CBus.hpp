#pragma once

#include <cstddef>
#include <cstdint>

namespace sf {

class II2CBus {
public:
    virtual ~II2CBus() = default;

    virtual bool readRegister(uint8_t devAddr, uint8_t reg,
                              uint8_t* buf, size_t len) = 0;

    virtual bool writeRegister(uint8_t devAddr, uint8_t reg,
                               const uint8_t* data, size_t len) = 0;

    virtual bool probe(uint8_t devAddr) = 0;

    // --- Non-virtual convenience wrappers ---

    bool read8(uint8_t devAddr, uint8_t reg, uint8_t& out) {
        return readRegister(devAddr, reg, &out, 1);
    }

    bool write8(uint8_t devAddr, uint8_t reg, uint8_t val) {
        return writeRegister(devAddr, reg, &val, 1);
    }

    // Multi-byte reads/writes: use readRegister/writeRegister directly.
    // Each driver handles its own sensor-to-host byte order conversion
    // internally via sensorToHost16() — no endianness leaks into the bus API.

    // --- Command-based I2C (no register address) ---
    // Used by sensors like SHT40/SGP40 that use raw command bytes.
    // Default implementations return false so existing platform HALs still compile.

    virtual bool rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) {
        (void)devAddr; (void)data; (void)len;
        return false;
    }

    virtual bool rawRead(uint8_t devAddr, uint8_t* buf, size_t len) {
        (void)devAddr; (void)buf; (void)len;
        return false;
    }
};

} // namespace sf
