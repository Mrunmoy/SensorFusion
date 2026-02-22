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

    bool read16BE(uint8_t devAddr, uint8_t reg, uint16_t& out) {
        uint8_t buf[2];
        if (!readRegister(devAddr, reg, buf, 2)) return false;
        out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
        return true;
    }

    bool read16LE(uint8_t devAddr, uint8_t reg, uint16_t& out) {
        uint8_t buf[2];
        if (!readRegister(devAddr, reg, buf, 2)) return false;
        out = static_cast<uint16_t>((buf[1] << 8) | buf[0]);
        return true;
    }

    bool write16BE(uint8_t devAddr, uint8_t reg, uint16_t val) {
        uint8_t buf[2] = {
            static_cast<uint8_t>(val >> 8),
            static_cast<uint8_t>(val & 0xFF)
        };
        return writeRegister(devAddr, reg, buf, 2);
    }
};

} // namespace sf
