#pragma once

#include <cstddef>
#include <cstdint>

namespace sf {

// CRC-8 used by Sensirion sensors (SHT40, SGP40, etc.)
// Polynomial: 0x31, Init: 0xFF, No final XOR
static inline uint8_t crc8Sensirion(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

} // namespace sf
