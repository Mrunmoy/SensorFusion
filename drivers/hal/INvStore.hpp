#pragma once

#include <cstddef>
#include <cstdint>

namespace sf {

class INvStore {
public:
    virtual ~INvStore() = default;

    virtual bool read(uint32_t address, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint32_t address, const uint8_t* data, size_t len) = 0;
    virtual size_t capacity() const = 0;
};

} // namespace sf
