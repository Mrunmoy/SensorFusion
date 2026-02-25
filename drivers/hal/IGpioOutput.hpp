#pragma once

#include <cstdint>

namespace sf {

class IGpioOutput {
public:
    virtual ~IGpioOutput() = default;
    virtual bool write(bool level) = 0;
};

} // namespace sf
