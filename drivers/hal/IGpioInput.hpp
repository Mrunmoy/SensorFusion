#pragma once

#include <cstdint>

namespace sf {

class IGpioInput {
public:
    virtual ~IGpioInput() = default;
    virtual bool read(bool& level) = 0;
};

} // namespace sf
