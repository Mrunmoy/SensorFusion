#pragma once

#include <cstdint>

namespace sf {

enum class GpioEdge : uint8_t { RISING, FALLING, BOTH };

class IGpioInterrupt {
public:
    virtual ~IGpioInterrupt() = default;

    using Callback = void(*)(void* context);

    virtual bool enable(GpioEdge edge, Callback cb, void* context) = 0;
    virtual bool disable() = 0;
};

} // namespace sf
