#pragma once

#include <cstdint>

namespace sf {

class IDelayProvider {
public:
    virtual ~IDelayProvider() = default;
    virtual void delayMs(uint32_t ms) = 0;
    virtual void delayUs(uint32_t us) = 0;
    virtual uint64_t getTimestampUs() = 0;
};

} // namespace sf
