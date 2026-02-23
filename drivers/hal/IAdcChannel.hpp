#pragma once

#include <cstdint>

namespace sf {

class IAdcChannel {
public:
    virtual ~IAdcChannel() = default;
    virtual bool readRaw(int32_t& out) = 0;
    virtual bool readMillivolts(int32_t& out) = 0;
};

} // namespace sf
