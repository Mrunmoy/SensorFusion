#pragma once

#include "IDelayProvider.hpp"

namespace sf {

class EspDelay : public IDelayProvider {
public:
    void delayMs(uint32_t ms) override;
    void delayUs(uint32_t us) override;
    uint64_t getTimestampUs() override;
};

} // namespace sf
