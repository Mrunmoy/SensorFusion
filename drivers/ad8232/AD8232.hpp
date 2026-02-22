#pragma once

#include "IAdcChannel.hpp"
#include "IDelayProvider.hpp"
#include <cstdint>

namespace sf {

struct ECGSample {
    int32_t  millivolts;
    uint64_t timestampUs;
};

class AD8232 {
public:
    AD8232(IAdcChannel& adc, IDelayProvider& delay);

    bool readRaw(int32_t& out);
    bool readMillivolts(int32_t& out);
    bool sample(ECGSample& out);

private:
    IAdcChannel& adc_;
    IDelayProvider& delay_;
};

} // namespace sf
