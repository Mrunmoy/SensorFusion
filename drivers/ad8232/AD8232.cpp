#include "AD8232.hpp"

namespace sf {

AD8232::AD8232(IAdcChannel& adc, IDelayProvider& delay)
    : adc_(adc), delay_(delay)
{}

bool AD8232::readRaw(int32_t& out) {
    return adc_.readRaw(out);
}

bool AD8232::readMillivolts(int32_t& out) {
    return adc_.readMillivolts(out);
}

bool AD8232::sample(ECGSample& out) {
    if (!adc_.readMillivolts(out.millivolts)) return false;
    out.timestampUs = delay_.getTimestampUs();
    return true;
}

} // namespace sf
