#include "StmAdcChannel.hpp"

namespace sf {

bool StmAdcChannel::readRaw(int32_t& out) {
    if (!hadc_) return false;
    if (HAL_ADC_Start(hadc_) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(hadc_, 100) != HAL_OK) return false;
    out = static_cast<int32_t>(HAL_ADC_GetValue(hadc_));
    HAL_ADC_Stop(hadc_);
    return true;
}

bool StmAdcChannel::readMillivolts(int32_t& out) {
    int32_t raw = 0;
    if (!readRaw(raw)) return false;

#ifdef VREFINT_CAL
    constexpr int32_t vrefMv = 3300;
#else
    constexpr int32_t vrefMv = 3300;
#endif

#if defined(ADC_RESOLUTION_12B)
    constexpr int32_t maxRaw = 4095;
#else
    constexpr int32_t maxRaw = 4095;
#endif

    out = (raw * vrefMv) / maxRaw;
    return true;
}

} // namespace sf
