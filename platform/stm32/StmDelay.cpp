#include "StmDelay.hpp"
#include "StmHal.hpp"

namespace sf {

void StmDelay::delayMs(uint32_t ms) {
    HAL_Delay(ms);
}

void StmDelay::delayUs(uint32_t us) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

uint64_t StmDelay::getTimestampUs() {
    const uint64_t ms = HAL_GetTick();
    return ms * 1000ull;
}

} // namespace sf
