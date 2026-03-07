#include "NrfDelay.hpp"
#include "NrfSdk.hpp"
#include "app_timer.h"

namespace sf {

void NrfDelay::delayMs(uint32_t ms) {
    while (ms--) {
        nrf_delay_ms(1);
    }
}

void NrfDelay::delayUs(uint32_t us) {
    nrf_delay_us(us);
}

uint64_t NrfDelay::getTimestampUs() {
    // app_timer runs at 32768 Hz on most nRF SDK setups.
    const uint32_t ticks = app_timer_cnt_get();
    return (static_cast<uint64_t>(ticks) * 1000000ull) / 32768ull;
}

} // namespace sf
