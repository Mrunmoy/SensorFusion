#include "EspDelay.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

namespace sf {

void EspDelay::delayMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void EspDelay::delayUs(uint32_t us) {
    ets_delay_us(us);
}

uint64_t EspDelay::getTimestampUs() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

} // namespace sf
