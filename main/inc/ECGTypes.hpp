#pragma once

#include <cstdint>

struct ECGSample
{
    int32_t  mv = -1;   // -1 if uncalibrated; else millivolts
    uint64_t t_us = 0;  // esp_timer_get_time()
};
