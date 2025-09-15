#pragma once

#include "esp_rom_sys.h" // esp_rom_delay_us()
#include "esp_timer.h"	 // esp_timer_get_time()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

// µs-precision pacer for high-rate loops (e.g., 200 Hz IMU/Fusion)
struct MicroPacer
{
	int32_t period_us{10000}; // default 100 Hz
	int64_t next_us{0};
};

inline MicroPacer make_pacer(float hz)
{
	MicroPacer p;
	if (hz > 0.f)
		p.period_us = std::max(1, static_cast<int>(std::lround(1e6f / hz)));
	p.next_us = esp_timer_get_time();
	return p;
}

// Sleep until next period boundary. Uses ms sleep for long gaps and short
// busy-wait for sub-ms.
inline void sleep_until(MicroPacer &p)
{
	p.next_us += p.period_us;
	const int64_t now = esp_timer_get_time();
	int64_t delta = p.next_us - now;

	if (delta > 2000)
	{ // >2 ms: yield to RTOS
		vTaskDelay(pdMS_TO_TICKS(delta / 1000));
	}
	else if (delta > 0)
	{ // <2 ms: short delay
		esp_rom_delay_us(static_cast<uint32_t>(delta));
	}
	else
	{ // we're late
		if (-delta > p.period_us)
			p.next_us = esp_timer_get_time(); // realign if very late
		taskYIELD();
	}
}
