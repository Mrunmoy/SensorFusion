#pragma once

#include "esp_timer.h" // esp_timer_get_time()
#include <stdint.h>

// Simple rate limiter for logs (or any periodic action) in microseconds.
struct RateLogger
{
	uint64_t last_us{0};
	uint64_t interval_us{1'000'000}; // default 1 Hz

	explicit RateLogger(uint64_t interval)
		: last_us(0), interval_us(interval)
	{
	}

	// Preferred: caller passes current time (avoids double syscalls)
	inline bool ready(uint64_t now_us)
	{
		if (last_us == 0 || now_us - last_us >= interval_us)
		{
			last_us = now_us;
			return true;
		}
		return false;
	}

	// Convenience overload: fetch time internally
	inline bool ready()
	{
		return ready(esp_timer_get_time());
	}
};
