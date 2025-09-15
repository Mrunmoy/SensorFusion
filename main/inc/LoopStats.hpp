#pragma once

#include "esp_timer.h"
#include <stdint.h>

struct LoopStats
{
	uint64_t count{0};
	int64_t last_t{0};
	int64_t min_dt{INT64_MAX};
	int64_t max_dt{0};
	double sum_dt{0.0};

	inline void reset()
	{
		count = 0;
		last_t = 0;
		min_dt = INT64_MAX;
		max_dt = 0;
		sum_dt = 0.0;
	}

	inline void tick()
	{
		int64_t now = esp_timer_get_time();
		if (last_t != 0)
		{
			int64_t dt = now - last_t;
			if (dt < min_dt)
				min_dt = dt;
			if (dt > max_dt)
				max_dt = dt;
			sum_dt += (double)dt;
			++count;
		}
		last_t = now;
	}

	inline void snapshot(uint64_t &out_count, double &avg_us, int64_t &min_us,
						 int64_t &max_us)
	{
		out_count = count;
		avg_us = (count ? (sum_dt / (double)count) : 0.0);
		min_us = (count ? min_dt : 0);
		max_us = (count ? max_dt : 0);
	}
};
