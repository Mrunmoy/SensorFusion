#pragma once

#include "freertos/FreeRTOS.h"
#include <algorithm>
#include <cmath>

// Convert frequency in Hz to a FreeRTOS period in ticks.
// Clamps to >= 1 tick so it never becomes 0.
inline TickType_t hz_to_ticks(float hz)
{
	if (hz <= 0.0f)
		return 1;
	const float ticks = static_cast<float>(configTICK_RATE_HZ) / hz;
	return static_cast<TickType_t>(std::max(1.0f, std::round(ticks)));
}
