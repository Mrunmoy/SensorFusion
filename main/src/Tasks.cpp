#include "Tasks.hpp"
#include "SFRegistry.hpp"
#include "TaskRate.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Your drivers are in main/inc
#include "BMP180Driver.hpp"
#include "MPU6050Driver.hpp"
#include "QMC5883LDriver.hpp"

#include "MicroPacer.hpp" // new header
#include "RateLogger.hpp" // new header

#include "LoopStats.hpp"

namespace
{
constexpr uint64_t LOG_INTERVAL_US = 1'000'000; // 5 seconds
} // namespace

/** IMU poll task: samples at ctx->hz and publishes IMURaw. */
extern "C" void ImuTask(void *pv)
{
	static const char *TAG = "ImuTask";
	auto *ctx = static_cast<ImuCtx *>(pv);
	LoopStats stats;
	// Switch to microsecond pacer for precise 200 Hz (or whatever ctx->hz is)
	auto pacer = make_pacer(ctx->hz);
	ESP_LOGI(TAG, "start @ %.1f Hz (µs pacer, period=%d us)", ctx->hz,
			 pacer.period_us);

	IMURaw s{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t ok = 0, fail = 0;

	for (;;)
	{
		if (ctx->mpu->sample(s.ax, s.ay, s.az, s.gx, s.gy, s.gz))
		{
			++ok;
			s.t_us = esp_timer_get_time();
			ctx->reg->write<SFKey::IMU_RAW>(s);
		}
		else
		{
			++fail;
		}

		if (rl.ready(esp_timer_get_time()))
		{
			uint64_t n;
			double avg;
			int64_t minv, maxv;
			stats.snapshot(n, avg, minv, maxv);
			ESP_LOGI(TAG,
					 "ok=%u fail=%u last: a[%.2f %.2f %.2f] g[%.3f %.3f %.3f]  "
					 "period_us avg=%.1f min=%lld max=%lld",
					 ok, fail, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, avg,
					 (long long)minv, (long long)maxv);
			ok = fail = 0;
			stats.reset();
		}

		stats.tick(); // track iteration period
		sleep_until(pacer);
	}
}

/** Magnetometer poll task: samples at ctx->hz and publishes MagRaw. */
extern "C" void MagTask(void *pv)
{
	static const char *TAG = "MagTask";
	auto *ctx = static_cast<MagCtx *>(pv);

	const TickType_t period = hz_to_ticks(ctx->hz);
	TickType_t next = xTaskGetTickCount();
	ESP_LOGI(TAG, "start @ %.1f Hz (period=%u ticks)", ctx->hz,
			 (unsigned)period);

	MagRaw m{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t ok = 0, fail = 0;

	for (;;)
	{
		if (ctx->mag->sample(m.mx, m.my, m.mz))
		{
			++ok;
			m.t_us = esp_timer_get_time();
			ctx->reg->write<SFKey::MAG_RAW>(m);
		}
		else
		{
			++fail;
		}

		if (rl.ready(esp_timer_get_time()))
		{
			ESP_LOGI(TAG, "ok=%u fail=%u last: m[%.2f %.2f %.2f] uT", ok, fail,
					 m.mx, m.my, m.mz);
			ok = fail = 0;
		}

		vTaskDelayUntil(&next, period);
	}
}

/** Barometer poll task: samples at ctx->hz and publishes BaroRaw. */
extern "C" void BaroTask(void *pv)
{
	static const char *TAG = "BaroTask";
	auto *ctx = static_cast<BaroCtx *>(pv);

	const TickType_t period = hz_to_ticks(ctx->hz);
	TickType_t next = xTaskGetTickCount();
	ESP_LOGI(TAG, "start @ %.1f Hz (period=%u ticks)", ctx->hz,
			 (unsigned)period);

	BaroRaw b{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t ok = 0, fail = 0;

	for (;;)
	{
		if (ctx->baro->sample(b.temp_c, b.pressure_pa, b.alt_m))
		{
			++ok;
			b.t_us = esp_timer_get_time();
			ctx->reg->write<SFKey::BARO_RAW>(b);
		}
		else
		{
			++fail;
		}

		if (rl.ready(esp_timer_get_time()))
		{
			ESP_LOGI(TAG, "ok=%u fail=%u last: T=%.2fC P=%.0fPa Alt=%.1fm", ok,
					 fail, b.temp_c, b.pressure_pa, b.alt_m);
			ok = fail = 0;
		}

		vTaskDelayUntil(&next, period);
	}
}

/** Fusion task: reads latest samples, runs fusion, publishes FUSION_STATE;
 * consumes CMD_RESET. */
extern "C" void FusionTask(void *pv)
{
	static const char *TAG = "FusionTask";
	auto *ctx = static_cast<FusionCtx *>(pv);
	LoopStats stats;
	auto pacer = make_pacer(ctx->hz);
	ESP_LOGI(TAG, "start @ %.1f Hz (µs pacer, period=%d us)", ctx->hz,
			 pacer.period_us);

	IMURaw imu{};
	MagRaw mag{};
	BaroRaw baro{};
	FusionState st{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t step = 0, published = 0;

	for (;;)
	{
		++step;

		if (ctx->reg->read<SFKey::IMU_RAW>(imu))
		{
			(void)ctx->reg->read<SFKey::MAG_RAW>(mag);
			(void)ctx->reg->read<SFKey::BARO_RAW>(baro);

			st = FusionState{};
			st.t_us = imu.t_us;
			// TODO: fill quaternion + r/p/y
			ctx->reg->write<SFKey::FUSION_STATE>(st);
			++published;
		}

		bool rst = false;
		if (ctx->reg->consume<SFKey::CMD_RESET>(rst) && rst)
		{
			ESP_LOGW(TAG, "CMD_RESET consumed — resetting filter state");
			// TODO: reset/rehoming logic
		}

		if (rl.ready(esp_timer_get_time()))
		{
			uint64_t n;
			double avg;
			int64_t minv, maxv;
			stats.snapshot(n, avg, minv, maxv);
			ESP_LOGI(TAG,
					 "steps=%u published=%u last_ts=%llu us  period_us "
					 "avg=%.1f min=%lld max=%lld",
					 step, published, (unsigned long long)st.t_us, avg,
					 (long long)minv, (long long)maxv);
			step = published = 0;
			stats.reset();
		}

		stats.tick();
		sleep_until(pacer);
	}
}
