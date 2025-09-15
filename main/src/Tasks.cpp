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

#include "fusion/Complementary.hpp"
#include "fusion/FusionAlgo.hpp"

namespace
{
constexpr uint64_t LOG_INTERVAL_US = 1'000'000; // 5 seconds
static inline float rad2deg(float r)
{
	return r * 57.29577951308232f;
}

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
 *  consumes CMD_RESET. Keeps µs pacer for precise timing. */
extern "C" void FusionTask(void *pv)
{
	static const char *TAG = "FusionTask";
	auto *ctx = static_cast<FusionCtx *>(pv);

	LoopStats stats;
	auto pacer = make_pacer(ctx->hz);
	ESP_LOGI(TAG, "start @ %.1f Hz (µs pacer, period=%d us)", ctx->hz,
			 pacer.period_us);

	// ---- pluggable algorithm wiring (minimal) -----------------------
	const fusion::Algo *algo = fusion::complementary_algo(); // choose algo
	fusion::Config cfg{}; // defaults (tweak later)

	// fixed, aligned state storage (no heap, no RTTI)
	static constexpr size_t MAX_STATE = 256;
	static constexpr size_t MAX_ALIGN = 16;
	fusion::Aligned<MAX_STATE, MAX_ALIGN> storage;

	if (algo->state_size > MAX_STATE || algo->state_align > MAX_ALIGN)
	{
		ESP_LOGE(TAG, "algo state too large/aligned for buffer");
		vTaskDelete(nullptr);
		return;
	}
	algo->init(storage.ptr(), &cfg);
	// ----------------------------------------------------------------

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

			fusion::Inputs in{};
			in.imu = imu;
			in.mag = mag;
			in.baro = baro;
			in.dt = pacer.period_us * 1e-6f; // use your pacer period

			fusion::Outputs out{};
			algo->step(storage.ptr(), &in, &out);

			st = out.state; // keep your logging shape below
			ctx->reg->write<SFKey::FUSION_STATE>(st);
			++published;
		}

		bool rst = false;
		if (ctx->reg->consume<SFKey::CMD_RESET>(rst) && rst)
		{
			ESP_LOGW(TAG, "CMD_RESET consumed — resetting filter state");
			algo->reset(storage.ptr());
		}

		if (rl.ready(esp_timer_get_time()))
		{
			uint64_t n;
			double avg;
			int64_t minv, maxv;
			stats.snapshot(n, avg, minv, maxv);

			// effective rate from measured periods
			const double eff_hz = (avg > 0.0) ? (1e6 / avg) : 0.0;

			ESP_LOGI(TAG,
					 "r=%.1f° p=%.1f° y=%.1f° | steps=%u published=%u "
					 "last_ts=%llu us | "
					 "rate=%.1f Hz period_us avg=%.1f min=%lld max=%lld",
					 rad2deg(st.roll), rad2deg(st.pitch), rad2deg(st.yaw), step,
					 published, (unsigned long long)st.t_us, eff_hz, avg,
					 (long long)minv, (long long)maxv);

			step = published = 0;
			stats.reset();
		}

		stats.tick();
		sleep_until(pacer); // <— still using your µs pacer
	}
}
