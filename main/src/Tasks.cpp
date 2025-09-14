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

namespace
{
constexpr uint64_t LOG_INTERVAL_US = 1'000'000; // 1 Hz logs

struct RateLogger
{
	uint64_t last_us{0};
	explicit RateLogger(uint64_t interval_us)
		: last_us(0), interval_us_(interval_us)
	{
	}
	bool ready(uint64_t now)
	{
		if (last_us == 0 || now - last_us >= interval_us_)
		{
			last_us = now;
			return true;
		}
		return false;
	}

  private:
	uint64_t interval_us_;
};
} // namespace

/** IMU poll task: samples at ctx->hz and publishes IMURaw. */
extern "C" void ImuTask(void *pv)
{
	static const char *TAG = "ImuTask";
	auto *ctx = static_cast<ImuCtx *>(pv);

	const TickType_t period = hz_to_ticks(ctx->hz);
	TickType_t next = xTaskGetTickCount();
	ESP_LOGI(TAG, "start @ %.1f Hz (period=%u ticks)", ctx->hz,
			 (unsigned)period);

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
			ESP_LOGI(TAG,
					 "ok=%u fail=%u last: a[%.2f %.2f %.2f] g[%.3f %.3f %.3f]",
					 ok, fail, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
			ok = fail = 0;
		}

		vTaskDelayUntil(&next, period);
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

	const TickType_t period = hz_to_ticks(ctx->hz);
	TickType_t next = xTaskGetTickCount();
	ESP_LOGI(TAG, "start @ %.1f Hz (period=%u ticks)", ctx->hz,
			 (unsigned)period);

	IMURaw imu{};
	MagRaw mag{};
	BaroRaw baro{};
	FusionState st{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t step = 0, published = 0;

	for (;;)
	{
		++step;

		// Proceed only if we have a fresh IMU sample.
		if (ctx->reg->read<SFKey::IMU_RAW>(imu))
		{
			(void)ctx->reg->read<SFKey::MAG_RAW>(mag);	 // best-effort
			(void)ctx->reg->read<SFKey::BARO_RAW>(baro); // best-effort

			// TODO: run your filter and fill st.{qw,qx,qy,qz,roll,pitch,yaw}
			st = FusionState{};
			st.t_us = imu.t_us;

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
			ESP_LOGI(TAG, "steps=%u published=%u last_ts=%llu us", step,
					 published, static_cast<unsigned long long>(st.t_us));
			step = published = 0;
		}

		vTaskDelayUntil(&next, period);
	}
}
