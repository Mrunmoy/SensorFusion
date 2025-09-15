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

constexpr float DEG2RAD = 0.017453292519943295f; // pi/180

// Expect these in your project; if not, you can keep these locals.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

static inline float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}
static inline float wrap_pi(float a)
{
	while (a > M_PI)
		a -= 2.0f * (float)M_PI;
	while (a < -M_PI)
		a += 2.0f * (float)M_PI;
	return a;
}

static inline float rad2deg(float r)
{
	return r * 57.29577951308232f;
}
static inline float safe_atan2(float y, float x)
{
	return (x == 0.f && y == 0.f) ? 0.f : atan2f(y, x);
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

	static uint32_t dbg_counter = 0;
	for (;;)
	{
		if (ctx->mpu->sample(s.ax, s.ay, s.az, s.gx, s.gy, s.gz))
		{
			++ok;
			s.t_us = esp_timer_get_time();
			ctx->reg->write<SFKey::IMU_RAW>(s);

			if ((++dbg_counter % 50u) == 0u)
			{ // ~4 logs/sec at 200 Hz
				int16_t rx, ry, rz;
				if (ctx->mpu->readRawAccel(rx, ry, rz))
				{
					ESP_LOGI("ImuTask", "RAW accel: [%d %d %d] counts", rx, ry,
							 rz);
				}
			}
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

static inline float wrap180(float d)
{
	d = fmodf(d + 180.0f, 360.0f);
	if (d < 0)
		d += 360.0f;
	return d - 180.0f;
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

	// ---- choose algorithm and config ---------------------------------
	const fusion::Algo *algo = fusion::complementary_algo();
	fusion::Config cfg{};  // defaults
	cfg.yaw_beta = 0.010f; // gentle mag blend until we calibrate mag

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
	// ------------------------------------------------------------------

	// --- quick “still” calibration (1.5 s): gyro bias (deg/s) + accel scale
	// fudge
	float gyro_bias_deg[3] = {
		0.f, 0.f, 0.f}; // leave in deg/s because filter expects deg/s input
	float accel_fudge =
		1.0f; // multiply ax/ay/az (m/s²) by this before passing to filter

	auto quick_cal = [&](float seconds = 1.5f)
	{
		const int target_samples =
			(int)(seconds * ctx->hz); // e.g. ~300 at 200 Hz
		int n = 0;
		double sum_gx = 0, sum_gy = 0, sum_gz = 0;
		double sum_anorm = 0;

		int64_t t_end = esp_timer_get_time() + (int64_t)(seconds * 1e6);
		IMURaw s{};
		while (esp_timer_get_time() < t_end && n < target_samples * 2)
		{
			if (ctx->reg->read<SFKey::IMU_RAW>(s))
			{
				sum_gx += s.gx; // deg/s as produced now
				sum_gy += s.gy;
				sum_gz += s.gz;
				const double an =
					std::sqrt((double)s.ax * s.ax + (double)s.ay * s.ay +
							  (double)s.az * s.az); // m/s²
				sum_anorm += an;
				++n;
			}
			// yield a bit; IMU producer is 200 Hz
			vTaskDelay(pdMS_TO_TICKS(1));
		}

		if (n > 10)
		{
			gyro_bias_deg[0] = (float)(sum_gx / n);
			gyro_bias_deg[1] = (float)(sum_gy / n);
			gyro_bias_deg[2] = (float)(sum_gz / n);
			const double mean_anorm = sum_anorm / n;
			if (mean_anorm > 1e-3)
			{
				accel_fudge = (float)(9.80665 / mean_anorm); // bring |a| to 1 g
			}
			ESP_LOGI(TAG,
					 "QuickCal: n=%d gyro_bias(deg/s)=[%.3f %.3f %.3f], "
					 "|a|=%.2f -> fudge=%.3f",
					 n, gyro_bias_deg[0], gyro_bias_deg[1], gyro_bias_deg[2],
					 (float)(mean_anorm), accel_fudge);
		}
		else
		{
			ESP_LOGW(TAG, "QuickCal: not enough samples (%d), skipping", n);
		}
	};
	quick_cal(1.5f);

	IMURaw imu{};
	MagRaw mag{};
	BaroRaw baro{};
	FusionState st{};
	RateLogger rl(250000); // log every 250 ms
	uint32_t step = 0, published = 0;

	for (;;)
	{
		++step;

		if (ctx->reg->read<SFKey::IMU_RAW>(imu))
		{
			(void)ctx->reg->read<SFKey::MAG_RAW>(mag);
			(void)ctx->reg->read<SFKey::BARO_RAW>(baro);

			// Apply quick-cal corrections in-line (no filter changes needed)
			IMURaw imu_corr = imu;
			imu_corr.gx -= gyro_bias_deg[0]; // still deg/s here; filter will
											 // convert to rad/s
			imu_corr.gy -= gyro_bias_deg[1];
			imu_corr.gz -= gyro_bias_deg[2];
			imu_corr.ax *= accel_fudge; // m/s²
			imu_corr.ay *= accel_fudge;
			imu_corr.az *= accel_fudge;

			fusion::Inputs in{};
			in.imu = imu_corr; // corrected
			in.mag = mag;
			in.baro = baro;
			in.dt =
				pacer.period_us *
				1e-6f; // fallback dt; algo will prefer IMU timestamp if sane

			fusion::Outputs out{};
			algo->step(storage.ptr(), &in, &out);

			st = out.state; // keep for logging/output
			ctx->reg->write<SFKey::FUSION_STATE>(st);

			// --- DEBUG: reference angles vs fused
			// ---------------------------------
			{
				// Note: imu_corr.* already includes scale fudge
				const float ax = imu_corr.ax, ay = imu_corr.ay,
							az = imu_corr.az;
				const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

				// reference tilt (accel only)
				// const float roll_acc = atan2f(ay, az);
				// const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az *
				// az)); reference tilt (accel only), assume X forward, Y left,
				// Z up
				float roll_acc = atan2f(ay, sqrtf(ax * ax + az * az));
				float pitch_acc = atan2f(-ax, az);

				// when computing/printing acc-only roll/pitch:
				roll_acc = wrap180(roll_acc);
				pitch_acc = wrap180(pitch_acc);

				// Tilt-compensated magnetic yaw (using fused roll/pitch)
				float yaw_mag = st.yaw;
				{
					const float cr = cosf(st.roll), sr = sinf(st.roll);
					const float cp = cosf(st.pitch), sp = sinf(st.pitch);
					const float mx = mag.mx, my = mag.my, mz = mag.mz;
					const float mhx = mx * cp + my * sr * sp + mz * cr * sp;
					const float mhy = my * cr - mz * sr;
					yaw_mag = safe_atan2(-mhy, mhx); // [-pi, pi]
				}

				if (rl.ready(esp_timer_get_time()))
				{
					uint64_t n;
					double avg;
					int64_t minv, maxv;
					stats.snapshot(n, avg, minv, maxv);
					const double eff_hz = (avg > 0.0) ? (1e6 / avg) : 0.0;

					ESP_LOGI(TAG,
							 "RPY fused= [%.1f, %.1f, %.1f] deg | "
							 "accRP= [%.1f, %.1f] deg | magY= %.1f deg | "
							 "|a|=%.2f (expect ~9.8 if m/s^2, ~1.0 if g) | "
							 "Δr=%.1f Δp=%.1f Δy=%.1f",
							 rad2deg(st.roll), rad2deg(st.pitch),
							 rad2deg(st.yaw), rad2deg(roll_acc),
							 rad2deg(pitch_acc), rad2deg(yaw_mag), acc_norm,
							 rad2deg(st.roll - roll_acc),
							 rad2deg(st.pitch - pitch_acc),
							 rad2deg(st.yaw - yaw_mag));

					(void)eff_hz;
					(void)n;
					(void)avg;
					(void)minv;
					(void)maxv;

					stats.reset();
				}
			}
			// ----------------------------------------------------------------------

			++published;
		}

		bool rst = false;
		if (ctx->reg->consume<SFKey::CMD_RESET>(rst) && rst)
		{
			ESP_LOGW(TAG, "CMD_RESET consumed — resetting filter state");
			algo->reset(storage.ptr());
			// keep quick-cal results; if you want a fresh quick-cal, call
			// quick_cal() here
		}

		if (rl.ready(esp_timer_get_time()))
		{
			uint64_t n;
			double avg;
			int64_t minv, maxv;
			stats.snapshot(n, avg, minv, maxv);

			const double eff_hz = (avg > 0.0) ? (1e6 / avg) : 0.0;

			ESP_LOGI(TAG,
					 "r=%.1f° p=%.1f° y=%.1f° | steps=%u published=%u "
					 "last_ts=%llu us | rate=%.1f Hz period_us avg=%.1f "
					 "min=%lld max=%lld",
					 rad2deg(st.roll), rad2deg(st.pitch), rad2deg(st.yaw), step,
					 published, (unsigned long long)st.t_us, eff_hz, avg,
					 (long long)minv, (long long)maxv);

			step = published = 0;
			stats.reset();
		}

		stats.tick();
		sleep_until(pacer); // precise pacing
	}
}
