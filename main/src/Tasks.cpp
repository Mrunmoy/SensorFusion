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

#include "LoopStats.hpp"
#include "MicroPacer.hpp"
#include "RateLogger.hpp"

#include "fusion/Complementary.hpp"
#include "fusion/FusionAlgo.hpp"
#include "WebSocketServer.hpp"

#include <math.h>
#include <stdio.h>

namespace
{
constexpr uint64_t LOG_INTERVAL_US = 1'000'000; // 1 second

constexpr float DEG2RAD = 0.017453292519943295f; // pi/180

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}
static inline float wrap_pi(float a) // wrap radians to [-pi, pi]
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

	// µs pacer for precise rates (e.g., 200 Hz)
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

			// Occasionally print raw counts to sanity-check scaling
			if ((++dbg_counter % 50u) == 0u) // ~4 logs/s at 200 Hz
			{
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

		stats.tick();
		sleep_until(pacer);
	}
}

// ----------------- Magnetometer -----------------

// File-scope extrema for running hard-iron estimate
static float mx_min = 1e9f, my_min = 1e9f, mz_min = 1e9f;
static float mx_max = -1e9f, my_max = -1e9f, mz_max = -1e9f;

extern "C" void MagTask(void *pv)
{
	static const char *TAG = "MagTask";
	auto *ctx = static_cast<MagCtx *>(pv);

	// Soft-iron matrix (identity until you fit it offline)
	static const float mag_soft[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	// Hard-iron bias (estimated from a sweep)
	static float bx = 0.f, by = 0.f, bz = 0.f;
	static bool calib_valid = false;

	// Require at least this span on EACH axis before enabling calibration
	static constexpr float SPAN_THRESH_UT = 8.0f; // µT; tweak 5–15 as needed
	static constexpr float LPF = 0.02f;			  // slow bias update when valid

	// --- Rate bump safeguard: ensure >= 25 Hz unless user already set higher
	// ---
	float hz = ctx->hz;
	if (hz < 25.f)
	{
		ESP_LOGW(TAG,
				 "ctx->hz=%.1f too low; bumping MagTask to 25.0 Hz for better "
				 "yaw tracking",
				 hz);
		hz = 25.f;
	}
	const TickType_t period = hz_to_ticks(hz);
	TickType_t next = xTaskGetTickCount();
	ESP_LOGI(TAG, "start @ %.1f Hz (period=%u ticks)", hz, (unsigned)period);

	// NOTE: If your QMC5883L driver exposes an ODR setter, consider enabling
	// ~50 Hz here. e.g., ctx->mag->setOutputDataRate(QMC5883LDriver::ODR_50HZ);

	MagRaw m{};
	RateLogger rl(LOG_INTERVAL_US);
	uint32_t ok = 0, fail = 0;

	for (;;)
	{
		if (ctx->mag->sample(m.mx, m.my, m.mz))
		{
			++ok;
			m.t_us = esp_timer_get_time();

			// Track extrema for rough bias (hard-iron)
			mx_min = fminf(mx_min, m.mx);
			mx_max = fmaxf(mx_max, m.mx);
			my_min = fminf(my_min, m.my);
			my_max = fmaxf(my_max, m.my);
			mz_min = fminf(mz_min, m.mz);
			mz_max = fmaxf(mz_max, m.mz);

			const float sx = mx_max - mx_min;
			const float sy = my_max - my_min;
			const float sz = mz_max - mz_min;

			// Enable calibration only after we’ve seen enough span on all axes
			if (!calib_valid && sx > SPAN_THRESH_UT && sy > SPAN_THRESH_UT &&
				sz > SPAN_THRESH_UT)
			{
				bx = 0.5f * (mx_max + mx_min);
				by = 0.5f * (my_max + my_min);
				bz = 0.5f * (mz_max + mz_min);
				calib_valid = true;
				ESP_LOGW(TAG,
						 "calib VALID: bias=[%.2f %.2f %.2f] span=[%.2f %.2f "
						 "%.2f] (µT)",
						 bx, by, bz, sx, sy, sz);

				// TODO (optional): persist bias/soft matrix to NVS and reload
				// on boot.
			}
			else if (calib_valid)
			{
				// Slowly refine bias as the sweep improves
				const float bx_new = 0.5f * (mx_max + mx_min);
				const float by_new = 0.5f * (my_max + my_min);
				const float bz_new = 0.5f * (mz_max + mz_min);
				bx = (1.0f - LPF) * bx + LPF * bx_new;
				by = (1.0f - LPF) * by + LPF * by_new;
				bz = (1.0f - LPF) * bz + LPF * bz_new;
			}

			// Apply calibration (ONLY if valid), else pass raw through
			float mx = m.mx, my = m.my, mz = m.mz;
			if (calib_valid)
			{
				mx -= bx;
				my -= by;
				mz -= bz; // hard-iron
				const float mx_c = mag_soft[0][0] * mx + mag_soft[0][1] * my +
								   mag_soft[0][2] * mz;
				const float my_c = mag_soft[1][0] * mx + mag_soft[1][1] * my +
								   mag_soft[1][2] * mz;
				const float mz_c = mag_soft[2][0] * mx + mag_soft[2][1] * my +
								   mag_soft[2][2] * mz;
				m.mx = mx_c;
				m.my = my_c;
				m.mz = mz_c;
			}
			// else keep m.mx/my/mz as raw to preserve proper magnitude

			ctx->reg->write<SFKey::MAG_RAW>(m);
		}
		else
		{
			++fail;
		}

		if (rl.ready(esp_timer_get_time()))
		{
			const float sx = mx_max - mx_min;
			const float sy = my_max - my_min;
			const float sz = mz_max - mz_min;
			ESP_LOGI(TAG,
					 "ok=%u fail=%u last: m[%.2f %.2f %.2f] uT | calib=%s "
					 "bias=[%.2f %.2f %.2f] span=[%.2f %.2f %.2f]",
					 ok, fail, m.mx, m.my, m.mz, calib_valid ? "ON" : "OFF", bx,
					 by, bz, sx, sy, sz);
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

	// ---- choose algorithm and config ---------------------------------
	const fusion::Algo *algo = fusion::complementary_algo();
	fusion::Config cfg{};  // defaults
	cfg.yaw_beta = 0.035f; // slightly stronger mag blend for faster convergence

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

	// --- Wait for gyro to stabilize after power-up (MPU6050 datasheet recommends this)
	ESP_LOGI(TAG, "Waiting 3 seconds for gyroscope to stabilize...");
	vTaskDelay(pdMS_TO_TICKS(3000));  // 3 second startup delay
	ESP_LOGI(TAG, "Gyro stabilization complete, starting calibration");

	// --- quick "still" calibration (1.5 s): gyro bias (deg/s) + accel scale
	// fudge
	float gyro_bias_deg[3] = {
		0.f, 0.f, 0.f};		  // keep in deg/s (filter expects deg/s input)
	float accel_fudge = 1.0f; // multiply ax/ay/az (g) by this before filter

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
							  (double)s.az * s.az); // g (gravity units)
				sum_anorm += an;
				++n;
			}
			vTaskDelay(pdMS_TO_TICKS(1)); // IMU producer is ~200 Hz
		}

		if (n > 10)
		{
			gyro_bias_deg[0] = (float)(sum_gx / n);
			gyro_bias_deg[1] = (float)(sum_gy / n);
			gyro_bias_deg[2] = (float)(sum_gz / n);
			const double mean_anorm = sum_anorm / n;
			if (mean_anorm > 1e-3)
				accel_fudge =
					(float)(1.0 / mean_anorm); // bring |a| to ~1 g

			ESP_LOGI(TAG,
					 "QuickCal: n=%d gyro_bias(deg/s)=[%.3f %.3f %.3f], "
					 "|a|=%.2f -> fudge=%.3f",
					 n, gyro_bias_deg[0], gyro_bias_deg[1], gyro_bias_deg[2],
					 (float)mean_anorm, accel_fudge);
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
			imu_corr.gx -=
				gyro_bias_deg[0]; // deg/s (filter will convert to rad/s)
			imu_corr.gy -= gyro_bias_deg[1];
			imu_corr.gz -= gyro_bias_deg[2];
			imu_corr.ax *= accel_fudge; // m/s²
			imu_corr.ay *= accel_fudge;
			imu_corr.az *= accel_fudge;

			// ---------- Gate magnetometer usage in motion ----------
			// Heuristics: use mag only when |a| ~ 1g, gyro not too high, field
			// magnitude plausible.
			const float anorm =
				sqrtf(imu_corr.ax * imu_corr.ax + imu_corr.ay * imu_corr.ay +
					  imu_corr.az * imu_corr.az);
			const float gsum = fabsf(imu_corr.gx) + fabsf(imu_corr.gy) +
							   fabsf(imu_corr.gz); // deg/s sum
			const float mnorm =
				sqrtf(mag.mx * mag.mx + mag.my * mag.my + mag.mz * mag.mz);
			const bool accel_ok = (anorm > 0.8f && anorm < 1.2f); // g (gravity units)
			const bool gyro_ok = (gsum < 120.0f);				   // deg/s
			const bool field_ok =
				(mnorm > 10.0f && mnorm < 100.0f); // µT plausible
			const bool mag_ok = accel_ok && gyro_ok && field_ok;

			MagRaw mag_use = mag;
			if (!mag_ok)
			{
				// Set to zero so filter knows mag is unavailable (have_mag check)
				mag_use.mx = 0.0f;
				mag_use.my = 0.0f;
				mag_use.mz = 0.0f;
			}

			fusion::Inputs in{};
			in.imu = imu_corr; // corrected IMU
			in.mag = mag_use;  // gated mag
			in.baro = baro;
			in.dt = pacer.period_us * 1e-6f; // fallback dt

			fusion::Outputs out{};
			algo->step(storage.ptr(), &in, &out);

			st = out.state;
			ctx->reg->write<SFKey::FUSION_STATE>(st);

			// ---- Broadcast to WebSocket (20 Hz update rate) ----
			{
				static uint32_t ws_counter = 0;
				if ((++ws_counter % 10) == 0) // Every 10th fusion step = 20 Hz
				{
					// Check for valid values before broadcasting (avoid NaN)
					if (std::isfinite(st.roll) && std::isfinite(st.pitch) && std::isfinite(st.yaw))
					{
						char json[256];
						snprintf(json, sizeof(json),
							"{\"imu\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
							"\"environment\":{\"temperature\":%.1f,\"humidity\":0.0,\"air_quality\":50.0}}",
							rad2deg(st.roll), rad2deg(st.pitch), rad2deg(st.yaw),
							baro.temp_c);
						WebSocketServer::broadcast(json);
					}
				}
			}

			// ---- DEBUG reference angles vs fused --------------------------
			{
				const float ax = imu_corr.ax, ay = imu_corr.ay,
							az = imu_corr.az;
				const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

				// Accel-only tilt (radians), wrap to [-pi, pi]
				float roll_acc = atan2f(ay, sqrtf(ax * ax + az * az));
				float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
				roll_acc = wrap_pi(roll_acc);
				pitch_acc = wrap_pi(pitch_acc);

				// Tilt-compensated magnetic yaw using fused roll/pitch
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
							 "|a|=%.2fg (expect ~1.0) | "
							 "Δr=%.1f Δp=%.1f Δy=%.1f | mag_ok=%s mnorm=%.1f",
							 rad2deg(st.roll), rad2deg(st.pitch),
							 rad2deg(st.yaw), rad2deg(roll_acc),
							 rad2deg(pitch_acc), rad2deg(yaw_mag), acc_norm,
							 rad2deg(st.roll - roll_acc),
							 rad2deg(st.pitch - pitch_acc),
							 rad2deg(st.yaw - yaw_mag), mag_ok ? "yes" : "no",
							 mnorm);

					(void)eff_hz;
					(void)n;
					(void)avg;
					(void)minv;
					(void)maxv;
					stats.reset();
				}
			}
			// ----------------------------------------------------------------

			++published;
		}

		bool rst = false;
		if (ctx->reg->consume<SFKey::CMD_RESET>(rst) && rst)
		{
			ESP_LOGW(TAG, "CMD_RESET consumed — resetting filter state");
			algo->reset(storage.ptr());
			// If you want a fresh quick-cal after reset, call quick_cal() here
		}

		// periodic loop stats
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
