#include "fusion/Complementary.hpp"

#include <cmath>
#include <cstdint>

// ---- replace the "Auto gyro-bias estimation" block with this:
#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char *TAGC = "Complementary";
#else
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

// Gates: how still we require during auto-bias
static constexpr float ACC_GATE_G = 0.15f;		   // | |a|-1g | < 0.15 g
static constexpr float GYRO_GATE_DPS = 10.0f;	   // |gyro| < 10 dps
static constexpr float GYRO_GATE_RAD = 0.1745329f; // rad/s

namespace fusion
{

static inline float deg2rad(float d)
{
	return d * 0.017453292519943295f;
}
static constexpr float PI = 3.14159265358979323846f;

struct State
{
	bool inited{false};

	// attitude (radians)
	float roll{0}, pitch{0}, yaw{0};

	// gyro bias (rad/s)
	float gbx{0}, gby{0}, gbz{0};

	// tunables
	float accel_alpha{0.02f}; // accel tilt blend
	float yaw_beta{0.02f};	  // mag-yaw blend
	float decl{0.0f};		  // magnetic declination (rad)

	// timing
	int64_t last_us{0};

	// startup auto-bias
	bool bias_done{false};
	int64_t bias_start_us{0};
	double sum_gx{0}, sum_gy{0}, sum_gz{0};
	uint32_t bias_count{0};
};

// Try to grab gyro bias automatically for this many seconds at startup
static constexpr float AUTO_BIAS_SEC = 0.75f;

static void init_fn(void *s, const Config *cfg)
{
	auto *st = static_cast<State *>(s);
	*st = State{};
	if (cfg)
	{
		st->accel_alpha = cfg->accel_alpha;
		st->yaw_beta = cfg->yaw_beta;
		st->decl = (cfg->mag_decl_deg) * 0.017453292519943295f;
		st->gbx = cfg->gyro_bias[0];
		st->gby = cfg->gyro_bias[1];
		st->gbz = cfg->gyro_bias[2];
	}
}

static void reset_fn(void *s)
{
	auto *st = static_cast<State *>(s);
	float aa = st->accel_alpha, yb = st->yaw_beta, d = st->decl;
	float bx = st->gbx, by = st->gby, bz = st->gbz;
	*st = State{};
	st->accel_alpha = aa;
	st->yaw_beta = yb;
	st->decl = d;
	st->gbx = bx;
	st->gby = by;
	st->gbz = bz;
}

static inline bool have_mag(const Inputs *in)
{
	// Treat "all zeros" as unavailable; okay for QMC5883L with valid field
	return (in->mag.mx != 0.f || in->mag.my != 0.f || in->mag.mz != 0.f);
}

static inline float tiltcomp_yaw(float roll, float pitch, float mx, float my,
								 float mz)
{
	const float cr = std::cos(roll), sr = std::sin(roll);
	const float cp = std::cos(pitch), sp = std::sin(pitch);
	const float mhx = mx * cp + my * sr * sp + mz * cr * sp;
	const float mhy = my * cr - mz * sr;
	return std::atan2(-mhy, mhx); // [-pi, pi]
}

static void step_fn(void *s, const Inputs *in, Outputs *out)
{
	auto *st = static_cast<State *>(s);
	if (!in || !out)
		return;

	// dt: prefer producer timestamps if sane; otherwise, use provided dt
	float dt = in->dt;
	if (in->imu.t_us > 0)
	{
		if (st->last_us == 0)
			st->last_us = in->imu.t_us;
		float dt_ts = (in->imu.t_us - st->last_us) * 1e-6f;
		if (dt_ts > 0.f && dt_ts < 0.2f)
			dt = dt_ts;
		st->last_us = in->imu.t_us;
	}
	if (dt <= 0.f)
		dt = 0.005f;

	// Expected IMURaw units coming in:
	//   gx/gy/gz in deg/s; ax/ay/az in g (scale cancels in tilt)
	const float gxr = deg2rad(in->imu.gx);
	const float gyr = deg2rad(in->imu.gy);
	const float gzr = deg2rad(in->imu.gz);
	const float ax = in->imu.ax;
	const float ay = in->imu.ay;
	const float az = in->imu.az;

	// One-time attitude init from accel (sign-agnostic denominators)
	if (!st->inited)
	{
		st->roll = std::atan2(ay, std::sqrt(ax * ax + az * az));
		st->pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

		// If we already have a magnetometer sample, start yaw near truth
		if (have_mag(in))
		{
			st->yaw = tiltcomp_yaw(st->roll, st->pitch, in->mag.mx, in->mag.my,
								   in->mag.mz) +
					  st->decl;
			// Wrap yaw to [-pi, pi]
			if (st->yaw > PI)
				st->yaw -= 2 * PI;
			if (st->yaw < -PI)
				st->yaw += 2 * PI;
		}
		else
		{
			st->yaw = 0.f;
		}

		// Start auto-bias window
		st->bias_start_us = (in->imu.t_us > 0) ? in->imu.t_us : 0;
		st->bias_done = (AUTO_BIAS_SEC <= 0.f); // skip if disabled
		st->sum_gx = st->sum_gy = st->sum_gz = 0.0;
		st->bias_count = 0;

		st->inited = true;
	}

	// Auto gyro-bias estimation for the first ~AUTO_BIAS_SEC
	if (!st->bias_done)
	{
		// Use raw units for gates (deg/s) and 'g'
		const float gnorm = std::sqrt(ax * ax + ay * ay + az * az);
		const bool acc_ok = std::fabs(gnorm - 1.0f) < ACC_GATE_G;
		const bool gyro_ok = std::fabs(in->imu.gx) < GYRO_GATE_DPS &&
							 std::fabs(in->imu.gy) < GYRO_GATE_DPS &&
							 std::fabs(in->imu.gz) < GYRO_GATE_DPS;

		if (acc_ok && gyro_ok)
		{
			st->sum_gx += static_cast<double>(gxr);
			st->sum_gy += static_cast<double>(gyr);
			st->sum_gz += static_cast<double>(gzr);
			st->bias_count++;
		}

		const bool time_ok =
			(in->imu.t_us > 0 && st->bias_start_us > 0)
				? (in->imu.t_us - st->bias_start_us) >=
					  static_cast<int64_t>(AUTO_BIAS_SEC * 1e6f)
				: (st->bias_count >=
				   static_cast<uint32_t>(AUTO_BIAS_SEC * 200.0f));

		// Require at least a few dozen "still" samples before locking bias
		const bool samples_ok = st->bias_count >= 40;

		if (time_ok && samples_ok)
		{
			const double invN = 1.0 / static_cast<double>(st->bias_count);
			st->gbx = static_cast<float>(st->sum_gx * invN);
			st->gby = static_cast<float>(st->sum_gy * invN);
			st->gbz = static_cast<float>(st->sum_gz * invN);
			st->bias_done = true;
			ESP_LOGI(TAGC,
					 "auto gyro bias = [%.3f %.3f %.3f] rad/s (%.2f %.2f %.2f "
					 "dps), N=%u",
					 st->gbx, st->gby, st->gbz, st->gbx * 57.29578f,
					 st->gby * 57.29578f, st->gbz * 57.29578f, st->bias_count);
		}
		// (If you want a hard stop even when moving, add a max duration guard
		// here.)
	}
	// Bias-compensated gyro
	const float gx = gxr - st->gbx;
	const float gy = gyr - st->gby;
	const float gz = gzr - st->gbz;

	// Integrate gyro
	st->roll += gx * dt;
	st->pitch += gy * dt;
	st->yaw += gz * dt;

	// Accel tilt correction (complementary, sign-agnostic)
	const float ra = std::atan2(ay, std::sqrt(ax * ax + az * az));
	const float pa = std::atan2(-ax, std::sqrt(ay * ay + az * az));
	const float a = st->accel_alpha;
	st->roll = (1.f - a) * st->roll + a * ra;
	st->pitch = (1.f - a) * st->pitch + a * pa;

	// Tilt-compensated magnetic yaw (best effort)
	if (have_mag(in))
	{
		float yaw_mag = tiltcomp_yaw(st->roll, st->pitch, in->mag.mx,
									 in->mag.my, in->mag.mz) +
						st->decl;
		// unwrap then blend
		float dy = std::fmod(yaw_mag - st->yaw + PI, 2 * PI) - PI;
		st->yaw += st->yaw_beta * dy;
	}

	// Wrap yaw into [-pi, pi] for neatness
	if (st->yaw > PI)
		st->yaw -= 2 * PI;
	if (st->yaw < -PI)
		st->yaw += 2 * PI;

	// Output
	out->state = FusionState{};
	out->state.t_us = (in->imu.t_us ? in->imu.t_us : 0);
	out->state.roll = st->roll;
	out->state.pitch = st->pitch;
	out->state.yaw = st->yaw;
	// TODO: quaternion later
}

static const Algo kAlgo = {"complementary", sizeof(State), alignof(State),
						   &init_fn,		&step_fn,	   &reset_fn};

const Algo *complementary_algo()
{
	return &kAlgo;
}

} // namespace fusion
