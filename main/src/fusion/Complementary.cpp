#include "fusion/Complementary.hpp"

#include <cmath>

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
	float roll{0}, pitch{0}, yaw{0}; // radians
	float gbx{0}, gby{0}, gbz{0};	 // gyro bias (rad/s)
	float accel_alpha{0.02f};
	float yaw_beta{0.02f};
	float decl{0.0f}; // radians
	int64_t last_us{0};
};

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

static void step_fn(void *s, const Inputs *in, Outputs *out)
{
	auto *st = static_cast<State *>(s);
	if (!in || !out)
		return;

	// dt from producer timestamp if available; fallback to provided dt
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

	// Units: IMURaw assumed ax/ay/az in g, gx/gy/gz in deg/s
	const float gx = deg2rad(in->imu.gx) - st->gbx;
	const float gy = deg2rad(in->imu.gy) - st->gby;
	const float gz = deg2rad(in->imu.gz) - st->gbz;
	const float ax = in->imu.ax * 9.80665f;
	const float ay = in->imu.ay * 9.80665f;
	const float az = in->imu.az * 9.80665f;

	if (!st->inited)
	{
		st->roll = std::atan2(ay, std::sqrt(ax * ax + az * az));
		st->pitch = std::atan2(-ax, az);
		st->yaw = 0.f;
		st->inited = true;
	}

	// Integrate gyro
	st->roll += gx * dt;
	st->pitch += gy * dt;
	st->yaw += gz * dt;

	// Accel tilt correction (complementary)
	const float ra = std::atan2(ay, std::sqrt(ax * ax + az * az));
	const float pa = std::atan2(-ax, az);
	const float a = st->accel_alpha;
	st->roll = (1.f - a) * st->roll + a * ra;
	st->pitch = (1.f - a) * st->pitch + a * pa;

	// Tilt-compensated mag yaw (best effort)
	if (in->mag.mx != 0.f || in->mag.my != 0.f || in->mag.mz != 0.f)
	{
		const float cr = cosf(st->roll), sr = sinf(st->roll);
		const float cp = cosf(st->pitch), sp = sinf(st->pitch);
		const float mx = in->mag.mx, my = in->mag.my, mz = in->mag.mz;
		const float mhx = mx * cp + my * sr * sp + mz * cr * sp;
		const float mhy = my * cr - mz * sr;
		float yaw_mag = std::atan2f(-mhy, mhx) + st->decl; // add declination
		// unwrap and blend
		float dy = fmodf(yaw_mag - st->yaw + PI, 2 * PI) - PI;
		st->yaw += st->yaw_beta * dy;
	}

	// wrap yaw
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
	// TODO: compute quaternion later
}

static const Algo kAlgo = {"complementary", sizeof(State), alignof(State),
						   &init_fn,		&step_fn,	   &reset_fn};

const Algo *complementary_algo()
{
	return &kAlgo;
}

} // namespace fusion
