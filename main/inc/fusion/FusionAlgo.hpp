#pragma once

#include <cstddef>
#include <cstdint>

#include "SFRegistry.hpp" // IMURaw, MagRaw, BaroRaw, FusionState

namespace fusion
{

// What the algo gets each step
struct Inputs
{
	IMURaw imu;	  // producer timestamp in imu.t_us
	MagRaw mag;	  // optional; may be zeroed if not read
	BaroRaw baro; // optional
	float dt;	  // seconds
};

// What the algo produces each step
struct Outputs
{
	FusionState state; // fill roll/pitch/yaw, quat later, etc.
};

// Tunables (can extend later; keep POD)
struct Config
{
	float accel_alpha = 0.02f;		// complementary tilt blend  (0..1)
	float yaw_beta = 0.02f;			// mag/gyro yaw blend        (0..1)
	float mag_decl_deg = 0.0f;		// magnetic declination
	float gyro_bias[3] = {0, 0, 0}; // rad/s
};

// Function pointer ABI
using InitFn = void (*)(void *state, const Config *cfg);
using StepFn = void (*)(void *state, const Inputs *in, Outputs *out);
using ResetFn = void (*)(void *state);

// Descriptor
struct Algo
{
	const char *name;
	uint32_t state_size;
	uint32_t state_align;
	InitFn init;
	StepFn step;
	ResetFn reset;
};

// Helper for fixed, aligned storage
template <size_t N, size_t A> struct Aligned
{
	alignas(A) std::uint8_t bytes[N];
	void *ptr()
	{
		return bytes;
	}
	const void *ptr() const
	{
		return bytes;
	}
};

} // namespace fusion
