#pragma once

#include <cstdint>

// Small helper for 3-component vectors
struct Vec3f
{
	float x;
	float y;
	float z;
};

// Raw IMU reading (scaled to SI units)
struct ImuSample
{
	Vec3f acceleration;	 // m/s^2
	Vec3f rotation;		 // rad/s
	float temperature_c; // °C
	uint32_t timestamp_ms;
};

// Magnetometer reading (microtesla)
struct MagSample
{
	Vec3f magnetic; // µT
	uint32_t timestamp_ms;
};

// Barometer/thermometer reading
struct BaroSample
{
	float pressure_pa;	 // Pa
	float temperature_c; // °C
	uint32_t timestamp_ms;
};

// High-level fused state (attitude + altitude)
struct FusedSample
{
	struct
	{
		float yaw_deg;
		float pitch_deg;
		float roll_deg;
	} attitude;
	float altitude_m; // m (relative)
	uint32_t timestamp_ms;
};
