#pragma once

#include <cstdint>

#include "regbus/Registry.hpp"

enum class SFKey : uint8_t
{
	IMU_RAW,
	MAG_RAW,
	BARO_RAW,
	FUSION_STATE,
	CMD_RESET
};

struct IMURaw
{
	uint64_t t_us;
	float ax, ay, az, gx, gy, gz;
};

struct MagRaw
{
	uint64_t t_us;
	float mx, my, mz;
};

struct BaroRaw
{
	uint64_t t_us;
	float temp_c;
	float pressure_pa;
	float alt_m;
};

struct FusionState
{
	uint64_t t_us;
	float qw, qx, qy, qz;
	float roll, pitch, yaw;
};

template <SFKey K> struct SFTraits;

template <> struct SFTraits<SFKey::IMU_RAW>
{
	using type = IMURaw;
	static constexpr regbus::Kind kind = regbus::Kind::Data;
};

template <> struct SFTraits<SFKey::MAG_RAW>
{
	using type = MagRaw;
	static constexpr regbus::Kind kind = regbus::Kind::Data;
};

template <> struct SFTraits<SFKey::BARO_RAW>
{
	using type = BaroRaw;
	static constexpr regbus::Kind kind = regbus::Kind::Data;
};

template <> struct SFTraits<SFKey::FUSION_STATE>
{
	using type = FusionState;
	static constexpr regbus::Kind kind = regbus::Kind::Data;
};

template <> struct SFTraits<SFKey::CMD_RESET>
{
	using type = bool;
	static constexpr regbus::Kind kind = regbus::Kind::Cmd;
};

using SFReg =
	regbus::Registry<SFKey, SFTraits, SFKey::IMU_RAW, SFKey::MAG_RAW,
					 SFKey::BARO_RAW, SFKey::FUSION_STATE, SFKey::CMD_RESET>;
