#pragma once

#include <cstdint>

#include "regbus/Registry.hpp"
#include "ECGTypes.hpp"

enum class SFKey : uint8_t
{
    ECG_RAW,
	CMD_RESET
};

template <SFKey K> struct SFTraits;

template <> struct SFTraits<SFKey::ECG_RAW>
{
    using type = ECGSample;
    static constexpr regbus::Kind kind = regbus::Kind::Data;
};
 

template <> struct SFTraits<SFKey::CMD_RESET>
{
	using type = bool;
	static constexpr regbus::Kind kind = regbus::Kind::Cmd;
};

using SFReg =
	regbus::Registry<SFKey, SFTraits,
                     SFKey::CMD_RESET,
                     SFKey::ECG_RAW>;