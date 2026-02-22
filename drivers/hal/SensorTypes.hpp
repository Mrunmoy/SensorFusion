#pragma once

#include <cstdint>

namespace sf {

struct AccelData { float x, y, z; };  // in g
struct GyroData  { float x, y, z; };  // in deg/s
struct MagData   { float x, y, z; };  // in µT

} // namespace sf
