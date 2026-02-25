#pragma once

#include "SensorTypes.hpp"
#include <cstddef>

namespace sf {

class ZuptDetector {
public:
    explicit ZuptDetector(float threshold = 0.01f);

    bool processSample(const AccelData& accel);
    bool isStationary() const;
    float variance() const;
    void reset();

private:
    static constexpr size_t WINDOW_SIZE = 32;

    float threshold_;
    bool stationary_ = false;
    float currentVariance_ = 1.0f;

    float magBuf_[WINDOW_SIZE]{};
    size_t idx_ = 0;
    size_t count_ = 0;
    float sum_ = 0.0f;
    float sumSq_ = 0.0f;
};

} // namespace sf
