#include "ZuptDetector.hpp"
#include <cmath>

namespace sf {

ZuptDetector::ZuptDetector(float threshold)
    : threshold_(threshold) {}

bool ZuptDetector::processSample(const AccelData& accel) {
    float mag = std::sqrt(accel.x * accel.x +
                          accel.y * accel.y +
                          accel.z * accel.z);

    if (count_ == WINDOW_SIZE) {
        float old = magBuf_[idx_];
        sum_ -= old;
        sumSq_ -= old * old;
    } else {
        ++count_;
    }

    magBuf_[idx_] = mag;
    sum_ += mag;
    sumSq_ += mag * mag;
    idx_ = (idx_ + 1) % WINDOW_SIZE;

    if (count_ >= 2) {
        float mean = sum_ / static_cast<float>(count_);
        currentVariance_ = (sumSq_ / static_cast<float>(count_)) - (mean * mean);
        if (currentVariance_ < 0.0f) currentVariance_ = 0.0f;
    }

    stationary_ = (count_ == WINDOW_SIZE) && (currentVariance_ < threshold_);
    return stationary_;
}

bool ZuptDetector::isStationary() const { return stationary_; }
float ZuptDetector::variance() const { return currentVariance_; }

void ZuptDetector::reset() {
    stationary_ = false;
    currentVariance_ = 1.0f;
    idx_ = 0;
    count_ = 0;
    sum_ = 0.0f;
    sumSq_ = 0.0f;
    for (size_t i = 0; i < WINDOW_SIZE; ++i) magBuf_[i] = 0.0f;
}

} // namespace sf
