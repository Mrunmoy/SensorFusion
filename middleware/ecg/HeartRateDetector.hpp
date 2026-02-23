#pragma once

#include <cstdint>
#include <cstddef>

namespace sf {

class HeartRateDetector {
public:
    explicit HeartRateDetector(float sampleRateHz = 360.0f);

    void processSample(int32_t millivolts);
    float heartRateBpm() const;
    bool rPeakDetected() const;
    void reset();

private:
    float sampleRate_;
    bool rPeakFlag_ = false;
    float bpm_ = 0.0f;

    // Bandpass filter state (cascaded low-pass + high-pass)
    // Low-pass: y[n] = 2*y[n-1] - y[n-2] + x[n] - 2*x[n-6] + x[n-12]
    static constexpr size_t LP_DELAY = 12;
    float lpBuf_[LP_DELAY + 1]{};
    size_t lpIdx_ = 0;
    float lpY1_ = 0.0f;
    float lpY2_ = 0.0f;

    // High-pass: y[n] = y[n-1] - x[n]/32 + x[n-16] - x[n-17] + x[n-32]/32
    static constexpr size_t HP_DELAY = 32;
    float hpBuf_[HP_DELAY + 1]{};
    size_t hpIdx_ = 0;
    float hpY1_ = 0.0f;

    // Derivative: y[n] = (1/8) * (-x[n-2] - 2*x[n-1] + 2*x[n+1-4] + x[n-4])
    // Simplified: y[n] = (2*x[n] + x[n-1] - x[n-3] - 2*x[n-4]) / 8
    static constexpr size_t DERIV_DELAY = 4;
    float derivBuf_[DERIV_DELAY + 1]{};
    size_t derivIdx_ = 0;

    // Moving window integrator
    static constexpr size_t MWI_WINDOW = 30;  // ~83ms at 360Hz
    float mwiBuf_[MWI_WINDOW]{};
    size_t mwiIdx_ = 0;
    float mwiSum_ = 0.0f;

    // Adaptive thresholding
    float threshold_ = 0.0f;
    float signalLevel_ = 0.0f;
    float noiseLevel_ = 0.0f;

    // R-R interval tracking
    uint32_t sampleCount_ = 0;
    uint32_t lastPeakSample_ = 0;
    static constexpr size_t RR_HISTORY = 8;
    float rrIntervals_[RR_HISTORY]{};
    size_t rrIdx_ = 0;
    size_t rrCount_ = 0;

    // Refractory period (~200ms)
    uint32_t refractoryPeriod_ = 0;

    float lowPassFilter(float x);
    float highPassFilter(float x);
    float derivative(float x);
    float movingWindowIntegrator(float x);
    void updateThreshold(float mwiVal, bool isPeak);
    void recordRR(uint32_t interval);
};

} // namespace sf
