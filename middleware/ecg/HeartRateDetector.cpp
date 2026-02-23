#include "HeartRateDetector.hpp"
#include <cmath>

namespace sf {

HeartRateDetector::HeartRateDetector(float sampleRateHz)
    : sampleRate_(sampleRateHz) {
    refractoryPeriod_ = static_cast<uint32_t>(sampleRateHz * 0.2f);  // 200ms
}

void HeartRateDetector::processSample(int32_t millivolts) {
    rPeakFlag_ = false;
    float x = static_cast<float>(millivolts);

    // Pan-Tompkins pipeline
    float lp = lowPassFilter(x);
    float hp = highPassFilter(lp);
    float d = derivative(hp);
    float sq = d * d;  // squaring
    float mwi = movingWindowIntegrator(sq);

    sampleCount_++;

    // Check for peak (above threshold, past refractory period)
    bool pastRefractory = (sampleCount_ - lastPeakSample_) > refractoryPeriod_;

    if (mwi > threshold_ && pastRefractory) {
        // Signal peak detected
        rPeakFlag_ = true;

        if (lastPeakSample_ > 0) {
            uint32_t interval = sampleCount_ - lastPeakSample_;
            recordRR(interval);
        }

        lastPeakSample_ = sampleCount_;
        updateThreshold(mwi, true);
    } else {
        updateThreshold(mwi, false);
    }
}

float HeartRateDetector::heartRateBpm() const {
    return bpm_;
}

bool HeartRateDetector::rPeakDetected() const {
    return rPeakFlag_;
}

void HeartRateDetector::reset() {
    rPeakFlag_ = false;
    bpm_ = 0.0f;

    for (auto& v : lpBuf_) v = 0.0f;
    lpIdx_ = 0; lpY1_ = 0.0f; lpY2_ = 0.0f;

    for (auto& v : hpBuf_) v = 0.0f;
    hpIdx_ = 0; hpY1_ = 0.0f;

    for (auto& v : derivBuf_) v = 0.0f;
    derivIdx_ = 0;

    for (auto& v : mwiBuf_) v = 0.0f;
    mwiIdx_ = 0; mwiSum_ = 0.0f;

    threshold_ = 0.0f;
    signalLevel_ = 0.0f;
    noiseLevel_ = 0.0f;

    sampleCount_ = 0;
    lastPeakSample_ = 0;
    for (auto& v : rrIntervals_) v = 0.0f;
    rrIdx_ = 0; rrCount_ = 0;
}

// Pan-Tompkins low-pass filter (second-order)
// H(z) = (1 - z^-6)^2 / (1 - z^-1)^2
// y[n] = 2*y[n-1] - y[n-2] + x[n] - 2*x[n-6] + x[n-12]
float HeartRateDetector::lowPassFilter(float x) {
    size_t n = lpIdx_;
    lpBuf_[n] = x;

    size_t n6 = (n + LP_DELAY + 1 - 6) % (LP_DELAY + 1);
    size_t n12 = (n + LP_DELAY + 1 - 12) % (LP_DELAY + 1);

    float y = 2.0f * lpY1_ - lpY2_ + x - 2.0f * lpBuf_[n6] + lpBuf_[n12];
    lpY2_ = lpY1_;
    lpY1_ = y;

    lpIdx_ = (n + 1) % (LP_DELAY + 1);
    return y / 36.0f;  // normalize gain
}

// Pan-Tompkins high-pass filter
// H(z) = (-1/32 + z^-16 - z^-17 + z^-32/32) / (1 - z^-1)
// y[n] = y[n-1] - x[n]/32 + x[n-16] - x[n-17] + x[n-32]/32
float HeartRateDetector::highPassFilter(float x) {
    size_t n = hpIdx_;
    hpBuf_[n] = x;

    size_t n16 = (n + HP_DELAY + 1 - 16) % (HP_DELAY + 1);
    size_t n17 = (n + HP_DELAY + 1 - 17) % (HP_DELAY + 1);
    size_t n32 = (n + HP_DELAY + 1 - 32) % (HP_DELAY + 1);

    float y = hpY1_ - x / 32.0f + hpBuf_[n16] - hpBuf_[n17] + hpBuf_[n32] / 32.0f;
    hpY1_ = y;

    hpIdx_ = (n + 1) % (HP_DELAY + 1);
    return y;
}

// Five-point derivative
// y[n] = (2*x[n] + x[n-1] - x[n-3] - 2*x[n-4]) / 8
float HeartRateDetector::derivative(float x) {
    size_t n = derivIdx_;
    derivBuf_[n] = x;

    size_t n1 = (n + DERIV_DELAY + 1 - 1) % (DERIV_DELAY + 1);
    size_t n3 = (n + DERIV_DELAY + 1 - 3) % (DERIV_DELAY + 1);
    size_t n4 = (n + DERIV_DELAY + 1 - 4) % (DERIV_DELAY + 1);

    float y = (2.0f * x + derivBuf_[n1] - derivBuf_[n3] - 2.0f * derivBuf_[n4]) / 8.0f;

    derivIdx_ = (n + 1) % (DERIV_DELAY + 1);
    return y;
}

// Moving window integrator
float HeartRateDetector::movingWindowIntegrator(float x) {
    mwiSum_ -= mwiBuf_[mwiIdx_];
    mwiBuf_[mwiIdx_] = x;
    mwiSum_ += x;
    mwiIdx_ = (mwiIdx_ + 1) % MWI_WINDOW;
    return mwiSum_ / static_cast<float>(MWI_WINDOW);
}

void HeartRateDetector::updateThreshold(float mwiVal, bool isPeak) {
    if (isPeak) {
        signalLevel_ = 0.125f * mwiVal + 0.875f * signalLevel_;
    } else {
        noiseLevel_ = 0.125f * mwiVal + 0.875f * noiseLevel_;
    }
    threshold_ = noiseLevel_ + 0.25f * (signalLevel_ - noiseLevel_);
}

void HeartRateDetector::recordRR(uint32_t interval) {
    float rrSec = static_cast<float>(interval) / sampleRate_;
    rrIntervals_[rrIdx_] = rrSec;
    rrIdx_ = (rrIdx_ + 1) % RR_HISTORY;
    if (rrCount_ < RR_HISTORY) rrCount_++;

    // Average RR intervals for BPM
    float sum = 0.0f;
    for (size_t i = 0; i < rrCount_; ++i) {
        sum += rrIntervals_[i];
    }
    float avgRR = sum / static_cast<float>(rrCount_);
    if (avgRR > 0.0f) {
        bpm_ = 60.0f / avgRR;
    }
}

} // namespace sf
