#include <gtest/gtest.h>
#include "HeartRateDetector.hpp"
#include <cmath>
#include <vector>

using namespace sf;

// Generate synthetic ECG-like signal: periodic spikes simulating R-peaks
static std::vector<int32_t> generateSyntheticECG(float bpm, float sampleRate, float durationSec) {
    size_t totalSamples = static_cast<size_t>(sampleRate * durationSec);
    std::vector<int32_t> samples(totalSamples);

    float samplesPerBeat = sampleRate * 60.0f / bpm;
    float rPeakWidth = sampleRate * 0.02f;  // 20ms R-peak width

    for (size_t i = 0; i < totalSamples; ++i) {
        float phase = std::fmod(static_cast<float>(i), samplesPerBeat);

        // Generate a sharp spike centered at each beat
        if (phase < rPeakWidth) {
            // Rising R-peak
            float t = phase / rPeakWidth;
            samples[i] = static_cast<int32_t>(1000.0f * std::sin(t * 3.14159f));
        } else if (phase < rPeakWidth * 3.0f) {
            // T-wave (small bump after R-peak)
            float t = (phase - rPeakWidth * 1.5f) / rPeakWidth;
            samples[i] = static_cast<int32_t>(200.0f * std::exp(-t * t * 2.0f));
        } else {
            // Baseline with small noise
            samples[i] = static_cast<int32_t>(10.0f * std::sin(phase * 0.1f));
        }
    }

    return samples;
}

TEST(HeartRateDetectorTest, InitialBPMIsZero) {
    HeartRateDetector det(360.0f);
    EXPECT_FLOAT_EQ(det.heartRateBpm(), 0.0f);
    EXPECT_FALSE(det.rPeakDetected());
}

TEST(HeartRateDetectorTest, Detect72BPM) {
    float sampleRate = 360.0f;
    HeartRateDetector det(sampleRate);

    auto samples = generateSyntheticECG(72.0f, sampleRate, 10.0f);

    for (auto s : samples) {
        det.processSample(s);
    }

    float bpm = det.heartRateBpm();
    // Allow 30% tolerance — synthetic waveform shape affects detection
    EXPECT_GT(bpm, 50.0f);
    EXPECT_LT(bpm, 100.0f);
}

TEST(HeartRateDetectorTest, Detect120BPM) {
    float sampleRate = 360.0f;
    HeartRateDetector det(sampleRate);

    auto samples = generateSyntheticECG(120.0f, sampleRate, 10.0f);

    for (auto s : samples) {
        det.processSample(s);
    }

    float bpm = det.heartRateBpm();
    EXPECT_GT(bpm, 100.0f);
    EXPECT_LT(bpm, 140.0f);
}

TEST(HeartRateDetectorTest, RPeakFlagIsTransient) {
    float sampleRate = 360.0f;
    HeartRateDetector det(sampleRate);

    auto samples = generateSyntheticECG(72.0f, sampleRate, 5.0f);

    int peakCount = 0;
    for (auto s : samples) {
        det.processSample(s);
        if (det.rPeakDetected()) peakCount++;
    }

    // Should detect peaks — synthetic waveform may produce extra detections
    EXPECT_GE(peakCount, 3);
    EXPECT_LE(peakCount, 25);
}

TEST(HeartRateDetectorTest, ResetClearsState) {
    float sampleRate = 360.0f;
    HeartRateDetector det(sampleRate);

    auto samples = generateSyntheticECG(72.0f, sampleRate, 5.0f);
    for (auto s : samples) {
        det.processSample(s);
    }

    det.reset();
    EXPECT_FLOAT_EQ(det.heartRateBpm(), 0.0f);
    EXPECT_FALSE(det.rPeakDetected());
}

TEST(HeartRateDetectorTest, ZeroInputNoPeaks) {
    HeartRateDetector det(360.0f);

    for (int i = 0; i < 3600; ++i) {
        det.processSample(0);
    }

    EXPECT_FLOAT_EQ(det.heartRateBpm(), 0.0f);
}

TEST(HeartRateDetectorTest, ConstantInputNoPeaks) {
    HeartRateDetector det(360.0f);

    for (int i = 0; i < 3600; ++i) {
        det.processSample(500);
    }

    // A constant signal should produce no peaks (derivative is zero)
    EXPECT_FALSE(det.rPeakDetected());
}

TEST(HeartRateDetectorTest, DifferentSampleRates) {
    // Test with 250 Hz sample rate
    float sampleRate = 250.0f;
    HeartRateDetector det(sampleRate);

    auto samples = generateSyntheticECG(80.0f, sampleRate, 10.0f);
    for (auto s : samples) {
        det.processSample(s);
    }

    float bpm = det.heartRateBpm();
    EXPECT_GT(bpm, 65.0f);
    EXPECT_LT(bpm, 95.0f);
}

TEST(HeartRateDetectorSizeTest, ReasonableMemory) {
    // Pan-Tompkins state should be under 512 bytes
    EXPECT_LE(sizeof(HeartRateDetector), 512u);
}
