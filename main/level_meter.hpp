// Sound level computation for a two-microphone input.
//
// Input: blocks of left/right samples scaled to +/-1.0 full scale.
// Output (all in dBFS, where an RMS of 1.0 is 0 dB):
//   laf    A-weighted, Fast (125 ms) exponential time weighting
//   las    A-weighted, Slow (1 s) exponential time weighting
//   laeq   A-weighted equivalent continuous level since the last reset
//   lafmax / lafmin  max / min of LAF since the last reset
//   mic1 / mic2      LAF of each microphone on its own
//   clipping         true if either raw channel hit full scale in the last ~1 s
//
// The main reading uses the average of the two microphones.  Acoustic sound
// arrives at both capsules (nearly) in phase and adds coherently, while the
// self-noise of each capsule is uncorrelated, so averaging lowers the noise
// floor by about 3 dB.  Per-microphone levels are kept so a blocked or faulty
// capsule is easy to spot.
//
// Converting to dB SPL is a single calibration offset added by the caller.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cmath>
#include <cstddef>

#include "a_weighting.hpp"

namespace slm {

struct Levels {
    float laf = -200, las = -200, laeq = -200;
    float lafmax = -200, lafmin = -200;
    float mic1 = -200, mic2 = -200;
    float leq_seconds = 0;
    bool clipping = false;
};

class LevelMeter {
public:
    explicit LevelMeter(double fs)
        : fs_(fs), wa_(fs), w1_(fs), w2_(fs)
    {
        alpha_fast_ = float(1.0 - std::exp(-1.0 / (fs * 0.125)));
        alpha_slow_ = float(1.0 - std::exp(-1.0 / (fs * 1.0)));
        // Ignore min/max for the first 0.5 s while the filters settle.
        settle_left_ = size_t(fs * 0.5);
        clip_hold_samples_ = size_t(fs);
    }

    void process(const float *left, const float *right, size_t n)
    {
        double block_sum = 0;
        float fast = fast_, slow = slow_, m1 = m1_, m2 = m2_;
        float vmax = max_ms_, vmin = min_ms_;
        const bool track = settle_left_ == 0;
        bool clipped = false;

        for (size_t i = 0; i < n; ++i) {
            const float l = left[i], r = right[i];
            if (std::fabs(l) >= kClipLevel || std::fabs(r) >= kClipLevel) clipped = true;

            const float y = wa_.process(0.5f * (l + r));
            const float y1 = w1_.process(l);
            const float y2 = w2_.process(r);
            const float p = y * y;

            block_sum += p;
            fast += alpha_fast_ * (p - fast);
            slow += alpha_slow_ * (p - slow);
            m1 += alpha_fast_ * (y1 * y1 - m1);
            m2 += alpha_fast_ * (y2 * y2 - m2);
            if (track) {
                if (fast > vmax) vmax = fast;
                if (fast < vmin) vmin = fast;
            }
        }

        fast_ = fast; slow_ = slow; m1_ = m1; m2_ = m2;
        if (track) {
            max_ms_ = vmax;
            min_ms_ = vmin;
            leq_sum_ += block_sum;
            leq_count_ += n;
        } else {
            settle_left_ = settle_left_ > n ? settle_left_ - n : 0;
        }

        if (clipped) {
            clip_left_ = clip_hold_samples_;
        } else {
            clip_left_ = clip_left_ > n ? clip_left_ - n : 0;
        }
    }

    // Clears LAeq, max and min.  Time-weighted levels keep running.
    void reset_stats()
    {
        leq_sum_ = 0;
        leq_count_ = 0;
        max_ms_ = 0;
        min_ms_ = kHuge;
    }

    Levels snapshot() const
    {
        Levels s;
        s.laf = to_db(fast_);
        s.las = to_db(slow_);
        s.mic1 = to_db(m1_);
        s.mic2 = to_db(m2_);
        if (leq_count_ > 0) {
            s.laeq = to_db(float(leq_sum_ / double(leq_count_)));
            s.leq_seconds = float(double(leq_count_) / fs_);
        }
        if (max_ms_ > 0) s.lafmax = to_db(max_ms_);
        if (min_ms_ < kHuge) s.lafmin = to_db(min_ms_);
        s.clipping = clip_left_ > 0;
        return s;
    }

    static float to_db(float ms) { return 10.0f * std::log10(ms > 1e-20f ? ms : 1e-20f); }

private:
    static constexpr float kClipLevel = 0.999f;
    static constexpr float kHuge = 1e30f;

    double fs_;
    AWeighting wa_, w1_, w2_;
    float alpha_fast_, alpha_slow_;
    float fast_ = 0, slow_ = 0, m1_ = 0, m2_ = 0;
    float max_ms_ = 0, min_ms_ = kHuge;
    double leq_sum_ = 0;
    unsigned long long leq_count_ = 0;
    size_t settle_left_;
    size_t clip_left_ = 0, clip_hold_samples_;
};

}  // namespace slm
