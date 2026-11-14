// Sound level computation for a two-microphone input.
//
// Input: blocks of left/right samples scaled to +/-1.0 full scale.
// Output (all in dBFS, where an RMS of 1.0 is 0 dB):
//   laf    A-weighted, Fast (125 ms) exponential time weighting
//   las    A-weighted, Slow (1 s) exponential time weighting
//   laeq   A-weighted equivalent continuous level over the last 15 minutes
//          (rolling), or since the last reset if that was more recent
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
// Rolling LAeq: the mean square of each whole second is stored in a ring of
// kLeqWindowSeconds entries. LAeq is the energy average of the ring plus the
// partly filled current second, so it covers exactly the last 15 minutes
// (to the sample) once 15 minutes have passed, and everything since the
// reset before that.
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
    float leq_seconds = 0;  // time covered by laeq (up to the window length)
    bool clipping = false;
};

class LevelMeter {
public:
    static constexpr size_t kLeqWindowSeconds = 15 * 60;

    explicit LevelMeter(double fs)
        : fs_(fs), wa_(fs), w1_(fs), w2_(fs)
    {
        alpha_fast_ = float(1.0 - std::exp(-1.0 / (fs * 0.125)));
        alpha_slow_ = float(1.0 - std::exp(-1.0 / (fs * 1.0)));
        samples_per_second_ = size_t(std::lround(fs));
        // Ignore min/max/Leq for the first 0.5 s while the filters settle.
        settle_left_ = size_t(fs * 0.5);
        clip_hold_samples_ = size_t(fs);
    }

    void process(const float *left, const float *right, size_t n)
    {
        float fast = fast_, slow = slow_, m1 = m1_, m2 = m2_;
        float vmax = max_ms_, vmin = min_ms_;
        bool clipped = false;

        for (size_t i = 0; i < n; ++i) {
            const float l = left[i], r = right[i];
            if (std::fabs(l) >= kClipLevel || std::fabs(r) >= kClipLevel) clipped = true;

            const float y = wa_.process(0.5f * (l + r));
            const float y1 = w1_.process(l);
            const float y2 = w2_.process(r);
            const float p = y * y;

            fast += alpha_fast_ * (p - fast);
            slow += alpha_slow_ * (p - slow);
            m1 += alpha_fast_ * (y1 * y1 - m1);
            m2 += alpha_fast_ * (y2 * y2 - m2);

            if (settle_left_ > 0) {
                --settle_left_;
                continue;
            }
            if (fast > vmax) vmax = fast;
            if (fast < vmin) vmin = fast;

            second_sum_ += p;
            if (++second_count_ == samples_per_second_) push_second();
        }

        fast_ = fast; slow_ = slow; m1_ = m1; m2_ = m2;
        max_ms_ = vmax;
        min_ms_ = vmin;

        if (clipped) {
            clip_left_ = clip_hold_samples_;
        } else {
            clip_left_ = clip_left_ > n ? clip_left_ - n : 0;
        }
    }

    // Clears LAeq (the 15-minute window), max and min. Time-weighted levels
    // keep running.
    void reset_stats()
    {
        ring_count_ = 0;
        ring_head_ = 0;
        ring_sum_ = 0;
        second_sum_ = 0;
        second_count_ = 0;
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
        // Whole seconds in the ring plus the current partial second.
        const double samples = double(ring_count_) * samples_per_second_ + double(second_count_);
        if (samples > 0) {
            const double energy = ring_sum_ * samples_per_second_ + second_sum_;
            s.laeq = to_db(float(energy / samples));
            s.leq_seconds = float(samples / fs_);
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

    // Store the finished second; drop the oldest once the window is full.
    void push_second()
    {
        ring_[ring_head_] = second_sum_ / double(samples_per_second_);
        ring_head_ = (ring_head_ + 1) % kLeqWindowSeconds;
        if (ring_count_ < kLeqWindowSeconds) ++ring_count_;
        second_sum_ = 0;
        second_count_ = 0;
        // Re-add the ring (900 adds per second) rather than keeping a running
        // total, so rounding errors can never build up over long runs.
        double sum = 0;
        for (size_t i = 0; i < ring_count_; ++i) sum += ring_[i];
        ring_sum_ = sum;
    }

    double fs_;
    AWeighting wa_, w1_, w2_;
    float alpha_fast_, alpha_slow_;
    float fast_ = 0, slow_ = 0, m1_ = 0, m2_ = 0;
    float max_ms_ = 0, min_ms_ = kHuge;
    size_t samples_per_second_;
    size_t settle_left_;
    size_t clip_left_ = 0, clip_hold_samples_;

    // Rolling LAeq window
    double ring_[kLeqWindowSeconds] = {};  // mean square of each second
    size_t ring_head_ = 0, ring_count_ = 0;
    double ring_sum_ = 0;                  // sum of ring_[0 .. ring_count_)
    double second_sum_ = 0;                // sum of p in the current second
    size_t second_count_ = 0;
};

}  // namespace slm
