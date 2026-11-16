// Peak programme meter (PPM), BBC / EBU type: IEC 60268-10 Type II.
//
// Ballistics (quasi-peak, so very short peaks read low):
//   - A 5 kHz tone burst of 10 ms reads 4 dB below the same tone held
//     steady, 5 ms reads 9 dB below, 1.5 ms reads 17 dB below.
//   - Falls back 24 dB (mark 7 to mark 1) in 2.8 s: 8.6 dB/s, linear in dB.
// A steady sine reads its RMS level, as the meter is aligned with sine tone.
//
// Implementation: full-wave rectifier into a peak detector. While the input
// is above the reading it charges with time constant `attack_tau`; otherwise
// the reading decays at the fall rate. A single time constant can't match
// all three burst figures exactly; attack_tau = 6.0 ms gives -4.6, -8.3 and
// -16.7 dB, each inside the tolerances (±0.75, ±1, ±2 dB) that
// test/host_test.cpp checks.
//
// Steady tones are aligned at 1 kHz (see the constructor) and read within
// 0.2 dB from 100 Hz to 10 kHz at 48 kHz.
//
// A 20 Hz high-pass (2nd-order Butterworth) removes DC and infrasound, which
// a programme meter ignores.
//
// Scale: BBC marks 1..7, 4 dB apart, with 4 at the alignment level; the EBU
// scale (Type IIb) puts the same positions at -12..+12 dB with "TEST" at 0.
// ppm_position() maps dB relative to mark 4 onto 0..8 where marks are at
// 1..7, 0 is the rest stop and 8 is the end stop.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cmath>

#include "a_weighting.hpp"  // slm::Biquad

namespace slm {

class PpmMeter {
public:
    static constexpr double kFallDbPerSecond = 24.0 / 2.8;

    explicit PpmMeter(double fs, double attack_tau = 0.0060)
    {
        k_attack_ = float(1.0 - std::exp(-1.0 / (fs * attack_tau)));
        k_release_ = float(std::pow(10.0, -kFallDbPerSecond / 20.0 / fs));

        // 20 Hz Butterworth high-pass (RBJ cookbook, Q = 1/sqrt 2).
        const double pi = 3.14159265358979323846;
        const double w0 = 2 * pi * 20.0 / fs, cw = std::cos(w0), alpha = std::sin(w0) / (2 * 0.70710678);
        const double a0 = 1 + alpha;
        hp_.b0 = float((1 + cw) / 2 / a0);
        hp_.b1 = float(-(1 + cw) / a0);
        hp_.b2 = float((1 + cw) / 2 / a0);
        hp_.a1 = float(-2 * cw / a0);
        hp_.a2 = float((1 - alpha) / a0);

        // Alignment: a steady sine must read its RMS level. Because the
        // detector only charges near each peak, against the slow fall, it
        // settles a little below the peak (about 0.35 dB at these settings),
        // so measure a 1 kHz tone once here and scale readings to match.
        for (int i = 0; i < int(fs * 0.5); ++i) process(float(std::sin(2 * pi * 1000.0 * i / fs)));
        scale_ = 0.70710678f / env_;
        reset();
    }

    inline void process(float x)
    {
        const float a = std::fabs(hp_.process(x));
        if (a > env_) env_ += k_attack_ * (a - env_);
        else env_ = env_ * k_release_ + 1e-12f;  // + tiny floor: never denormal
    }

    // Reading in dB re full scale, as the RMS of the steady sine that would
    // give the same reading.
    float reading_db() const { return 20.0f * std::log10(env_ * scale_ + 1e-10f); }

    void reset()
    {
        env_ = 0;
        hp_.reset();
    }

private:
    Biquad hp_;
    float k_attack_, k_release_;
    float env_ = 0;
    float scale_ = 0.70710678f;  // envelope -> sine RMS, set in the constructor
};

// Needle position for a level `d` dB relative to mark 4: marks 1..7 are 4 dB
// apart (mark 1 = -12 dB, mark 7 = +12 dB). Below mark 1 the scale is
// compressed (12 dB to the rest stop at 0); above 7 it continues to the end
// stop at 8 (+16 dB).
inline float ppm_position(float d)
{
    float p;
    if (d >= -12.0f) p = 4.0f + d / 4.0f;
    else p = 1.0f - (-12.0f - d) / 12.0f;
    return p < 0 ? 0 : (p > 8 ? 8 : p);
}

}  // namespace slm
