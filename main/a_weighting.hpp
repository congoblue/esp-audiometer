// A-weighting filter (IEC 61672-1) implemented as a cascade of three biquads.
//
// The analog prototype is
//
//            k * s^4
//   A(s) = -------------------------------------------
//          (s + w1)^2 (s + w2) (s + w3) (s + w4)^2
//
// with pole frequencies f1 = 20.598997 Hz, f2 = 107.65265 Hz,
// f3 = 737.86223 Hz, f4 = 12194.217 Hz (IEC 61672-1, Annex E).
//
// It is split into three second-order sections:
//   S1: s^2 / (s + w1)^2           high-pass, bilinear transform
//   S2: s^2 / ((s + w2)(s + w3))   high-pass, bilinear transform
//   S3: 1 / (s + w4)^2             low-pass,  matched-z with a fitted zero
//
// The bilinear transform is accurate for the low-frequency poles but squashes
// the 12.2 kHz pole towards Nyquist, which is a large error at 48 kHz.  For S3
// the poles are mapped with z = exp(-w4/fs) and a single zero is placed so the
// digital magnitude matches the analog one at a reference frequency near the
// top of the band.  The combined response stays inside the IEC 61672 class 1
// tolerances from 10 Hz to 16 kHz at fs = 48 kHz (see test/host_test.cpp).
//
// Coefficients are computed in double at start-up; processing runs in float
// using transposed direct form II, which behaves well for the low-frequency
// poles close to z = 1.
//
// This header has no ESP-IDF dependencies so it can be unit-tested on a host.

#pragma once

#include <cmath>
#include <complex>
#include <cstddef>

namespace slm {

struct Biquad {
    // y = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;

    inline float process(float x)
    {
        const float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void reset() { z1 = z2 = 0; }

    // Complex response at normalised angular frequency w (rad/sample).
    std::complex<double> response(double w) const
    {
        const std::complex<double> z1c = std::polar(1.0, -w);
        const std::complex<double> z2c = z1c * z1c;
        return (double(b0) + double(b1) * z1c + double(b2) * z2c) /
               (1.0 + double(a1) * z1c + double(a2) * z2c);
    }
};

class AWeighting {
public:
    static constexpr double kF1 = 20.598997;
    static constexpr double kF2 = 107.65265;
    static constexpr double kF3 = 737.86223;
    static constexpr double kF4 = 12194.217;

    explicit AWeighting(double fs = 48000.0) { design(fs); }

    // Reference analog A-weighting gain in dB (IEC 61672-1 eq. E.6, normalised
    // so that A(1 kHz) = 0 dB).
    static double analog_db(double f)
    {
        const double f2 = f * f;
        const double num = kF4 * kF4 * f2 * f2;
        const double den = (f2 + kF1 * kF1) *
                           std::sqrt((f2 + kF2 * kF2) * (f2 + kF3 * kF3)) *
                           (f2 + kF4 * kF4);
        return 20.0 * std::log10(num / den) - analog_db_raw_1k();
    }

    void design(double fs)
    {
        fs_ = fs;
        const double pi = 3.14159265358979323846;
        const double K = 2.0 * fs;  // bilinear: s = K (1 - z^-1) / (1 + z^-1)

        // --- S1: s^2 / (s + w1)^2, bilinear with pre-warped pole ----------
        {
            const double w = K * std::tan(pi * kF1 / fs);
            set_hp2(s_[0], K, w, w);
        }
        // --- S2: s^2 / ((s + w2)(s + w3)), bilinear with pre-warped poles -
        {
            const double wa = K * std::tan(pi * kF2 / fs);
            const double wb = K * std::tan(pi * kF3 / fs);
            set_hp2(s_[1], K, wa, wb);
        }
        // --- S3: 1 / (s + w4)^2, matched-z poles + fitted zero ------------
        {
            const double w4 = 2.0 * pi * kF4;
            const double p = std::exp(-w4 / fs);
            // H(z) = g (1 + c z^-1) / (1 - p z^-1)^2
            // c is chosen so the DC-normalised |H|^2 matches the analog
            // section at f_ref; g gives unity DC gain (the overall level is
            // normalised at 1 kHz below anyway).
            const double f_ref = std::fmin(0.4 * fs, 16000.0);
            const double x2 = (f_ref / kF4) * (f_ref / kF4);
            const double target = 1.0 / ((1.0 + x2) * (1.0 + x2));  // |1/(1+jf/f4)^2|^2
            c_ = fit_zero(p, 2.0 * pi * f_ref / fs, target);
            const double g = (1.0 - p) * (1.0 - p) / (1.0 + c_);
            Biquad &b = s_[2];
            b.b0 = float(g);
            b.b1 = float(g * c_);
            b.b2 = 0.0f;
            b.a1 = float(-2.0 * p);
            b.a2 = float(p * p);
        }

        // --- Normalise the cascade to 0 dB at 1 kHz ------------------------
        const double w1k = 2.0 * pi * 1000.0 / fs;
        std::complex<double> h = 1.0;
        for (const auto &b : s_) h *= b.response(w1k);
        const double norm = 1.0 / std::abs(h);
        s_[0].b0 = float(s_[0].b0 * norm);
        s_[0].b1 = float(s_[0].b1 * norm);
        s_[0].b2 = float(s_[0].b2 * norm);
        reset();
    }

    inline float process(float x)
    {
        x = s_[0].process(x);
        x = s_[1].process(x);
        return s_[2].process(x);
    }

    void process(const float *in, float *out, size_t n)
    {
        for (size_t i = 0; i < n; ++i) out[i] = process(in[i]);
    }

    void reset()
    {
        for (auto &b : s_) b.reset();
    }

    // Magnitude response of the digital filter in dB at frequency f (Hz).
    double digital_db(double f) const
    {
        const double w = 2.0 * 3.14159265358979323846 * f / fs_;
        std::complex<double> h = 1.0;
        for (const auto &b : s_) h *= b.response(w);
        return 20.0 * std::log10(std::abs(h));
    }

private:
    static double analog_db_raw_1k()
    {
        const double f2 = 1000.0 * 1000.0;
        const double num = kF4 * kF4 * f2 * f2;
        const double den = (f2 + kF1 * kF1) *
                           std::sqrt((f2 + kF2 * kF2) * (f2 + kF3 * kF3)) *
                           (f2 + kF4 * kF4);
        return 20.0 * std::log10(num / den);
    }

    // Bilinear transform of s^2 / ((s + wa)(s + wb)).
    static void set_hp2(Biquad &b, double K, double wa, double wb)
    {
        // Denominator: s^2 + (wa+wb) s + wa wb, with s = K(1-z^-1)/(1+z^-1),
        // multiplied through by (1+z^-1)^2.
        const double K2 = K * K;
        const double sum = wa + wb;
        const double prod = wa * wb;
        const double a0 = K2 + sum * K + prod;
        const double a1 = 2.0 * (prod - K2);
        const double a2 = K2 - sum * K + prod;
        // Numerator: K^2 (1 - z^-1)^2
        b.b0 = float(K2 / a0);
        b.b1 = float(-2.0 * K2 / a0);
        b.b2 = float(K2 / a0);
        b.a1 = float(a1 / a0);
        b.a2 = float(a2 / a0);
    }

    // Find c in (-1, 1] so that the DC-normalised response of
    // (1 + c z^-1) / (1 - p z^-1)^2 matches `target` (power ratio) at w.
    static double fit_zero(double p, double w, double target)
    {
        auto power = [&](double c) {
            const std::complex<double> z1 = std::polar(1.0, -w);
            const std::complex<double> den = (1.0 - p * z1) * (1.0 - p * z1);
            const std::complex<double> h = (1.0 + c * z1) / den;
            const double dc = (1.0 + c) / ((1.0 - p) * (1.0 - p));
            const double m = std::abs(h) / dc;
            return m * m;
        };
        // power(c) is monotonically decreasing in c on [-0.99, 1]; bisect.
        double lo = -0.99, hi = 1.0;
        for (int i = 0; i < 100; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (power(mid) > target) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    double fs_ = 48000.0;
    double c_ = 0.0;
    Biquad s_[3];
};

}  // namespace slm
