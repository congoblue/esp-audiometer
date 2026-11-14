// Steep low-pass filter (inverse Chebyshev / Chebyshev type II).
//
// Why: this board's microphone signal picks up electrical interference from
// its own electronics at about 14.8 kHz (with more near 15.8 and 17.4 kHz).
// Covering the mic ports does not change it, so it is not sound. At 14.8 kHz
// A-weighting only removes ~6 dB, so without this filter the interference
// adds ~33 dB(A) and dominates quiet-room readings.
//
// Chebyshev type II: flat (monotonic) passband and an equiripple stopband
// that never rises above -stop_db. With the defaults (10th order, stopband
// from 14.5 kHz at -50 dB, fs = 48 kHz) the passband loses about 0.4 dB at
// 12 kHz and is essentially flat below 10 kHz (see test/host_test.cpp).
//
// Design (analog prototype, then bilinear transform with pre-warping):
//   theta_k = pi (2k - 1) / (2N),  k = 1..N
//   eps     = 1 / sqrt(10^(stop_db/10) - 1)
//   mu      = asinh(1/eps) / N
//   Chebyshev-I prototype pole  q_k = -sinh(mu) sin(theta_k) + j cosh(mu) cos(theta_k)
//   type-II pole                p_k = Ws / q_k
//   type-II zero pair           +-j Ws / cos(theta_k)
// Each conjugate pole pair and zero pair forms one biquad, scaled to unity
// gain at DC.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cmath>
#include <complex>
#include <cstddef>

#include "a_weighting.hpp"  // slm::Biquad

namespace slm {

class ChebyLowpass {
public:
    static constexpr int kOrder = 10;  // must be even
    static constexpr int kSections = kOrder / 2;

    explicit ChebyLowpass(double fs = 48000.0, double f_stop = 14500.0, double stop_db = 50.0)
    {
        design(fs, f_stop, stop_db);
    }

    // False when the stopband edge is too close to Nyquist to design (low
    // sample rates); process() then passes the signal through unchanged.
    bool enabled() const { return enabled_; }

    void design(double fs, double f_stop, double stop_db)
    {
        fs_ = fs;
        enabled_ = f_stop < 0.45 * fs;
        if (!enabled_) return;
        const double pi = 3.14159265358979323846;
        const double K = 2.0 * fs;
        const double ws = K * std::tan(pi * f_stop / fs);  // pre-warped stopband edge
        const double eps = 1.0 / std::sqrt(std::pow(10.0, stop_db / 10.0) - 1.0);
        const double mu = std::asinh(1.0 / eps) / kOrder;

        for (int k = 1; k <= kSections; ++k) {
            const double theta = pi * (2 * k - 1) / (2.0 * kOrder);
            const std::complex<double> q(-std::sinh(mu) * std::sin(theta), std::cosh(mu) * std::cos(theta));
            const std::complex<double> p = ws / q;  // analog pole (upper half plane)
            const double wz = ws / std::cos(theta); // analog zero at +-j wz

            // Section: (s^2 + wz^2) / (s^2 + a s + c), a = -2 Re(p), c = |p|^2
            const double a = -2.0 * p.real(), c = std::norm(p);
            const double K2 = K * K, wz2 = wz * wz;
            const double a0 = K2 + a * K + c;
            const double a1 = 2.0 * (c - K2);
            const double a2 = K2 - a * K + c;
            const double g = c / wz2;  // unity gain at DC
            Biquad &b = s_[k - 1];
            b.b0 = float(g * (K2 + wz2) / a0);
            b.b1 = float(g * 2.0 * (wz2 - K2) / a0);
            b.b2 = float(g * (K2 + wz2) / a0);
            b.a1 = float(a1 / a0);
            b.a2 = float(a2 / a0);
            b.reset();
        }
    }

    inline float process(float x)
    {
        if (!enabled_) return x;
        for (auto &s : s_) x = s.process(x);
        return x;
    }

    void reset()
    {
        for (auto &s : s_) s.reset();
    }

    // Magnitude response in dB at frequency f (Hz).
    double response_db(double f) const
    {
        if (!enabled_) return 0.0;
        const double w = 2.0 * 3.14159265358979323846 * f / fs_;
        std::complex<double> h = 1.0;
        for (const auto &s : s_) h *= s.response(w);
        return 20.0 * std::log10(std::abs(h) + 1e-300);
    }

private:
    double fs_ = 48000.0;
    bool enabled_ = false;
    Biquad s_[kSections];
};

}  // namespace slm
