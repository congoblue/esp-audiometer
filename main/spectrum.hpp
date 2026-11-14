// Spectrum analyser: Hann-windowed FFT, power summed into log-spaced bands.
//
// Band powers are true mean squares: a sine of amplitude A whose energy falls
// in one band reads A^2 / 2 there, and the bands together add up to the
// signal's mean square. That lets the display use the same dBFS -> dB SPL
// calibration as the sound level meter.
//
//   window w = Hann, sum(w^2) = 3N/8
//   one-sided mean square in bin k (0 < k < N/2) = 2 |X_k|^2 / (N * sum(w^2))
//                                                 = 16 |X_k|^2 / (3 N^2)
//
// Bands are spaced evenly on a log scale from f_lo to f_hi. Low bands can be
// narrower than one FFT bin; those use the bin nearest their centre, so a few
// neighbouring bars at the bottom end may show the same value.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace slm {

// In-place iterative radix-2 complex FFT of a fixed power-of-two size.
class Fft {
public:
    explicit Fft(size_t n) : n_(n), cos_(n / 2), sin_(n / 2)
    {
        const double pi = 3.14159265358979323846;
        for (size_t k = 0; k < n / 2; ++k) {
            cos_[k] = float(std::cos(2 * pi * double(k) / double(n)));
            sin_[k] = float(-std::sin(2 * pi * double(k) / double(n)));
        }
    }

    size_t size() const { return n_; }

    void run(float *re, float *im) const
    {
        const size_t n = n_;
        // Bit-reversal permutation.
        for (size_t i = 1, j = 0; i < n; ++i) {
            size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                std::swap(re[i], re[j]);
                std::swap(im[i], im[j]);
            }
        }
        // Butterflies.
        for (size_t len = 2; len <= n; len <<= 1) {
            const size_t half = len >> 1, step = n / len;
            for (size_t i = 0; i < n; i += len) {
                for (size_t k = 0; k < half; ++k) {
                    const float wr = cos_[k * step], wi = sin_[k * step];
                    float &ar = re[i + k], &ai = im[i + k];
                    float &br = re[i + k + half], &bi = im[i + k + half];
                    const float tr = br * wr - bi * wi;
                    const float ti = br * wi + bi * wr;
                    br = ar - tr;
                    bi = ai - ti;
                    ar += tr;
                    ai += ti;
                }
            }
        }
    }

private:
    size_t n_;
    std::vector<float> cos_, sin_;
};

class SpectrumAnalyzer {
public:
    static constexpr size_t kN = 4096;  // 85 ms at 48 kHz, 11.7 Hz bins
    static constexpr int kBands = 64;

    SpectrumAnalyzer(double fs, double f_lo = 50.0, double f_hi = 20000.0)
        : fft_(kN), window_(kN), re_(kN), im_(kN)
    {
        const double pi = 3.14159265358979323846;
        for (size_t i = 0; i < kN; ++i) window_[i] = float(0.5 - 0.5 * std::cos(2 * pi * double(i) / double(kN)));

        const double df = fs / double(kN);
        df_ = df;
        const double ratio = std::pow(f_hi / f_lo, 1.0 / kBands);
        for (int b = 0; b < kBands; ++b) {
            const double lo = f_lo * std::pow(ratio, b), hi = lo * ratio;
            centre_[b] = float(std::sqrt(lo * hi));
            // Bins whose centre frequency lies in [lo, hi).
            size_t k0 = size_t(std::ceil(lo / df)), k1 = size_t(std::ceil(hi / df));
            if (k1 <= k0) {  // band narrower than a bin: nearest bin to its centre
                k0 = size_t(std::lround(centre_[b] / df));
                k1 = k0 + 1;
            }
            const size_t nyq = kN / 2;
            if (k0 < 1) k0 = 1;
            if (k1 > nyq) k1 = nyq;
            if (k0 >= k1) k0 = k1 - 1;
            k_lo_[b] = k0;
            k_hi_[b] = k1;
        }
    }

    // x: the kN most recent samples, oldest first. out: mean square per band.
    void analyse(const float *x, float *out)
    {
        for (size_t i = 0; i < kN; ++i) {
            re_[i] = x[i] * window_[i];
            im_[i] = 0.0f;
        }
        fft_.run(re_.data(), im_.data());
        const double scale = 16.0 / (3.0 * double(kN) * double(kN));
        for (int b = 0; b < kBands; ++b) {
            double sum = 0;
            for (size_t k = k_lo_[b]; k < k_hi_[b]; ++k) sum += double(re_[k]) * re_[k] + double(im_[k]) * im_[k];
            out[b] = float(sum * scale);
        }
    }

    // Diagnostics, valid after analyse(): the loudest FFT bin in bands
    // [b_first, b_last] and how far it stands above the mean bin power there.
    // A tone gives a large ratio (15 dB or more); noise stays within a few dB.
    struct Peak {
        float freq_hz;
        float ratio_db;
    };
    Peak loudest_bin(int b_first, int b_last) const
    {
        const size_t k0 = k_lo_[b_first], k1 = k_hi_[b_last];
        size_t best = k0;
        double best_p = -1, sum = 0;
        for (size_t k = k0; k < k1; ++k) {
            const double p = double(re_[k]) * re_[k] + double(im_[k]) * im_[k];
            sum += p;
            if (p > best_p) {
                best_p = p;
                best = k;
            }
        }
        const double mean = sum / double(k1 - k0);
        return {float(double(best) * df_), float(10.0 * std::log10((best_p + 1e-30) / (mean + 1e-30)))};
    }

    // Width of a band in Hz (as summed: whole bins).
    float band_width_hz(int b) const { return float(double(k_hi_[b] - k_lo_[b]) * df_); }

    float band_centre(int b) const { return centre_[b]; }
    size_t band_first_bin(int b) const { return k_lo_[b]; }
    size_t band_end_bin(int b) const { return k_hi_[b]; }

    // Gives the FFT direct access for testing.
    const Fft &fft() const { return fft_; }

private:
    double df_ = 0;
    Fft fft_;
    std::vector<float> window_, re_, im_;
    float centre_[kBands];
    size_t k_lo_[kBands], k_hi_[kBands];
};

}  // namespace slm
