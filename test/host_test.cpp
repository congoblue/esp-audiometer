// Host-side checks for the A-weighting filter and level maths.
//
// Build and run (any C++17 compiler):
//   clang++ -std=c++17 -O2 -I../main host_test.cpp -o host_test && ./host_test
//
// 1. Compares the digital filter's magnitude response with the analog
//    IEC 61672-1 A-weighting curve at the third-octave frequencies and checks
//    the filter's own error against the class 1 tolerance limits.
// 2. Runs sine waves through the float implementation to check that the
//    measured RMS gain matches the designed response (catches precision
//    problems with the low-frequency poles).

#include <cmath>
#include <cstdio>
#include <vector>

#include "a_weighting.hpp"
#include "level_meter.hpp"

struct Tol { double f, plus, minus; };

// IEC 61672-1:2013 class 1 acceptance limits (dB).  Large negative limits are
// used where the standard allows -infinity.
static const Tol kClass1[] = {
    {10, 3.5, -99}, {12.5, 3.0, -99}, {16, 2.5, -4.5}, {20, 2.5, -2.5},
    {25, 2.5, -2.0}, {31.5, 2.0, -1.5}, {40, 1.5, -1.5}, {50, 1.5, -1.5},
    {63, 1.5, -1.5}, {80, 1.5, -1.5}, {100, 1.5, -1.5}, {125, 1.5, -1.5},
    {160, 1.5, -1.5}, {200, 1.5, -1.5}, {250, 1.4, -1.4}, {315, 1.4, -1.4},
    {400, 1.4, -1.4}, {500, 1.4, -1.4}, {630, 1.4, -1.4}, {800, 1.4, -1.4},
    {1000, 1.1, -1.1}, {1250, 1.4, -1.4}, {1600, 1.6, -1.6}, {2000, 1.6, -1.6},
    {2500, 1.6, -1.6}, {3150, 1.6, -1.6}, {4000, 1.6, -1.6}, {5000, 2.1, -2.1},
    {6300, 2.1, -2.6}, {8000, 2.1, -3.1}, {10000, 2.6, -3.6}, {12500, 3.0, -6.0},
    {16000, 3.5, -17.0}, {20000, 4.0, -99},
};

static int check_response(double fs)
{
    slm::AWeighting a(fs);
    int failures = 0;
    double worst_to_10k = 0;
    std::printf("\nfs = %.0f Hz\n%8s %9s %9s %7s\n", fs, "f (Hz)", "analog", "digital", "error");
    for (const Tol &t : kClass1) {
        if (t.f >= fs / 2) continue;
        const double ref = slm::AWeighting::analog_db(t.f);
        const double dig = a.digital_db(t.f);
        const double err = dig - ref;
        const bool ok = err <= t.plus && err >= t.minus;
        if (t.f <= 10000) worst_to_10k = std::fmax(worst_to_10k, std::fabs(err));
        std::printf("%8.1f %9.2f %9.2f %+7.2f %s\n", t.f, ref, dig, err, ok ? "" : "  << FAIL");
        failures += !ok;
    }
    std::printf("worst |error| 10 Hz..10 kHz: %.2f dB\n", worst_to_10k);
    if (worst_to_10k > 0.5) {
        std::printf("FAIL: filter error above 0.5 dB below 10 kHz\n");
        ++failures;
    }
    return failures;
}

// Feed a sine through the float filter and compare RMS gain with design.
static int check_sine(double fs, double f, double amplitude)
{
    slm::AWeighting a(fs);
    const int n = int(fs * 4);       // 4 s
    const int settle = int(fs * 2);  // ignore first 2 s (20 Hz pole settles slowly)
    double acc = 0;
    for (int i = 0; i < n; ++i) {
        const float x = float(amplitude * std::sin(2 * M_PI * f * i / fs));
        const float y = a.process(x);
        if (i >= settle) acc += double(y) * y;
    }
    const double rms = std::sqrt(acc / (n - settle));
    const double gain_db = 20 * std::log10(rms / (amplitude / std::sqrt(2.0)));
    const double expect = a.digital_db(f);
    const double err = gain_db - expect;
    const bool ok = std::fabs(err) < 0.05;
    std::printf("sine %7.1f Hz amp %.0e: gain %+7.2f dB, design %+7.2f dB, err %+.3f %s\n",
                f, amplitude, gain_db, expect, err, ok ? "" : "<< FAIL");
    return !ok;
}

// Exponential time weighting: a 1 kHz tone switched on should reach the
// steady state and a step down should decay at 34.7 dB/s (F) / 4.3 dB/s (S).
static int check_time_weighting(double fs)
{
    slm::LevelMeter m(fs);
    const int blk = 480;
    std::vector<float> l(blk), r(blk);
    const double amp = 0.1;  // -20 dBFS peak, -23 dBFS rms (sine)
    int i = 0;
    for (int b = 0; b < 1000; ++b) {  // 10 s of tone so Slow fully settles
        for (int k = 0; k < blk; ++k, ++i) {
            l[k] = r[k] = float(amp * std::sin(2 * M_PI * 1000.0 * i / fs));
        }
        m.process(l.data(), r.data(), blk);
    }
    slm::Levels s = m.snapshot();
    const double expect = 20 * std::log10(amp / std::sqrt(2.0));
    std::printf("\ntone: LAF %.2f LAS %.2f LAeq %.2f Lmax %.2f (expect %.2f dBFS)\n",
                s.laf, s.las, s.laeq, s.lafmax, expect);
    int fails = 0;
    if (std::fabs(s.laf - expect) > 0.1 || std::fabs(s.las - expect) > 0.1 ||
        std::fabs(s.laeq - expect) > 0.1 || std::fabs(s.lafmax - expect) > 0.2) {
        std::printf("FAIL: steady-state level\n");
        ++fails;
    }
    // Now 1 s of silence: Fast should drop ~34.7 dB, Slow ~4.3 dB.
    for (int k = 0; k < blk; ++k) l[k] = r[k] = 0;
    for (int b = 0; b < 100; ++b) m.process(l.data(), r.data(), blk);
    s = m.snapshot();
    const double dF = expect - s.laf, dS = expect - s.las;
    std::printf("after 1 s silence: Fast dropped %.1f dB (expect 34.7), Slow %.1f dB (expect 4.3)\n", dF, dS);
    if (std::fabs(dF - 34.7) > 1.0 || std::fabs(dS - 4.3) > 0.3) {
        std::printf("FAIL: decay rate\n");
        ++fails;
    }
    return fails;
}

int main()
{
    int failures = 0;
    failures += check_response(48000);
    failures += check_response(44100);
    std::printf("\n");
    for (double f : {31.5, 100.0, 1000.0, 4000.0, 10000.0}) {
        failures += check_sine(48000, f, 0.5);
        failures += check_sine(48000, f, 1e-4);
    }
    failures += check_time_weighting(48000);
    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
