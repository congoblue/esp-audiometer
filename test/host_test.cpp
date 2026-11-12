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
#include "battery_status.hpp"
#include "level_meter.hpp"
#include "orientation.hpp"
#include "seg7_layout.hpp"

#include <string>

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

// Orientation detector: hold time, dead band, flat-on-table behaviour, and
// touch remapping that is the inverse of the display rotation.
static int check_orientation()
{
    int fails = 0;
    auto expect = [&](const char *what, int got, int want) {
        const bool ok = got == want;
        std::printf("orientation: %-44s -> %d (want %d)%s\n", what, got, want, ok ? "" : "  << FAIL");
        fails += !ok;
    };
    auto feed = [](slm::OrientationDetector &d, float deg, float g, int ms) {
        const float r = deg * float(M_PI) / 180.0f;
        int o = d.orientation();
        for (int t = 0; t < ms; t += 50) o = d.update(g * std::sin(r), g * std::cos(r), 50);
        return o;
    };

    slm::OrientationDetector d;
    expect("upright", feed(d, 0, 1.0f, 1000), 0);
    expect("right edge up, only 200 ms (debounce)", feed(d, 90, 1.0f, 200), 0);
    expect("right edge up, held", feed(d, 90, 1.0f, 600), 1);
    expect("bottom edge up", feed(d, 180, 1.0f, 1000), 2);
    expect("left edge up", feed(d, 270, 1.0f, 1000), 3);
    expect("45 deg between left and top (dead band)", feed(d, 315, 1.0f, 1000), 3);
    expect("25 deg from top (inside capture)", feed(d, 335, 1.0f, 1000), 0);
    expect("flat on table, tilt 0.2 g towards right", feed(d, 90, 0.2f, 2000), 0);
    expect("brief wobble to 90 then back", (feed(d, 90, 1.0f, 300), feed(d, 0, 1.0f, 600)), 0);

    // Power-up: nearest orientation immediately, normal (0) when flat.
    auto at = [](float deg, float g) {
        const float r = deg * float(M_PI) / 180.0f;
        return slm::OrientationDetector::initial(g * std::sin(r), g * std::cos(r));
    };
    expect("power-up upright", at(0, 1.0f), 0);
    expect("power-up right edge up", at(90, 1.0f), 1);
    expect("power-up upside down", at(180, 1.0f), 2);
    expect("power-up left edge up, tilted 40 deg off", at(310, 1.0f), 3);
    expect("power-up flat (0.1 g in plane)", at(90, 0.1f), 0);
    slm::OrientationDetector d2(at(180, 1.0f));
    expect("detector seeded upside down stays there", feed(d2, 180, 1.0f, 1000), 2);

    // Touch: for every rotation, the physical point where a logical point is
    // drawn must map back to that logical point.
    const int32_t n = 479;
    for (int q = 0; q < 4; ++q) {
        for (int32_t lx : {0, 10, 240, 479}) {
            for (int32_t ly : {0, 33, 400, 479}) {
                int32_t px, py;  // logical -> physical for a clockwise rotation
                switch (q) {
                case 1: px = n - ly; py = lx; break;
                case 2: px = n - lx; py = n - ly; break;
                case 3: px = ly; py = n - lx; break;
                default: px = lx; py = ly; break;
                }
                int32_t x = px, y = py;
                slm::rotate_touch_point(q, n, x, y);
                if (x != lx || y != ly) {
                    std::printf("touch q=%d (%d,%d) -> (%d,%d)  << FAIL\n", q, lx, ly, x, y);
                    ++fails;
                }
            }
        }
    }
    std::printf("touch remap round-trip: %s\n", fails ? "see failures" : "ok");
    return fails;
}

// Seven-segment layout: segment counts, fixed width, no overlaps, in bounds,
// plus an ASCII preview of the digit shapes.
static int check_seg7()
{
    int fails = 0;
    slm::Seg7Geometry g;  // the geometry used on the device

    const int expect_lit[10] = {6, 2, 5, 5, 4, 5, 6, 3, 7, 6};
    for (int d = 0; d < 10; ++d) {
        const char s[2] = {char('0' + d), 0};
        int lit = 0, total = 0;
        slm::seg7_layout(s, g, [&](const slm::Seg7Segment &r) { lit += r.lit; ++total; });
        if (lit != expect_lit[d] || total != 7) {
            std::printf("seg7 '%c': %d lit of %d  << FAIL\n", s[0], lit, total);
            ++fails;
        }
    }

    // Narrow hundreds cell: two segments, lit for 'I', unlit for 'i'.
    for (const char *s : {"I", "i"}) {
        int lit = 0, total = 0;
        slm::seg7_layout(s, g, [&](const slm::Seg7Segment &r) { lit += r.lit; ++total; });
        const int want_lit = s[0] == 'I' ? 2 : 0;
        if (lit != want_lit || total != 2 || slm::seg7_width(s, g) != g.thick) {
            std::printf("seg7 '%s': %d lit of %d, width %d  << FAIL\n", s, lit, total, slm::seg7_width(s, g));
            ++fails;
        }
    }

    // Fixed-width formatting keeps the width constant (main.cpp's fmt_seg7).
    const char *samples[] = {"I88.8", "i54.3", "I05.2", "i--.-", "i 0.0"};
    const int32_t w0 = slm::seg7_width(samples[0], g);
    for (const char *s : samples) {
        if (slm::seg7_width(s, g) != w0) {
            std::printf("seg7 width of \"%s\" differs  << FAIL\n", s);
            ++fails;
        }
    }

    // Rasterise "888.8": every pixel belongs to at most one segment, nothing
    // falls outside the widget, and each bar reaches its full thickness.
    std::vector<slm::Seg7Segment> segs;
    const int32_t w = slm::seg7_layout("I88.8", g, [&](const slm::Seg7Segment &r) { segs.push_back(r); });
    std::vector<int> owner(size_t(w) * g.digit_h, -1);
    for (size_t i = 0; i < segs.size(); ++i) {
        const auto &s = segs[i];
        if (s.x1 < 0 || s.y1 < 0 || s.x2 > w || s.y2 > g.digit_h || s.x1 >= s.x2 || s.y1 >= s.y2) {
            std::printf("seg7 segment %zu out of bounds  << FAIL\n", i);
            ++fails;
            continue;
        }
        int width_max = 0;
        for (int32_t y = s.y1; y < s.y2; ++y) {
            int row = 0;
            for (int32_t x = s.x1; x < s.x2; ++x) {
                if (!slm::seg7_contains(s, x, y)) continue;
                ++row;
                int &o = owner[size_t(y) * w + x];
                if (o >= 0) {
                    std::printf("seg7 segments %d and %zu overlap at (%d,%d)  << FAIL\n", o, i, x, y);
                    ++fails;
                }
                o = int(i);
            }
            width_max = row > width_max ? row : width_max;
        }
        if (s.shape == slm::Seg7Shape::Vertical && width_max != g.thick) {
            std::printf("seg7 segment %zu max thickness %d, want %d  << FAIL\n", i, width_max, g.thick);
            ++fails;
        }
    }
    std::printf("\nseg7: \"I88.8\" is %d x %d px (\"888.8\" would be %d), %zu segments, %s\n", w, g.digit_h,
                slm::seg7_width("888.8", g), segs.size(), fails ? "FAILED" : "ok");

    // ASCII preview at reduced size: '#' lit, '.' unlit.
    slm::Seg7Geometry s;
    s.digit_w = 16; s.digit_h = 25; s.thick = 4; s.gap = 1; s.spacing = 4; s.dp_w = 4; s.dp_spacing = 2;
    const char *txt = "I05.2";
    const int32_t pw = slm::seg7_width(txt, s);
    std::vector<std::string> canvas(s.digit_h, std::string(pw, ' '));
    slm::seg7_layout(txt, s, [&](const slm::Seg7Segment &r) {
        for (int32_t y = r.y1; y < r.y2; ++y)
            for (int32_t x = r.x1; x < r.x2; ++x)
                if (slm::seg7_contains(r, x, y)) canvas[y][x] = r.lit ? '#' : '.';
    });
    std::printf("preview \"%s\":\n", txt);
    for (const auto &row : canvas) std::printf("  %s\n", row.c_str());
    return fails;
}

// Battery: AXP2101 register decoding, colour thresholds and fill width.
static int check_battery()
{
    int fails = 0;
    auto check = [&](const char *what, bool ok) {
        if (!ok) {
            std::printf("battery: %s  << FAIL\n", what);
            ++fails;
        }
    };
    using slm::BatteryLevel;
    // STATUS1 bit3 = battery, bit5 = VBUS; STATUS2 bits[7:5] = 001 charging.
    auto s = slm::decode_axp2101(0x28, 0x20, 57);
    check("present+charging", s.valid && s.present && s.charging && s.usb_power && s.percent == 57);
    s = slm::decode_axp2101(0x08, 0x40, 80);
    check("discharging", s.present && !s.charging && !s.usb_power);
    s = slm::decode_axp2101(0x20, 0x20, 0);
    check("no battery is never 'charging'", !s.present && !s.charging && s.usb_power);
    s = slm::decode_axp2101(0x08, 0x00, 0xFF);
    check("gauge value clamped to 100", s.percent == 100);
    s = slm::decode_axp2101(0x08, 0xA0, 50);  // bits[7:5] = 101: not charging
    check("only 001 counts as charging", !s.charging);

    check("21% green", slm::battery_level(21) == BatteryLevel::Ok);
    check("20% yellow", slm::battery_level(20) == BatteryLevel::Low);
    check("10% yellow", slm::battery_level(10) == BatteryLevel::Low);
    check("9% red", slm::battery_level(9) == BatteryLevel::Critical);
    check("0% red", slm::battery_level(0) == BatteryLevel::Critical);

    check("fill 0%", slm::battery_fill_width(0, 34) == 0);
    check("fill 1% shows a sliver", slm::battery_fill_width(1, 34) == 1);
    check("fill 50%", slm::battery_fill_width(50, 34) == 17);
    check("fill 100%", slm::battery_fill_width(100, 34) == 34);
    std::printf("battery: %s\n", fails ? "FAILED" : "ok");
    return fails;
}

int main()
{
    int failures = 0;
    failures += check_battery();
    failures += check_seg7();
    failures += check_orientation();
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
