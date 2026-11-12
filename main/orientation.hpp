// Screen orientation from an accelerometer, with hysteresis and debounce.
//
// Input is the acceleration measured in the *screen* frame: +x towards the
// physical right edge of the panel, +y towards the physical top edge, in g.
// At rest an accelerometer reads +1 g pointing away from the earth, so the
// in-plane part of the reading points to whichever edge is "up".
//
// Output is the number of quarter turns clockwise (0..3) that the UI must be
// rotated so its top edge faces up:
//   0 = physical top edge up        1 = physical right edge up
//   2 = physical bottom edge up     3 = physical left edge up
//
// A new orientation is accepted only when
//   * the device is not lying flat (in-plane gravity >= kMinTiltG), and
//   * the tilt is within kCaptureDeg of the new orientation's axis, so there
//     is a 2 * (45 - kCaptureDeg) degree dead band between orientations, and
//   * it has been held for kHoldMs.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cmath>
#include <cstdint>

namespace slm {

class OrientationDetector {
public:
    static constexpr float kMinTiltG = 0.5f;   // ~30 degrees from flat
    static constexpr float kCaptureDeg = 30.0f;
    static constexpr uint32_t kHoldMs = 400;

    explicit OrientationDetector(int initial = 0) : current_(initial & 3) {}

    // Feed one sample; returns the (possibly unchanged) orientation.
    int update(float ax, float ay, uint32_t dt_ms)
    {
        const float tilt = std::sqrt(ax * ax + ay * ay);
        int target = current_;
        if (tilt >= kMinTiltG) {
            // 0 deg = top edge up, increasing clockwise.
            float deg = std::atan2(ax, ay) * (180.0f / 3.14159265f);
            if (deg < 0) deg += 360.0f;
            const int q = int(std::lround(deg / 90.0f)) & 3;
            float off = std::fabs(deg - q * 90.0f);
            if (off > 180.0f) off = 360.0f - off;
            if (off <= kCaptureDeg) target = q;
        }

        if (target == current_) {
            pending_ = current_;
            held_ms_ = 0;
        } else if (target != pending_) {
            pending_ = target;
            held_ms_ = dt_ms;
        } else {
            held_ms_ += dt_ms;
            if (held_ms_ >= kHoldMs) {
                current_ = target;
                held_ms_ = 0;
            }
        }
        return current_;
    }

    int orientation() const { return current_; }

    // Orientation to use at power-up: the nearest quarter turn with no hold
    // time or dead band. Returns 0 (normal) if the device is lying flat.
    static int initial(float ax, float ay)
    {
        if (std::sqrt(ax * ax + ay * ay) < kMinInitialTiltG) return 0;
        float deg = std::atan2(ax, ay) * (180.0f / 3.14159265f);
        if (deg < 0) deg += 360.0f;
        return int(std::lround(deg / 90.0f)) & 3;
    }

    static constexpr float kMinInitialTiltG = 0.3f;

private:
    int current_;
    int pending_ = -1;
    uint32_t held_ms_ = 0;
};

// Map a touch point reported for the unrotated panel to UI coordinates when
// the UI is rotated `quarter_turns` clockwise on a square panel of side n+1.
inline void rotate_touch_point(int quarter_turns, int32_t n, int32_t &x, int32_t &y)
{
    const int32_t px = x, py = y;
    switch (quarter_turns & 3) {
    case 1: x = py;     y = n - px; break;
    case 2: x = n - px; y = n - py; break;
    case 3: x = n - py; y = px;     break;
    default: break;
    }
}

}  // namespace slm
