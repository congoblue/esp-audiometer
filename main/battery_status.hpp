// Battery state from the AXP2101 PMU, and how it maps to the on-screen icon.
//
// Register meanings follow XPowersLib (used by Waveshare's 01_AXP2101 example):
//   0x00 STATUS1  bit3 = battery connected, bit5 = VBUS good
//   0x01 STATUS2  bits[7:5] = 001 charging, 010 discharging, 000 standby
//   0xA4          fuel-gauge state of charge, 0..100 %
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cstdint>

namespace slm {

struct BatteryStatus {
    bool valid = false;     // PMU was read successfully
    bool present = false;   // battery connected
    bool charging = false;
    bool usb_power = false; // VBUS present
    int percent = 0;        // 0..100, meaningful only when present
};

inline BatteryStatus decode_axp2101(uint8_t status1, uint8_t status2, uint8_t percent_reg)
{
    BatteryStatus s;
    s.valid = true;
    s.present = (status1 >> 3) & 1;
    s.usb_power = (status1 >> 5) & 1;
    s.charging = s.present && (status2 >> 5) == 0x01;
    s.percent = percent_reg > 100 ? 100 : percent_reg;
    return s;
}

enum class BatteryLevel { Ok, Low, Critical };

// Green above 20 %, yellow from 10 % to 20 %, red below 10 %.
inline BatteryLevel battery_level(int percent)
{
    if (percent > 20) return BatteryLevel::Ok;
    if (percent >= 10) return BatteryLevel::Low;
    return BatteryLevel::Critical;
}

// Width of the fill bar for an inner width of `inner_w` pixels. Any non-zero
// charge shows at least one pixel so a nearly flat battery is still visible.
inline int32_t battery_fill_width(int percent, int32_t inner_w)
{
    if (percent <= 0) return 0;
    if (percent >= 100) return inner_w;
    const int32_t w = (percent * inner_w + 50) / 100;
    return w < 1 ? 1 : w;
}

// ---------------------------------------------------------------------------
// PWR key (AXP2101 PWRON) -> hold-to-power-off.
//
// IRQ status register 0x49 bits (XPowersLib):
//   bit0 positive edge (key released)   bit1 negative edge (key pressed)
//   bit2 long press (held for the 0x27 IRQ time, set to 2 s)
//   bit3 short press
// ---------------------------------------------------------------------------
enum : uint8_t { PKEY_RELEASE = 0x01, PKEY_PRESS = 0x02, PKEY_LONG = 0x04, PKEY_SHORT = 0x08 };

enum class PowerKey { Idle, Held, PowerOff };

// A long press only counts if the press itself was seen after start-up, so
// keeping PWR held after switching the meter on does not switch it off again.
class PowerKeyTracker {
public:
    PowerKey update(uint8_t irq_status)
    {
        if (state_ == PowerKey::PowerOff) return state_;
        if (irq_status & PKEY_PRESS) armed_ = true;
        if ((irq_status & PKEY_LONG) && armed_) {
            state_ = PowerKey::PowerOff;
        } else if (irq_status & PKEY_RELEASE) {
            // A press and release in the same poll is a tap: back to idle.
            armed_ = false;
            state_ = PowerKey::Idle;
        } else if (irq_status & PKEY_PRESS) {
            state_ = PowerKey::Held;
        }
        return state_;
    }

    PowerKey state() const { return state_; }

private:
    bool armed_ = false;
    PowerKey state_ = PowerKey::Idle;
};

}  // namespace slm
