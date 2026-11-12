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

}  // namespace slm
