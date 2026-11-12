// Seven-segment display layout: turns a string such as " 54.3" into a list
// of segments (lit or unlit) for a renderer to draw.
//
// Segments are the classic pointed hexagons: each one is a bar of thickness
// `thick` whose ends taper to a point at 45 degrees, so neighbouring segments
// meet along a diagonal with `gap` pixels between them.
//
// Characters:
//   '0'-'9', '-'  digit cell
//   ' '           blank digit cell (all segments unlit)
//   '.'           decimal point (a square), placed after the previous digit
//   'I' / 'i'     narrow cell, one segment wide, that can only show "1":
//                 'I' lit, 'i' unlit. Used for a leading hundreds digit so
//                 the readout is not pushed off-centre by a full-width cell.
// Anything else is drawn as a blank digit.
//
// Every digit cell has the same width, so a fixed-width format such as
// "%5.1f" keeps the number still on screen as its value changes.
//
// No LVGL or ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cstdint>

namespace slm {

struct Seg7Geometry {
    int32_t digit_w = 54;    // width of one digit cell
    int32_t digit_h = 100;   // height of one digit cell
    int32_t thick = 12;      // segment thickness (even numbers look best)
    int32_t gap = 2;         // gap between neighbouring segments
    int32_t spacing = 16;    // space between digit cells
    int32_t dp_w = 12;       // decimal point size (square)
    int32_t dp_spacing = 7;  // space either side of the decimal point
};

enum class Seg7Shape : uint8_t { Horizontal, Vertical, Dot };

// A segment's bounding box, tip to tip. Coordinates are pixel edges relative
// to the widget's top-left: the segment covers x1 <= x < x2, y1 <= y < y2.
struct Seg7Segment {
    int32_t x1, y1, x2, y2;
    Seg7Shape shape;
    bool lit;
};

// True if the pixel centre (px + 0.5, py + 0.5) is inside the segment.
inline bool seg7_contains(const Seg7Segment &s, int32_t px, int32_t py)
{
    const float x = px + 0.5f, y = py + 0.5f;
    if (x < s.x1 || x > s.x2 || y < s.y1 || y > s.y2) return false;
    if (s.shape == Seg7Shape::Dot) return true;
    const bool horiz = s.shape == Seg7Shape::Horizontal;
    const float along_lo = horiz ? x - s.x1 : y - s.y1;  // distance from each tip
    const float along_hi = horiz ? s.x2 - x : s.y2 - y;
    const float across = horiz ? y - 0.5f * (s.y1 + s.y2) : x - 0.5f * (s.x1 + s.x2);
    const float a = across < 0 ? -across : across;
    return a <= along_lo && a <= along_hi;
}

// Segment bits: a=top, b=top-right, c=bottom-right, d=bottom, e=bottom-left,
// f=top-left, g=middle.
enum : uint8_t { SEG_A = 1, SEG_B = 2, SEG_C = 4, SEG_D = 8, SEG_E = 16, SEG_F = 32, SEG_G = 64 };

inline uint8_t seg7_mask(char c)
{
    static const uint8_t digits[10] = {
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // 0
        SEG_B | SEG_C,                                          // 1
        SEG_A | SEG_B | SEG_G | SEG_E | SEG_D,                  // 2
        SEG_A | SEG_B | SEG_G | SEG_C | SEG_D,                  // 3
        SEG_F | SEG_G | SEG_B | SEG_C,                          // 4
        SEG_A | SEG_F | SEG_G | SEG_C | SEG_D,                  // 5
        SEG_A | SEG_F | SEG_G | SEG_E | SEG_D | SEG_C,          // 6
        SEG_A | SEG_B | SEG_C,                                  // 7
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,  // 8
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,          // 9
    };
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c == '-') return SEG_G;
    return 0;
}

// Calls emit(const Seg7Segment &) for every segment of every digit (lit and
// unlit) and for each decimal point. Returns the total width.
template <class Emit>
int32_t seg7_layout(const char *text, const Seg7Geometry &g, Emit &&emit)
{
    const int32_t t = g.thick, w = g.digit_w, h = g.digit_h;
    const int32_t half = t / 2;
    // Neighbouring bars meet at a corner point on both centre lines. Pulling
    // each tip back by p along its bar leaves a diagonal gap of p * sqrt(2),
    // so p = gap / sqrt(2), rounded up so the gap is never smaller than asked.
    const int32_t pull = (g.gap * 7071 + 9999) / 10000;
    const int32_t mid = h / 2;  // centre line of the middle segment
    int32_t x = 0;
    bool first = true;

    for (const char *p = text; *p; ++p) {
        if (*p == '.') {
            const int32_t dx = x + g.dp_spacing;
            emit(Seg7Segment{dx, h - g.dp_w, dx + g.dp_w, h, Seg7Shape::Dot, true});
            x = dx + g.dp_w + g.dp_spacing;
            first = true;  // no extra digit spacing after a decimal point
            continue;
        }
        if (!first) x += g.spacing;
        first = false;

        if (*p == 'I' || *p == 'i') {  // narrow "1": segments b and c only
            const bool lit = *p == 'I';
            emit(Seg7Segment{x, half + pull, x + t, mid - pull, Seg7Shape::Vertical, lit});
            emit(Seg7Segment{x, mid + pull, x + t, h - t + half - pull, Seg7Shape::Vertical, lit});
            x += t;
            continue;
        }

        const uint8_t m = seg7_mask(*p);
        auto hseg = [&](uint8_t bit, int32_t yc) {
            emit(Seg7Segment{x + half + pull, yc - half, x + w - half - pull, yc - half + t,
                             Seg7Shape::Horizontal, (m & bit) != 0});
        };
        auto vseg = [&](uint8_t bit, int32_t xl, int32_t ytop, int32_t ybot) {
            emit(Seg7Segment{x + xl, ytop + pull, x + xl + t, ybot - pull,
                             Seg7Shape::Vertical, (m & bit) != 0});
        };
        // Bars run along the centre lines of a w x h box inset by half a bar,
        // so the outer edges of the digit touch the cell edges.
        hseg(SEG_A, half);
        hseg(SEG_G, mid);
        hseg(SEG_D, h - t + half);
        vseg(SEG_F, 0, half, mid);
        vseg(SEG_B, w - t, half, mid);
        vseg(SEG_E, 0, mid, h - t + half);
        vseg(SEG_C, w - t, mid, h - t + half);

        x += w;
    }
    return x;
}

// Width of the laid-out text without drawing anything.
inline int32_t seg7_width(const char *text, const Seg7Geometry &g)
{
    return seg7_layout(text, g, [](const Seg7Segment &) {});
}

}  // namespace slm
