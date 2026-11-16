#include "ppm_view.hpp"

#include <cmath>

namespace {

// Radii from the centre, for a 440 px dial (scaled for other sizes).
constexpr float kScaleR = 200.0f / 220.0f;    // scale arc
constexpr float kTickR = 222.0f / 220.0f;     // outer end of major ticks
constexpr float kMinorR = 211.0f / 220.0f;    // outer end of half-mark ticks
constexpr float kLabelR = 178.0f / 220.0f;    // centre of the labels
constexpr float kNeedleIn = 140.0f / 220.0f;  // needle inner end
constexpr float kNeedleOut = 224.0f / 220.0f; // needle outer end

constexpr float kStartDeg = 135.0f;  // position 0 (rest), bottom left
constexpr float kSweepDeg = 270.0f;  // to position 8 (end stop), bottom right

const lv_color_t kWhite = lv_color_hex(0xF2F2F2);
const lv_color_t kGrey = lv_color_hex(0x5A5A5A);
const lv_color_t kRed = lv_color_hex(0xE74C3C);

struct PpmState {
    float pos = 0;
    int32_t drawn_px = -1;  // needle angle last shown, in 0.25 degree steps
    bool ebu = false;
};

float angle_deg(float pos) { return kStartDeg + pos / 8.0f * kSweepDeg; }

lv_point_precise_t polar(const lv_area_t &c, float r, float deg)
{
    const float a = deg * 3.14159265f / 180.0f;
    const float cx = (c.x1 + c.x2) / 2.0f, cy = (c.y1 + c.y2) / 2.0f;
    return {(lv_value_precise_t)(cx + r * cosf(a)), (lv_value_precise_t)(cy + r * sinf(a))};
}

void line(lv_layer_t *layer, lv_point_precise_t p1, lv_point_precise_t p2, int32_t w, lv_color_t col, bool round)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.p1 = p1;
    d.p2 = p2;
    d.width = w;
    d.color = col;
    d.round_start = round;
    d.round_end = round;
    lv_draw_line(layer, &d);
}

void arc(lv_layer_t *layer, const lv_area_t &c, float r, int32_t w, float from_pos, float to_pos, lv_color_t col)
{
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.center = {(c.x1 + c.x2) / 2, (c.y1 + c.y2) / 2};
    d.radius = (uint16_t)(r + w / 2);  // radius is the outer edge
    d.width = w;
    d.color = col;
    d.start_angle = fmodf(angle_deg(from_pos), 360.0f);
    d.end_angle = fmodf(angle_deg(to_pos), 360.0f);
    lv_draw_arc(layer, &d);
}

void label(lv_layer_t *layer, lv_point_precise_t centre, const char *text, const lv_font_t *font, lv_color_t col)
{
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = font;
    d.color = col;
    d.align = LV_TEXT_ALIGN_CENTER;
    d.text = text;
    d.text_local = 1;
    const int32_t h = lv_font_get_line_height(font);
    const int32_t x = (int32_t)centre.x, y = (int32_t)centre.y;
    const lv_area_t a = {x - 30, y - h / 2, x + 30, y - h / 2 + h - 1};
    lv_draw_label(layer, &d, &a);
}

void draw_cb(lv_event_t *e)
{
    const auto *st = static_cast<PpmState *>(lv_event_get_user_data(e));
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(lv_event_get_target_obj(e), &c);
    const float half = lv_area_get_width(&c) / 2.0f;

    // Scale arc: grey up to mark 6, red above.
    arc(layer, c, half * kScaleR, 3, 1.0f, 6.0f, kGrey);
    arc(layer, c, half * kScaleR, 6, 6.0f, 7.5f, kRed);

    static const char *const kBbc[] = {"1", "2", "3", "4", "5", "6", "7"};
    static const char *const kEbu[] = {"-12", "-8", "-4", "TEST", "+4", "+8", "+12"};
    for (int m = 1; m <= 7; ++m) {
        const float deg = angle_deg((float)m);
        const lv_color_t col = m == 7 ? kRed : kWhite;
        line(layer, polar(c, half * kScaleR, deg), polar(c, half * kTickR, deg), 4, col, false);
        if (m < 7) {
            const float mid = angle_deg(m + 0.5f);
            line(layer, polar(c, half * kScaleR, mid), polar(c, half * kMinorR, mid), 2, m >= 6 ? kRed : kWhite, false);
        }
        if (st->ebu) label(layer, polar(c, half * kLabelR, deg), kEbu[m - 1], &lv_font_montserrat_16, col);
        else label(layer, polar(c, half * kLabelR, deg), kBbc[m - 1], &lv_font_montserrat_24, col);
    }

    // Needle.
    const float deg = angle_deg(st->pos);
    line(layer, polar(c, half * kNeedleIn, deg), polar(c, half * kNeedleOut, deg), 6, kWhite, true);
}

void delete_cb(lv_event_t *e) { delete static_cast<PpmState *>(lv_event_get_user_data(e)); }

}  // namespace

lv_obj_t *ppm_view_create(lv_obj_t *parent, int32_t size)
{
    auto *st = new PpmState;
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(obj, size, size);
    // Drawn slightly outside its box (needle tip, tick ends).
    lv_obj_add_event_cb(obj, [](lv_event_t *e) { lv_event_set_ext_draw_size(e, 8); },
                        LV_EVENT_REFR_EXT_DRAW_SIZE, nullptr);
    lv_obj_refresh_ext_draw_size(obj);
    lv_obj_set_user_data(obj, st);
    lv_obj_add_event_cb(obj, draw_cb, LV_EVENT_DRAW_MAIN, st);
    lv_obj_add_event_cb(obj, delete_cb, LV_EVENT_DELETE, st);
    return obj;
}

namespace {
// Marks the screen area covered by the needle at `pos` for redrawing.
void invalidate_needle(lv_obj_t *obj, float pos)
{
    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    const float half = lv_area_get_width(&c) / 2.0f;
    const lv_point_precise_t a = polar(c, half * kNeedleIn, angle_deg(pos));
    const lv_point_precise_t b = polar(c, half * kNeedleOut, angle_deg(pos));
    constexpr int32_t m = 5;  // half the needle width, plus antialiasing
    lv_area_t r = {(int32_t)fminf(a.x, b.x) - m, (int32_t)fminf(a.y, b.y) - m,
                   (int32_t)fmaxf(a.x, b.x) + m, (int32_t)fmaxf(a.y, b.y) + m};
    lv_obj_invalidate_area(obj, &r);
}
}  // namespace

void ppm_view_set(lv_obj_t *obj, float position)
{
    auto *st = static_cast<PpmState *>(lv_obj_get_user_data(obj));
    position = position < 0 ? 0 : (position > 8 ? 8 : position);
    const int32_t step = (int32_t)lroundf(angle_deg(position) * 4.0f);
    if (step == st->drawn_px) return;  // needle hasn't visibly moved
    st->drawn_px = step;
    invalidate_needle(obj, st->pos);   // where it was
    st->pos = position;
    invalidate_needle(obj, st->pos);   // where it is now
}

void ppm_view_set_ebu(lv_obj_t *obj, bool ebu)
{
    auto *st = static_cast<PpmState *>(lv_obj_get_user_data(obj));
    st->ebu = ebu;
    lv_obj_invalidate(obj);
}

bool ppm_view_is_ebu(lv_obj_t *obj) { return static_cast<PpmState *>(lv_obj_get_user_data(obj))->ebu; }
