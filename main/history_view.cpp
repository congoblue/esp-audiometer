#include "history_view.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr int32_t kAxisW = 34;        // label column on the left
constexpr int32_t kGap = 1;           // pixels between bars
constexpr int32_t kTopPad = 9;        // room for the top grid label
constexpr int32_t kBaseH = 2;         // baseline thickness
constexpr float kMinSpanDb = 30.0f;   // smallest vertical range
constexpr float kMinColourSpanDb = 6.0f;  // levels closer than this don't use the whole hue range

const lv_color_t kGrid = lv_color_hex(0x262626);
const lv_color_t kBase = lv_color_hex(0x404040);
const lv_color_t kLabel = lv_color_hex(0x8A8A8A);
const lv_color_t kLeqMark = lv_color_hex(0xA0A0A0);
constexpr int32_t kLeqH = 2;          // LAeq mark thickness

struct HistState {
    int bars;
    std::vector<float> db;   // oldest first
    std::vector<float> leq;  // LAeq 15m at each point
    int count = 0;
};

void fill(lv_layer_t *layer, lv_color_t c, int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = LV_OPA_COVER;
    const lv_area_t a = {x1, y1, x2, y2};
    lv_draw_rect(layer, &d, &a);
}

void label(lv_layer_t *layer, int32_t x1, int32_t x2, int32_t y_mid, const char *text)
{
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = &lv_font_montserrat_16;
    d.color = kLabel;
    d.align = LV_TEXT_ALIGN_RIGHT;
    d.text = text;
    d.text_local = 1;  // LVGL copies the text; the buffer is on the stack
    const int32_t h = lv_font_get_line_height(d.font);
    const lv_area_t a = {x1, y_mid - h / 2, x2, y_mid - h / 2 + h - 1};
    lv_draw_label(layer, &d, &a);
}

void draw_cb(lv_event_t *e)
{
    const auto *st = static_cast<HistState *>(lv_event_get_user_data(e));
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(lv_event_get_target_obj(e), &c);

    // Range of the levels on screen.
    float mn = 0, mx = 0;
    for (int i = 0; i < st->count; ++i) {
        const float v = st->db[i];
        if (i == 0 || v < mn) mn = v;
        if (i == 0 || v > mx) mx = v;
    }
    if (st->count == 0) mn = mx = 40.0f;
    // The scale also fits the LAeq marks; the colours use the bars only.
    float smn = mn, smx = mx;
    for (int i = 0; i < st->count; ++i) {
        if (st->leq[i] < smn) smn = st->leq[i];
        if (st->leq[i] > smx) smx = st->leq[i];
    }

    // Vertical scale: whole tens with a little headroom, at least 30 dB.
    float lo = std::floor((smn - 3.0f) / 10.0f) * 10.0f;
    float hi = std::ceil((smx + 3.0f) / 10.0f) * 10.0f;
    if (lo < 0) lo = 0;
    if (hi - lo < kMinSpanDb) hi = lo + kMinSpanDb;
    const float step = hi - lo > 60.0f ? 20.0f : 10.0f;

    const int32_t gx1 = c.x1 + kAxisW + 6;  // graph area
    const int32_t gx2 = c.x2;
    const int32_t top = c.y1 + kTopPad;
    const int32_t bottom = c.y2 - kBaseH;    // bottom row of the bars
    const float px_per_db = float(bottom - top) / (hi - lo);

    // Grid lines and labels.
    for (float v = lo; v <= hi + 0.1f; v += step) {
        const int32_t y = bottom - (int32_t)lroundf((v - lo) * px_per_db);
        if (v > lo) fill(layer, kGrid, gx1, y, gx2, y);
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", (int)v);
        label(layer, c.x1, c.x1 + kAxisW - 1, y, buf);
    }
    fill(layer, kBase, gx1, c.y2 - kBaseH + 1, gx2, c.y2);

    // Bars, right-aligned so the newest is at the right edge.
    const int32_t pitch = (gx2 - gx1 + 1) / st->bars;
    const int32_t x_end = gx2 + 1;  // one past the last bar slot
    const float colour_span = mx - mn > kMinColourSpanDb ? mx - mn : kMinColourSpanDb;
    for (int i = 0; i < st->count; ++i) {
        const int slot = st->bars - st->count + i;
        const int32_t xa = x_end - (st->bars - slot) * pitch;
        const int32_t xb = xa + pitch - kGap - 1;
        const float v = st->db[i];
        int32_t bh = (int32_t)lroundf((v - lo) * px_per_db);
        if (bh < 1) bh = 1;
        if (bh > bottom - top) bh = bottom - top;
        // Quietest violet (hue 270) .. loudest red (hue 0).
        float t = (v - mn) / colour_span;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        const lv_color_t col = lv_color_hsv_to_rgb((uint16_t)lroundf(270.0f * (1.0f - t)), 100, 100);
        fill(layer, col, xa, bottom - bh + 1, xb, bottom);
    }

    // LAeq marks over the bars, spanning the gaps too so they read as a line.
    for (int i = 0; i < st->count; ++i) {
        const int slot = st->bars - st->count + i;
        const int32_t xa = x_end - (st->bars - slot) * pitch;
        int32_t lh = (int32_t)lroundf((st->leq[i] - lo) * px_per_db);
        if (lh < kLeqH) lh = kLeqH;
        if (lh > bottom - top) lh = bottom - top;
        fill(layer, kLeqMark, xa, bottom - lh + 1, xa + pitch - 1, bottom - lh + kLeqH);
    }
}

void delete_cb(lv_event_t *e) { delete static_cast<HistState *>(lv_event_get_user_data(e)); }

}  // namespace

lv_obj_t *history_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bars)
{
    auto *st = new HistState;
    st->bars = bars;
    st->db.assign(bars, 0.0f);
    st->leq.assign(bars, 0.0f);

    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_user_data(obj, st);
    lv_obj_add_event_cb(obj, draw_cb, LV_EVENT_DRAW_MAIN, st);
    lv_obj_add_event_cb(obj, delete_cb, LV_EVENT_DELETE, st);
    return obj;
}

void history_view_set(lv_obj_t *obj, const float *db, const float *leq_db, int count)
{
    auto *st = static_cast<HistState *>(lv_obj_get_user_data(obj));
    if (count > st->bars) {
        db += count - st->bars;
        leq_db += count - st->bars;
        count = st->bars;
    }
    for (int i = 0; i < count; ++i) {
        st->db[i] = db[i];
        st->leq[i] = leq_db[i];
    }
    st->count = count;
    lv_obj_invalidate(obj);
}
