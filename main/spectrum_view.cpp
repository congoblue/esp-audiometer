#include "spectrum_view.hpp"

#include <vector>

namespace {

constexpr int32_t kGap = 2;          // pixels between bars
constexpr int32_t kPeakH = 3;        // peak marker thickness
constexpr float kPeakFallPx = 2.0f;  // per update

struct SpecState {
    int bars;
    std::vector<float> level_px;  // bar half-height in pixels
    std::vector<float> peak_px;   // peak marker position in pixels
    std::vector<lv_color_t> colour;
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

void draw_cb(lv_event_t *e)
{
    const auto *st = static_cast<SpecState *>(lv_event_get_user_data(e));
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(lv_event_get_target_obj(e), &c);

    const int32_t w = lv_area_get_width(&c), h = lv_area_get_height(&c);
    const int32_t pitch = w / st->bars;
    const int32_t x0 = c.x1 + (w - pitch * st->bars) / 2;  // centre the bars
    const int32_t mid = c.y1 + h / 2;
    const int32_t max_half = h / 2 - kPeakH;

    for (int i = 0; i < st->bars; ++i) {
        const int32_t xa = x0 + i * pitch + kGap / 2;
        const int32_t xb = xa + pitch - kGap - 1;
        int32_t bh = (int32_t)st->level_px[i];
        if (bh > max_half) bh = max_half;
        if (bh > 0) fill(layer, st->colour[i], xa, mid - bh, xb, mid + bh - 1);

        int32_t pk = (int32_t)st->peak_px[i];
        if (pk > max_half) pk = max_half;
        fill(layer, st->colour[i], xa, mid - pk - kPeakH, xb, mid - pk - 1);  // top marker
        fill(layer, st->colour[i], xa, mid + pk, xb, mid + pk + kPeakH - 1);  // bottom marker
    }
}

void delete_cb(lv_event_t *e) { delete static_cast<SpecState *>(lv_event_get_user_data(e)); }

}  // namespace

lv_obj_t *spectrum_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bars)
{
    auto *st = new SpecState;
    st->bars = bars;
    st->level_px.assign(bars, 0.0f);
    st->peak_px.assign(bars, 0.0f);
    for (int i = 0; i < bars; ++i) {
        const uint16_t hue = (uint16_t)(i * 270 / bars);  // red -> violet, as in the demo
        st->colour.push_back(lv_color_hsv_to_rgb(hue, 100, 100));
    }

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

void spectrum_view_set(lv_obj_t *obj, const float *levels)
{
    auto *st = static_cast<SpecState *>(lv_obj_get_user_data(obj));
    const float half = lv_obj_get_height(obj) / 2.0f - kPeakH;
    for (int i = 0; i < st->bars; ++i) {
        float v = levels[i];
        v = v < 0 ? 0 : (v > 1 ? 1 : v);
        const float px = v * half;
        st->level_px[i] = px;
        if (px >= st->peak_px[i]) st->peak_px[i] = px;
        else st->peak_px[i] = st->peak_px[i] > kPeakFallPx ? st->peak_px[i] - kPeakFallPx : 0.0f;
    }
    lv_obj_invalidate(obj);
}
