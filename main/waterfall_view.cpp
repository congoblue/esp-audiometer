#include "waterfall_view.hpp"

#include <cstring>

#include "esp_heap_caps.h"

namespace {

struct WfState {
    int bands;
    int32_t w, h, row_h;
    uint32_t stride;    // bytes per pixel row
    uint8_t *buf;       // RGB565, PSRAM
    uint16_t lut[256];  // level 0..255 -> RGB565
};

// Colour map stops: (position 0..255, colour). Brightness rises throughout.
void build_lut(uint16_t *lut)
{
    struct Stop { int pos; uint32_t rgb; };
    const Stop stops[] = {
        {0, 0x000000},    // silence: black
        {50, 0x00008C},   // dark blue
        {105, 0x8000C8},  // purple
        {160, 0xFF0020},  // red
        {215, 0xFFD000},  // yellow
        {255, 0xFFFFFF},  // loudest: white
    };
    for (size_t s = 0; s + 1 < sizeof(stops) / sizeof(stops[0]); ++s) {
        const Stop &a = stops[s], &b = stops[s + 1];
        for (int i = a.pos; i <= b.pos; ++i) {
            const int t = (i - a.pos) * 256 / (b.pos - a.pos);
            auto mix = [&](int shift) {
                const int ca = (a.rgb >> shift) & 0xFF, cb = (b.rgb >> shift) & 0xFF;
                return (uint8_t)(ca + (cb - ca) * t / 256);
            };
            lut[i] = lv_color_to_u16(lv_color_make(mix(16), mix(8), mix(0)));
        }
    }
}

void delete_cb(lv_event_t *e)
{
    auto *st = static_cast<WfState *>(lv_event_get_user_data(e));
    heap_caps_free(st->buf);
    delete st;
}

}  // namespace

lv_obj_t *waterfall_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bands, int32_t row_h)
{
    const uint32_t stride = lv_draw_buf_width_to_stride(w, LV_COLOR_FORMAT_RGB565);
    auto *buf = static_cast<uint8_t *>(heap_caps_calloc(stride * h, 1, MALLOC_CAP_SPIRAM));
    if (buf == nullptr) return nullptr;

    auto *st = new WfState;
    st->bands = bands;
    st->w = w;
    st->h = h;
    st->row_h = row_h;
    st->stride = stride;
    st->buf = buf;
    build_lut(st->lut);

    lv_obj_t *obj = lv_canvas_create(parent);
    lv_canvas_set_buffer(obj, buf, w, h, LV_COLOR_FORMAT_RGB565);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(obj, st);
    lv_obj_add_event_cb(obj, delete_cb, LV_EVENT_DELETE, st);
    return obj;
}

void waterfall_view_clear(lv_obj_t *obj)
{
    auto *st = static_cast<WfState *>(lv_obj_get_user_data(obj));
    memset(st->buf, 0, st->stride * st->h);  // black is 0 in RGB565
    lv_obj_invalidate(obj);
}

void waterfall_view_push(lv_obj_t *obj, const float *levels)
{
    auto *st = static_cast<WfState *>(lv_obj_get_user_data(obj));

    // Scroll everything down one row; the bottom row falls off.
    memmove(st->buf + st->row_h * st->stride, st->buf, (st->h - st->row_h) * st->stride);

    // New top row: each band fills an equal share of the width.
    auto *row = reinterpret_cast<uint16_t *>(st->buf);
    for (int32_t x = 0; x < st->w; ++x) {
        const int b = x * st->bands / st->w;
        float v = levels[b];
        v = v < 0 ? 0 : (v > 1 ? 1 : v);
        row[x] = st->lut[(int)(v * 255.0f + 0.5f)];
    }
    for (int32_t y = 1; y < st->row_h; ++y) memcpy(st->buf + y * st->stride, row, st->w * 2);

    lv_obj_invalidate(obj);
}
