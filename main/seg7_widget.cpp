#include "seg7_widget.hpp"

#include <cstring>

namespace {

struct Seg7State {
    slm::Seg7Geometry geometry;
    char text[16] = "";
    lv_color_t lit = lv_color_white();
    lv_color_t unlit = lv_color_hex(0x202020);
};

void fill_rect(lv_layer_t *layer, lv_color_t colour, int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = colour;
    d.bg_opa = LV_OPA_COVER;
    d.radius = 0;
    const lv_area_t a = {x1, y1, x2 - 1, y2 - 1};  // lv_area_t is inclusive
    lv_draw_rect(layer, &d, &a);
}

void fill_triangle(lv_layer_t *layer, lv_color_t colour, int32_t ax, int32_t ay, int32_t bx, int32_t by,
                   int32_t cx, int32_t cy)
{
    lv_draw_triangle_dsc_t d;
    lv_draw_triangle_dsc_init(&d);
    d.color = colour;
    d.opa = LV_OPA_COVER;
    d.p[0].x = ax; d.p[0].y = ay;
    d.p[1].x = bx; d.p[1].y = by;
    d.p[2].x = cx; d.p[2].y = cy;
    lv_draw_triangle(layer, &d);
}

// Draws one hexagonal segment as a rectangle plus a triangle at each end.
// The triangles reach one pixel into the rectangle so no seam shows.
void draw_segment(lv_layer_t *layer, int32_t ox, int32_t oy, const slm::Seg7Segment &s, lv_color_t colour)
{
    const int32_t x1 = ox + s.x1, y1 = oy + s.y1, x2 = ox + s.x2, y2 = oy + s.y2;
    switch (s.shape) {
    case slm::Seg7Shape::Dot:
        fill_rect(layer, colour, x1, y1, x2, y2);
        break;
    case slm::Seg7Shape::Horizontal: {
        const int32_t half = (y2 - y1) / 2, ym = (y1 + y2) / 2;
        fill_rect(layer, colour, x1 + half, y1, x2 - half, y2);
        fill_triangle(layer, colour, x1, ym, x1 + half + 1, y1, x1 + half + 1, y2);
        fill_triangle(layer, colour, x2, ym, x2 - half - 1, y1, x2 - half - 1, y2);
        break;
    }
    case slm::Seg7Shape::Vertical: {
        const int32_t half = (x2 - x1) / 2, xm = (x1 + x2) / 2;
        fill_rect(layer, colour, x1, y1 + half, x2, y2 - half);
        fill_triangle(layer, colour, xm, y1, x1, y1 + half + 1, x2, y1 + half + 1);
        fill_triangle(layer, colour, xm, y2, x1, y2 - half - 1, x2, y2 - half - 1);
        break;
    }
    }
}

void draw_cb(lv_event_t *e)
{
    auto *st = static_cast<Seg7State *>(lv_event_get_user_data(e));
    lv_obj_t *obj = lv_event_get_target_obj(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    slm::seg7_layout(st->text, st->geometry, [&](const slm::Seg7Segment &s) {
        draw_segment(layer, c.x1, c.y1, s, s.lit ? st->lit : st->unlit);
    });
}

void delete_cb(lv_event_t *e) { delete static_cast<Seg7State *>(lv_event_get_user_data(e)); }

Seg7State *state(lv_obj_t *obj) { return static_cast<Seg7State *>(lv_obj_get_user_data(obj)); }

}  // namespace

lv_obj_t *seg7_create(lv_obj_t *parent, const slm::Seg7Geometry &geometry, const char *width_template)
{
    auto *st = new Seg7State;
    st->geometry = geometry;
    strncpy(st->text, width_template, sizeof(st->text) - 1);

    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(obj, slm::seg7_width(width_template, geometry), geometry.digit_h);
    lv_obj_set_user_data(obj, st);
    lv_obj_add_event_cb(obj, draw_cb, LV_EVENT_DRAW_MAIN, st);
    lv_obj_add_event_cb(obj, delete_cb, LV_EVENT_DELETE, st);
    return obj;
}

void seg7_set_text(lv_obj_t *obj, const char *text)
{
    Seg7State *st = state(obj);
    if (strncmp(st->text, text, sizeof(st->text) - 1) == 0) return;
    strncpy(st->text, text, sizeof(st->text) - 1);
    lv_obj_invalidate(obj);
}

void seg7_set_colors(lv_obj_t *obj, lv_color_t lit, lv_color_t unlit)
{
    Seg7State *st = state(obj);
    st->lit = lit;
    st->unlit = unlit;
    lv_obj_invalidate(obj);
}
