#pragma once

// LVGL widget that shows text as a seven-segment display. Unlit segments are
// drawn in a dim colour, like a real LED/LCD readout.

#include "lvgl.h"

#include "seg7_layout.hpp"

// Creates the widget, sized to fit `width_template` (e.g. "888.8"). Use text
// of the same layout in seg7_set_text so the readout never moves.
lv_obj_t *seg7_create(lv_obj_t *parent, const slm::Seg7Geometry &geometry, const char *width_template);

// Sets the text; redraws only if it changed.
void seg7_set_text(lv_obj_t *obj, const char *text);

void seg7_set_colors(lv_obj_t *obj, lv_color_t lit, lv_color_t unlit);
