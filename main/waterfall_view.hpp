#pragma once

// Spectrum waterfall: each update adds one row at the top, one cell per band
// (low frequencies on the left), and the older rows move down. Louder bands
// are brighter: black -> blue -> purple -> red -> yellow -> white.
//
// Drawn on an LVGL canvas whose RGB565 pixel buffer lives in PSRAM; scrolling
// moves the buffer down by one row, so only the new row is computed.

#include "lvgl.h"

// Returns nullptr if the PSRAM buffer can't be allocated.
lv_obj_t *waterfall_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bands, int32_t row_h);

// Fills the waterfall with black.
void waterfall_view_clear(lv_obj_t *obj);

// levels: one value per band, 0..1. Adds a row at the top and redraws.
void waterfall_view_push(lv_obj_t *obj, const float *levels);
