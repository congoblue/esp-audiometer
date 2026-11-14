#pragma once

// Spectrum bar display: single-sided bars rising from a row of "zero" dots
// along the bottom of the widget, coloured by hue from red (low frequencies) to violet
// (high) as in Waveshare's 05_Spec_Analyzer demo, with peak markers that fall
// slowly.

#include "lvgl.h"

lv_obj_t *spectrum_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bars);

// levels: one value per bar, 0..1 (fraction of the bar area height). Updates
// the peak markers and redraws. Call at a steady rate: bars fall at most 3 px
// and peaks 1 px per call.
void spectrum_view_set(lv_obj_t *obj, const float *levels);
