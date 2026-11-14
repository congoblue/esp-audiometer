#pragma once

// Spectrum bar display in the style of Waveshare's 05_Spec_Analyzer demo:
// bars mirrored above and below a centre line, coloured by hue from red
// (low frequencies) to violet (high), with peak markers that fall slowly.

#include "lvgl.h"

lv_obj_t *spectrum_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bars);

// levels: one value per bar, 0..1 (fraction of the half-height). Updates the
// peak markers and redraws. Call at a steady rate (peaks fall 2 px per call).
void spectrum_view_set(lv_obj_t *obj, const float *levels);
