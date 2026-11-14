#pragma once

// Rolling bar graph of recent dB(A) levels for the History screen.
//
// Newest bar on the right; older bars scroll off to the left. The vertical
// scale fits the levels shown, in 10 dB steps with labelled grid lines down
// the left. Bar colour is relative to what is on screen: violet for the
// quietest bar, red for the loudest.

#include "lvgl.h"

lv_obj_t *history_view_create(lv_obj_t *parent, int32_t w, int32_t h, int bars);

// db: `count` levels in dB SPL, oldest first (count <= bars). Redraws.
void history_view_set(lv_obj_t *obj, const float *db, int count);
