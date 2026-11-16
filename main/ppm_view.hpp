#pragma once

// Round PPM dial drawn around the edge of the screen: white scale and needle
// on black like a BBC meter, 270 degree sweep opening at the bottom. Marks
// 1..7 (BBC) or -12..+12 with TEST at 0 (EBU) at equal spacing; the section
// above mark 6 (the BBC "maximum permitted" level) is red.
//
// The needle covers only the outer ring, so readouts can sit in the middle.

#include "lvgl.h"

lv_obj_t *ppm_view_create(lv_obj_t *parent, int32_t size);

// position: 0..8 from slm::ppm_position() (marks at 1..7). Redraws only when
// the needle moves.
void ppm_view_set(lv_obj_t *obj, float position);

// true: EBU labels; false: BBC 1..7.
void ppm_view_set_ebu(lv_obj_t *obj, bool ebu);
bool ppm_view_is_ebu(lv_obj_t *obj);
