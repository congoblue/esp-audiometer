#pragma once

// AXP2101 battery monitoring and the battery icon widget.

#include "lvgl.h"

#include "battery_status.hpp"

// Starts a task that watches the PWR key every 50 ms and reads the battery
// every 2 s. Holding PWR for 2 s switches the unit off through the AXP2101.
// Call after bsp_display_start() (which initialises the shared I2C bus).
void battery_start();

// Latest reading; status.valid is false until the PMU has been read.
slm::BatteryStatus battery_get();

// PWR key state, for showing "Hold to power off" / "Powering off".
slm::PowerKey power_key_get();

// Battery icon: outline with a fill bar coloured by charge level, a lightning
// bolt while charging, and a red line through it when no battery is fitted.
lv_obj_t *battery_icon_create(lv_obj_t *parent);

// Updates the icon; redraws only if something visible changed.
void battery_icon_set(lv_obj_t *icon, const slm::BatteryStatus &status);
