#pragma once

// Rotates the UI to follow the device orientation, using the QMI8658 IMU.
//
// Must be called after bsp_display_start(), with the display lock held.
// Starts a low-priority task that polls the accelerometer, rotates the panel
// in hardware (CO5300 MADCTL), and remaps touch coordinates to match.
// If the IMU is not found the display stays in its default orientation.
void auto_rotate_start();
