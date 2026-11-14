# ESP32-S3 A-weighted Sound Level Meter

A sound level meter in C++ for the
[Waveshare ESP32-S3-Touch-AMOLED-2.16](https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-2.16).
It reads both on-board microphones through the ES7210 ADC and shows A-weighted
levels on the 480×480 AMOLED.

## What it shows

| Reading | Meaning |
|---|---|
| Big number | LAF (Fast, 125 ms) or LAS (Slow, 1 s). Tap the number to switch. |
| Arc gauge | 30–130 dB(A). Green below 70, amber 70–85, red at 85 and above. |
| LAeq 15m | Equivalent continuous level over the last 15 minutes (rolling). Until 15 minutes have passed since power-on or **Reset**, it covers the time so far. |
| Max / Min | Max and min of LAF since the last **Reset**. |
| Mic 1 / Mic 2 | LAF of each capsule on its own. A large gap means one mic is blocked or faulty. |
| OVERLOAD | Either mic hit ADC full scale in the last second, so the reading is not valid. |

**Cal** opens a calibration page with ±0.1 and ±1 dB steps. The value is saved to NVS.

## How it works

```
2 × MEMS mic ─► ES7210 (PGA, 24-bit ADC) ─I2S 48 kHz/32-bit stereo─► ESP32-S3
   audio task (core 1): average L+R ─► A-weighting IIR ─► Fast/Slow/Leq/Max/Min
   LVGL (BSP adapter task): 10 Hz UI refresh
```

- **A-weighting** ([main/a_weighting.hpp](main/a_weighting.hpp)): three biquads.
  The low-frequency poles use a pre-warped bilinear transform. The 12.2 kHz
  pole pair uses matched-z mapping with a fitted zero, which avoids the
  bilinear transform's large error near Nyquist. At 48 kHz the filter is within
  0.07 dB of the IEC 61672-1 curve from 10 Hz to 10 kHz, and within 0.54 dB at
  20 kHz.
- **Two microphones**: the main reading uses the average of both capsules.
  Sound adds coherently across the two capsules but their self-noise does not,
  so averaging lowers the noise floor by about 3 dB. Each mic is also measured
  on its own as a health check.
- **Time weighting** ([main/level_meter.hpp](main/level_meter.hpp)): exponential
  averaging per sample, τ = 125 ms (F) and 1 s (S). Leq is the plain energy
  average. Max and min ignore the first 0.5 s after boot while the filters settle.
- **Hardware access** uses Waveshare's BSP component
  (`waveshare/esp32_s3_touch_amoled_2_16`): CO5300 display, CST9220 touch,
  ES7210 microphone codec over `esp_codec_dev`, and LVGL 9.

## Build and flash

You need **ESP-IDF v5.5 or newer**, which is what the BSP requires.

```sh
idf.py set-target esp32s3     # first time only
idf.py build
idf.py -p COMx flash monitor
```

The component manager downloads the BSP, LVGL and the drivers on the first
build. The serial monitor also prints the levels once a second.

Settings are under `idf.py menuconfig` → **Sound Level Meter**:

| Option | Default | Notes |
|---|---|---|
| `SLM_MIC_GAIN_DB` | 12 | ES7210 PGA gain. Lower it for very loud sources, raise it for quiet rooms. Calibration is stored relative to 0 dB gain, so you do not need to recalibrate after changing it. |
| `SLM_CAL_OFFSET_X10` | 1350 | Default offset of 135.0 dB, used until you save a calibration on the device. |

## Auto-rotation

The QMI8658 accelerometer works out which edge of the board is pointing up,
and the display turns in 90° steps to match ([main/auto_rotate.cpp](main/auto_rotate.cpp)).

- The display controller does the rotation (CO5300 MADCTL), so it costs no
  CPU. Touch coordinates are rotated in software to match.
- The screen only turns once the board has been held in the new position for
  0.4 s, and there is a ±15° dead band between positions.
- Lying flat on a table, the screen keeps its current orientation.

If an orientation comes out wrong, use **menuconfig → Sound Level Meter →
Auto-rotation**:

| Symptom | Fix |
|---|---|
| Wrong by the same angle in every position | `SLM_IMU_ROTATION_OFFSET`, in quarter turns clockwise. The default is 3 (90° anticlockwise). |
| Taps register a quarter or half turn away from the button, the same in every orientation | `SLM_TOUCH_ROTATION_OFFSET`. The default is 1; try 3 if that is worse. |
| On its side the picture is upside down, but taps hit what's drawn | `SLM_IMU_MIRROR` |
| On its side the picture is upside down, and taps hit the opposite button | `SLM_PANEL_SWAP_90_270` |

Set the log level for the `rotate` tag to Debug to see the raw accelerometer
readings.

## Calibration

The default offset is only an estimate. It assumes a −38 dBV/Pa MEMS capsule.
**Until you calibrate, treat readings as relative, not absolute.**

1. Put a 94 dB (1 kHz) acoustic calibrator over the mic ports, or place the
   board next to a reference sound level meter in a steady noise source.
2. Tap **Cal**.
3. Use the step buttons until **Live LAS** matches the reference.
4. Tap **Save**.

Formula: `dB SPL = dBFS(RMS) − PGA gain + offset`.

## Limitations

- These are uncompensated consumer MEMS microphones behind the enclosure's
  acoustic ports. Their frequency response, and the small enclosure, affect the
  result more than the filter does. Expect roughly ±1–2 dB accuracy on broadband
  noise after calibration. This is **not a certified IEC 61672 instrument**.
- The noise floor is set by the capsules' self-noise, typically about
  30–35 dB(A), so very quiet rooms read high.
- The upper limit depends on the PGA gain. Watch for the OVERLOAD indicator.

## Host tests

The DSP headers have no ESP-IDF dependencies. The tests check the filter
against the IEC curve and class 1 tolerances, check float precision with sine
inputs, and check Fast/Slow decay rates (34.7 dB/s and 4.3 dB/s):

```sh
cd test
clang++ -std=c++17 -O2 -D_USE_MATH_DEFINES -I../main host_test.cpp -o host_test
./host_test
```
