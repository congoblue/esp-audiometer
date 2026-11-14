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
| LAeq 15m | Smaller 7-segment readout, in whole dB. Equivalent continuous level over the last 15 minutes (rolling). Until 15 minutes have passed since power-on, it covers the time so far. |
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
- **Interference low-pass** ([main/lowpass.hpp](main/lowpass.hpp)): the mic
  signal on this board picks up electrical interference from the board itself,
  at about 14.8 kHz with more near 15.8 and 17.4 kHz. Covering the mic ports
  makes no difference, so it isn't sound. A 10th-order Chebyshev type II
  low-pass runs before the A-weighting. It is flat to 11 kHz, −0.4 dB at
  12.5 kHz, and at least 50 dB down from 14.5 kHz. Measured on the board, it
  lowered the quiet-room reading from about 35 to 27.5 dB(A), and brought the
  two mics from 6.5 dB apart to within 0.1 dB of each other. Sound above about
  13 kHz is no longer measured, which takes the meter outside class 1
  tolerance at 16 kHz.
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

## Screens

There are four screens. The button at the bottom centre (above the battery
icon) moves to the next one in turn:

**dB meter** → **Analyser** → **History** → **Waterfall** → **dB meter**

On the meter screen the button is **Analyser**, next to **Cal**. The other
three screens all show the current dB(A) bottom left and LAeq 15m bottom
right, on 7-segment readouts. Tap the dB(A) readout to switch between Fast
and Slow; the setting is shared by all the screens.

## Spectrum screen

- 64 bars on a log frequency scale from 50 Hz to 13 kHz (the interference
  low-pass removes everything above), taken from the
  average of both mics. Bars rise from a row of zero-position dots just above
  the readouts and are coloured red to violet. Bars fall back at up to
  75 px/s and the peak markers at 25 px/s, in the style of
  Waveshare's `05_Spec_Analyzer` demo.
- The bars show unweighted levels per band in dB SPL, using the same
  calibration as the meter. The zero dots are 20 dB and full height is
  100 dB, with a square-root scale as in the demo.
- 4096-point FFT (85 ms window, 11.7 Hz bins), refreshed 25 times a second.
  It only runs while the analyser or waterfall screen is showing.

## History screen

A rolling bar graph of dB(A) over the last 15 minutes
([main/level_history.hpp](main/level_history.hpp),
[main/history_view.cpp](main/history_view.cpp)).

- 90 bars, each the LAeq of 10 seconds. The newest bar is on the right and is
  updated every second while it fills; then the graph moves one bar to the
  left.
- History is recorded from power-on whichever screen is showing, so opening
  the screen shows the last 15 minutes straight away.
- The vertical scale fits the levels on screen, in 10 dB steps (at least
  30 dB tall), with labelled grid lines down the left.
- Colours are relative to what is on screen: the quietest bar is violet and
  the loudest red. When all the levels are within 6 dB of each other, the
  colours stay at the violet end, so small changes don't look dramatic.
- A grey mark on each bar shows what LAeq 15m was at that time, so the marks
  form a line showing how the 15-minute average has moved.

## Waterfall screen

The spectrum over time ([main/waterfall_view.cpp](main/waterfall_view.cpp)).

- Every 0.25 s a new line is drawn at the top and the older lines move down.
  The screen holds 100 lines, so it shows the last 25 seconds.
- Each line has the same 64 bands as the analyser (50 Hz on the left to
  13 kHz on the right, marked at 100, 1k and 10k) with the same
  calibration and level scale. Each line is the energy average of the FFT
  frames in its quarter second.
- Louder is brighter: black → blue → purple → red → yellow → white.
- It starts empty each time you open the screen, because the FFT only runs
  while the analyser or waterfall is showing.

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
- The noise floor is set by the capsules' self-noise. Measured on this board,
  it is about 27–28 dB(A) with the default calibration, so very quiet rooms
  read high.
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
