// A-weighted sound level meter for the Waveshare ESP32-S3-Touch-AMOLED-2.16.
//
// Audio path: two on-board MEMS microphones -> ES7210 ADC -> I2S (48 kHz,
// 32-bit, stereo) -> A-weighting + Fast/Slow time weighting (level_meter.hpp)
// -> LVGL UI on the 480x480 CO5300 AMOLED.

#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "esp_codec_dev.h"
#include "lvgl.h"

#include "auto_rotate.hpp"
#include "history_view.hpp"
#include "level_history.hpp"
#include "level_meter.hpp"
#include "seg7_widget.hpp"
#include "spectrum.hpp"
#include "spectrum_view.hpp"
#include "waterfall_view.hpp"
#include "battery.hpp"

namespace {

const char *TAG = "slm";

constexpr uint32_t kSampleRate = 48000;
constexpr size_t kBlockFrames = 480;  // 10 ms per read
constexpr float kMicGainDb = CONFIG_SLM_MIC_GAIN_DB;

// Display range of the arc gauge (dB SPL).
constexpr float kArcMinDb = 30.0f;
constexpr float kArcMaxDb = 130.0f;

// ---------------------------------------------------------------------------
// Shared state between the audio task and the UI
// ---------------------------------------------------------------------------
std::mutex g_levels_mutex;
slm::Levels g_levels;
std::atomic<bool> g_reset_request{false};
std::atomic<bool> g_audio_ok{false};

// Calibration: SPL = dBFS - mic_gain + g_cal_offset.  Expressing the offset
// at 0 dB PGA gain means changing CONFIG_SLM_MIC_GAIN_DB keeps calibration.
float g_cal_offset = CONFIG_SLM_CAL_OFFSET_X10 / 10.0f;

// ---------------------------------------------------------------------------
// Calibration persistence
// ---------------------------------------------------------------------------
constexpr const char *kNvsNamespace = "slm";
constexpr const char *kNvsCalKey = "cal_x10";

void load_calibration()
{
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, kNvsCalKey, &v) == ESP_OK) {
        g_cal_offset = v / 10.0f;
        ESP_LOGI(TAG, "Loaded calibration offset %.1f dB", g_cal_offset);
    }
    nvs_close(h);
}

void save_calibration()
{
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, kNvsCalKey, (int32_t)lroundf(g_cal_offset * 10.0f));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved calibration offset %.1f dB", g_cal_offset);
}

float to_spl(float dbfs) { return dbfs - kMicGainDb + g_cal_offset; }

// ---------------------------------------------------------------------------
// History screen data: 90 bars x 10 s = the last 15 minutes. The audio task
// adds each finished second; the UI copies it when it changes.
// ---------------------------------------------------------------------------
constexpr int kHistBars = 90;
constexpr int kHistSecondsPerBar = 10;
std::mutex g_hist_mutex;
slm::LevelHistory g_history(kHistBars, kHistSecondsPerBar);

// ---------------------------------------------------------------------------
// Spectrum analyser data
// ---------------------------------------------------------------------------
using slm::SpectrumAnalyzer;
constexpr size_t kSpecN = SpectrumAnalyzer::kN;
constexpr int kSpecBands = SpectrumAnalyzer::kBands;
constexpr uint32_t kSpecFrameMs = 40;   // 25 frames per second
constexpr float kSpecMinDb = 20.0f;     // band level (dB SPL) at the baseline
constexpr float kSpecMaxDb = 100.0f;    // band level (dB SPL) at full height

std::atomic<bool> g_spec_on{false};     // analyser or waterfall screen visible
std::mutex g_spec_mutex;
// Latest samples, (L+R)/2, unweighted. In PSRAM (allocated in app_main):
// internal RAM must stay free for the I2S and display DMA buffers.
float *g_spec_ring = nullptr;
size_t g_spec_head = 0;                 // next write position
float g_spec_db[kSpecBands];            // band levels in dB SPL
bool g_spec_fresh = false;

// Waterfall: one row every 0.25 s, each the energy average of the FFT frames
// in that time (about 6). Shares g_spec_mutex.
constexpr uint32_t kWfRowMs = 100;
constexpr int32_t kWfRowPx = 2;         // 300 px tall -> 150 rows = 25 s
float g_wf_db[kSpecBands];              // latest row, dB SPL
bool g_wf_fresh = false;
std::atomic<bool> g_wf_restart{false};  // start a new row from scratch

// Analyser range: up to 13 kHz, the top of the interference low-pass's
// passband (everything above is the board's own electrical noise).
constexpr double kSpecLowHz = 50.0;
constexpr double kSpecHighHz = 13000.0;

// Audio task: append (L+R)/2, low-passed like the meter's input, to the ring
// while the spectrum is shown.
void spec_push(const float *l, const float *r, size_t n)
{
    static slm::ChebyLowpass lp(kSampleRate);
    std::lock_guard<std::mutex> lock(g_spec_mutex);
    for (size_t i = 0; i < n; ++i) {
        g_spec_ring[g_spec_head] = lp.process(0.5f * (l[i] + r[i]));
        g_spec_head = (g_spec_head + 1) % kSpecN;
    }
}

void spectrum_task(void *)
{
    static SpectrumAnalyzer analyzer(kSampleRate, kSpecLowHz, kSpecHighHz);  // buffers are heap (PSRAM)
    auto *frame = static_cast<float *>(heap_caps_malloc(kSpecN * sizeof(float), MALLOC_CAP_SPIRAM));
    if (frame == nullptr) {
        ESP_LOGE(TAG, "No PSRAM for spectrum frame; analyser disabled");
        vTaskDelete(nullptr);
        return;
    }
    float ms[kSpecBands];
    double wf_acc[kSpecBands] = {};  // waterfall row being averaged
    int wf_frames = 0;
    TickType_t wf_start = xTaskGetTickCount();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kSpecFrameMs));
        if (!g_spec_on) continue;
        {
            std::lock_guard<std::mutex> lock(g_spec_mutex);
            // Oldest sample first.
            const size_t tail = kSpecN - g_spec_head;
            memcpy(frame, g_spec_ring + g_spec_head, tail * sizeof(float));
            memcpy(frame + tail, g_spec_ring, g_spec_head * sizeof(float));
        }
        analyzer.analyse(frame, ms);
        float db[kSpecBands];
        for (int b = 0; b < kSpecBands; ++b) db[b] = to_spl(slm::LevelMeter::to_db(ms[b]));

        // Waterfall row: average the frames of each 0.25 s.
        const TickType_t now = xTaskGetTickCount();
        if (g_wf_restart.exchange(false)) {
            memset(wf_acc, 0, sizeof(wf_acc));
            wf_frames = 0;
            wf_start = now;
        }
        for (int b = 0; b < kSpecBands; ++b) wf_acc[b] += ms[b];
        ++wf_frames;
        bool wf_row = false;
        float wf_db[kSpecBands];
        if (now - wf_start >= pdMS_TO_TICKS(kWfRowMs)) {
            for (int b = 0; b < kSpecBands; ++b) {
                wf_db[b] = to_spl(slm::LevelMeter::to_db(float(wf_acc[b] / wf_frames)));
                wf_acc[b] = 0;
            }
            wf_frames = 0;
            wf_start += pdMS_TO_TICKS(kWfRowMs);
            if (now - wf_start >= pdMS_TO_TICKS(kWfRowMs)) wf_start = now;  // fell behind: resync
            wf_row = true;
        }

        std::lock_guard<std::mutex> lock(g_spec_mutex);
        memcpy(g_spec_db, db, sizeof(db));
        g_spec_fresh = true;
        if (wf_row) {
            memcpy(g_wf_db, wf_db, sizeof(wf_db));
            g_wf_fresh = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Audio task
// ---------------------------------------------------------------------------
int32_t s_raw[kBlockFrames * 2];
float s_left[kBlockFrames];
float s_right[kBlockFrames];

// Opens the ES7210 at 48 kHz / 32 bit / stereo. Called from app_main BEFORE
// the display starts: the I2S driver needs internal DMA-capable RAM, and once
// LVGL is drawing its first full frame there may not be enough left (that
// caused a boot loop: "i2s_alloc_dma_desc: allocate DMA buffer failed",
// then a crash inside esp_codec_dev_open).
esp_codec_dev_handle_t open_microphones()
{
    esp_codec_dev_handle_t mic = bsp_audio_codec_microphone_init();
    if (mic == nullptr) {
        ESP_LOGE(TAG, "ES7210 microphone init failed");
        return nullptr;
    }
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = kSampleRate;
    fs.channel = 2;           // MIC1 = left slot, MIC2 = right slot
    fs.bits_per_sample = 32;  // 24-bit ADC data, MSB-aligned in 32-bit slots
    if (esp_codec_dev_open(mic, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to open microphone codec");
        return nullptr;
    }
    esp_codec_dev_set_in_gain(mic, kMicGainDb);
    ESP_LOGI(TAG, "Microphones running: %" PRIu32 " Hz, 2 ch, 32 bit, PGA %.1f dB",
             kSampleRate, kMicGainDb);
    return mic;
}

void audio_task(void *arg)
{
    auto mic = static_cast<esp_codec_dev_handle_t>(arg);
    g_audio_ok = true;

    static slm::LevelMeter meter(kSampleRate);
    constexpr float kScale = 1.0f / 2147483648.0f;
    int64_t last_log = 0;
    uint64_t seconds_seen = 0;

    while (true) {
        if (esp_codec_dev_read(mic, s_raw, sizeof(s_raw)) != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "I2S read failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        for (size_t i = 0; i < kBlockFrames; ++i) {
            s_left[i] = s_raw[2 * i] * kScale;
            s_right[i] = s_raw[2 * i + 1] * kScale;
        }

        if (g_reset_request.exchange(false)) meter.reset_stats();
        meter.process(s_left, s_right, kBlockFrames);
        if (g_spec_on && g_spec_ring) spec_push(s_left, s_right, kBlockFrames);
        if (meter.seconds_done() != seconds_seen) {  // a second finished in this block
            seconds_seen = meter.seconds_done();
            std::lock_guard<std::mutex> lock(g_hist_mutex);
            g_history.push_second(meter.last_second_ms(), meter.laeq_ms());
        }

        const slm::Levels snap = meter.snapshot();
        {
            std::lock_guard<std::mutex> lock(g_levels_mutex);
            g_levels = snap;
        }

        const int64_t now = esp_log_timestamp();
        if (now - last_log >= 1000) {
            last_log = now;
            ESP_LOGI(TAG, "LAF %.1f  LAS %.1f  LAeq %.1f dB(A)  [mic1 %.1f mic2 %.1f]%s",
                     to_spl(snap.laf), to_spl(snap.las), to_spl(snap.laeq),
                     to_spl(snap.mic1), to_spl(snap.mic2), snap.clipping ? " CLIP" : "");
        }
    }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
// The bottom button on each screen moves to the next: meter -> analyser ->
// history -> waterfall -> meter.
enum class Screen { Meter, Analyser, History, Waterfall };
constexpr int kScreens = 4;

// dB(A) (tap for Fast/Slow) and LAeq 15m readouts in the bottom corners of
// the analyser and history screens.
struct CornerReadouts {
    lv_obj_t *dba;      // seven-segment
    lv_obj_t *dba_cap;
    lv_obj_t *leq;      // seven-segment
};

struct Ui {
    Screen screen = Screen::Meter;
    lv_obj_t *main_view;   // sound level meter screen
    lv_obj_t *spec_view;   // spectrum analyser screen
    lv_obj_t *spec;
    CornerReadouts spec_corners;
    lv_obj_t *hist_view;   // history screen
    lv_obj_t *hist;
    CornerReadouts hist_corners;
    uint32_t hist_version = UINT32_MAX;  // history version last drawn
    lv_obj_t *wf_view;     // waterfall screen
    lv_obj_t *wf;          // canvas; nullptr if no PSRAM for it
    CornerReadouts wf_corners;
    lv_obj_t *arc;
    lv_obj_t *value;
    lv_obj_t *mode;
    lv_obj_t *over;
    lv_obj_t *leq;
    lv_obj_t *cal_panel;
    lv_obj_t *cal_value;
    lv_obj_t *cal_live;
    lv_obj_t *cal_mics;
    lv_obj_t *battery;
    lv_obj_t *power_overlay;
    lv_obj_t *power_label;
    slm::PowerKey shown_key = slm::PowerKey::Idle;
    bool slow = false;
    float cal_backup = 0;
} ui;

const lv_color_t kColBg = lv_color_hex(0x000000);
const lv_color_t kColText = lv_color_hex(0xF2F2F2);
const lv_color_t kColDim = lv_color_hex(0x8A8A8A);
const lv_color_t kColTrack = lv_color_hex(0x1E1E1E);
const lv_color_t kColGreen = lv_color_hex(0x2ECC71);
const lv_color_t kColAmber = lv_color_hex(0xF5A623);
const lv_color_t kColRed = lv_color_hex(0xE74C3C);
const lv_color_t kColButton = lv_color_hex(0x2A2A2A);

lv_color_t level_colour(float db)
{
    if (db >= 85.0f) return kColRed;  // typical occupational action level
    if (db >= 70.0f) return kColAmber;
    return kColGreen;
}

void fmt_db(char *buf, size_t n, float spl)
{
    if (spl < 0.0f) snprintf(buf, n, "--.-");
    else snprintf(buf, n, "%.1f", spl);
}

// Main reading for the seven-segment display, in whole dB. The hundreds place
// is a narrow "1"-only cell ('I' lit, 'i' unlit), then two digits, so the
// digits never shift:
//   54.49 -> "i54"   54.5 -> "i55"   99.5 -> "I00"   no reading -> "i--"
// roundf rounds halves away from zero, so for these positive levels .5 and
// above round up and anything below .5 rounds down.
void fmt_seg7(char *buf, size_t n, float spl)
{
    if (spl < 0.0f) {
        snprintf(buf, n, "i--");
        return;
    }
    const int v = (int)roundf(spl > 199.0f ? 199.0f : spl);  // round before splitting off the hundreds
    if (v >= 100) snprintf(buf, n, "I%02d", v - 100);
    else snprintf(buf, n, "i%2d", v);
}

// Sets a label's text only when it changes (avoids a redraw every update).
void set_label(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0) lv_label_set_text(label, text);
}

void update_cal_label()
{
    char buf[48];
    snprintf(buf, sizeof(buf), "Offset %.1f dB", g_cal_offset);
    lv_label_set_text(ui.cal_value, buf);
}

// Full-screen message while the PWR key is held, on the top layer so it also
// covers the calibration page.
void update_power_overlay()
{
    const slm::PowerKey k = power_key_get();
    if (k == ui.shown_key) return;
    ui.shown_key = k;
    if (k == slm::PowerKey::Idle) {
        lv_obj_add_flag(ui.power_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_label_set_text(ui.power_label, k == slm::PowerKey::Held ? "Hold to power off" : "Powering off");
    lv_obj_remove_flag(ui.power_overlay, LV_OBJ_FLAG_HIDDEN);
}

// Redraws the history graph when a new second has been added.
void update_history()
{
    double ms[kHistBars], leq_ms[kHistBars];
    size_t n;
    {
        std::lock_guard<std::mutex> lock(g_hist_mutex);
        if (g_history.version() == ui.hist_version) return;
        ui.hist_version = g_history.version();
        n = g_history.copy(ms, leq_ms);
    }
    float db[kHistBars], leq_db[kHistBars];
    for (size_t i = 0; i < n; ++i) {
        db[i] = to_spl(slm::LevelMeter::to_db((float)ms[i]));
        leq_db[i] = to_spl(slm::LevelMeter::to_db((float)leq_ms[i]));
    }
    history_view_set(ui.hist, db, leq_db, (int)n);
}

void ui_timer_cb(lv_timer_t *)
{
    slm::Levels l;
    {
        std::lock_guard<std::mutex> lock(g_levels_mutex);
        l = g_levels;
    }
    battery_icon_set(ui.battery, battery_get());  // redraws only on change
    update_power_overlay();

    if (!g_audio_ok) {
        seg7_set_text(ui.value, "i--");
        seg7_set_text(ui.leq, "i--");
        set_label(ui.over, "NO MICROPHONE");
        lv_obj_remove_flag(ui.over, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    const float main_db = to_spl(ui.slow ? l.las : l.laf);
    const float leq_db = to_spl(l.laeq);  // rolling 15-minute window
    char buf[128], a[16], b[16];

    fmt_seg7(buf, sizeof(buf), main_db);
    seg7_set_text(ui.value, buf);

    const float clamped = main_db < kArcMinDb ? kArcMinDb : (main_db > kArcMaxDb ? kArcMaxDb : main_db);
    lv_arc_set_value(ui.arc, (int32_t)lroundf(clamped * 10.0f));
    lv_obj_set_style_arc_color(ui.arc, level_colour(main_db), LV_PART_INDICATOR);

    // LAeq 15m: whole dB, same rounding as the main reading.
    fmt_seg7(buf, sizeof(buf), leq_db);
    seg7_set_text(ui.leq, buf);

    // Analyser / history / waterfall screen corners: dB(A) and LAeq 15m, in
    // whole dB.
    if (ui.screen != Screen::Meter) {
        const CornerReadouts &c = ui.screen == Screen::Analyser ? ui.spec_corners
                                  : ui.screen == Screen::History ? ui.hist_corners
                                                                 : ui.wf_corners;
        fmt_seg7(buf, sizeof(buf), main_db);
        seg7_set_text(c.dba, buf);
        fmt_seg7(buf, sizeof(buf), leq_db);
        seg7_set_text(c.leq, buf);
    }
    if (ui.screen == Screen::History) update_history();

    if (l.clipping) {
        set_label(ui.over, "OVERLOAD");
        lv_obj_remove_flag(ui.over, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ui.over, LV_OBJ_FLAG_HIDDEN);
    }

    if (!lv_obj_has_flag(ui.cal_panel, LV_OBJ_FLAG_HIDDEN)) {
        fmt_db(a, sizeof(a), to_spl(l.las));
        snprintf(buf, sizeof(buf), "Live LAS  %s dB(A)", a);
        lv_label_set_text(ui.cal_live, buf);

        // Per-microphone levels: a large difference means one capsule is
        // blocked or faulty, which would spoil a calibration.
        fmt_db(a, sizeof(a), to_spl(l.mic1));
        fmt_db(b, sizeof(b), to_spl(l.mic2));
        snprintf(buf, sizeof(buf), "Mic 1  %s     Mic 2  %s", a, b);
        lv_label_set_text(ui.cal_mics, buf);
    }
}

lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t colour, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, colour, 0);
    lv_label_set_text(l, text);
    return l;
}

lv_obj_t *make_button(lv_obj_t *parent, const char *text, int w, int h, lv_event_cb_t cb, void *user)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, kColButton, 0);
    lv_obj_set_style_radius(btn, h / 2, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t *l = make_label(btn, &lv_font_montserrat_20, kColText, text);
    lv_obj_center(l);
    return btn;
}

void on_toggle_mode(lv_event_t *)
{
    ui.slow = !ui.slow;
    lv_label_set_text(ui.mode, ui.slow ? "dB(A)  SLOW" : "dB(A)  FAST");
    lv_label_set_text(ui.spec_corners.dba_cap, ui.slow ? "dB(A) SLOW" : "dB(A) FAST");
    lv_label_set_text(ui.hist_corners.dba_cap, ui.slow ? "dB(A) SLOW" : "dB(A) FAST");
    lv_label_set_text(ui.wf_corners.dba_cap, ui.slow ? "dB(A) SLOW" : "dB(A) FAST");
}

// ---------------------------------------------------------------------------
// Analyser (spectrum) screen
// ---------------------------------------------------------------------------
// Band dB SPL -> 0..1 for display, then square root as in Waveshare's demo
// so quieter bands still show. Used by the analyser and the waterfall.
void spec_levels01(const float *db, float *level)
{
    for (int b = 0; b < kSpecBands; ++b) {
        float v = (db[b] - kSpecMinDb) / (kSpecMaxDb - kSpecMinDb);
        v = v < 0 ? 0 : (v > 1 ? 1 : v);
        level[b] = sqrtf(v);
    }
}

void spec_timer_cb(lv_timer_t *)
{
    if (!g_spec_on) return;
    float db[kSpecBands], level[kSpecBands];
    if (ui.screen == Screen::Analyser) {
        {
            std::lock_guard<std::mutex> lock(g_spec_mutex);
            if (!g_spec_fresh) return;
            g_spec_fresh = false;
            memcpy(db, g_spec_db, sizeof(db));
        }
        spec_levels01(db, level);
        spectrum_view_set(ui.spec, level);
    } else if (ui.screen == Screen::Waterfall && ui.wf) {
        {
            std::lock_guard<std::mutex> lock(g_spec_mutex);
            if (!g_wf_fresh) return;
            g_wf_fresh = false;
            memcpy(db, g_wf_db, sizeof(db));
        }
        spec_levels01(db, level);
        waterfall_view_push(ui.wf, level);
    }
}

void show_screen(Screen next)
{
    // Screens that need the FFT are skipped if its buffer couldn't be allocated.
    if (next == Screen::Analyser && g_spec_ring == nullptr) next = Screen::History;
    if (next == Screen::Waterfall && (g_spec_ring == nullptr || ui.wf == nullptr)) next = Screen::Meter;

    const bool spec = next == Screen::Analyser || next == Screen::Waterfall;
    if (spec) {
        std::lock_guard<std::mutex> lock(g_spec_mutex);
        memset(g_spec_ring, 0, kSpecN * sizeof(float));  // no stale audio from last time
        g_spec_fresh = false;
        g_wf_fresh = false;
    }
    if (next == Screen::Waterfall) {
        waterfall_view_clear(ui.wf);
        g_wf_restart = true;
    }
    if (next == Screen::History) ui.hist_version = UINT32_MAX;  // redraw on the next update

    ui.screen = next;
    g_spec_on = spec;
    lv_obj_t *const views[kScreens] = {ui.main_view, ui.spec_view, ui.hist_view, ui.wf_view};
    for (int i = 0; i < kScreens; ++i) {
        if (i == (int)next) lv_obj_remove_flag(views[i], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(views[i], LV_OBJ_FLAG_HIDDEN);
    }
}

void on_show_screen(lv_event_t *e) { show_screen((Screen)(intptr_t)lv_event_get_user_data(e)); }

// Full-screen container for one screen.
lv_obj_t *make_view(lv_obj_t *scr)
{
    lv_obj_t *v = lv_obj_create(scr);
    lv_obj_remove_style_all(v);
    lv_obj_set_size(v, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_remove_flag(v, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(v, LV_OBJ_FLAG_CLICKABLE);
    return v;
}

// Bottom corners: dB(A) on the left (tap for Fast / Slow, shared with the
// meter screen), LAeq 15m on the right, as small seven-segment readouts with
// captions below. Bottom centre, above the battery icon: the button to the
// next screen.
void build_corners(lv_obj_t *v, CornerReadouts &c, const char *next_text, Screen next)
{
    // 30 x 56 px digits; "I88" is 85 px wide.
    slm::Seg7Geometry small;
    small.digit_w = 30;
    small.digit_h = 56;
    small.thick = 7;
    small.gap = 1;
    small.spacing = 9;
    small.dp_w = 7;
    small.dp_spacing = 4;
    const lv_color_t ghost = lv_color_hex(0x1C1C1C);

    c.dba = seg7_create(v, small, "I88");
    seg7_set_colors(c.dba, kColText, ghost);
    seg7_set_text(c.dba, "i--");
    lv_obj_align(c.dba, LV_ALIGN_BOTTOM_LEFT, 44, -66);
    lv_obj_add_flag(c.dba, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(c.dba, 24);
    lv_obj_add_event_cb(c.dba, on_toggle_mode, LV_EVENT_CLICKED, nullptr);
    c.dba_cap = make_label(v, &lv_font_montserrat_16, kColDim, ui.slow ? "dB(A) SLOW" : "dB(A) FAST");
    lv_obj_align_to(c.dba_cap, c.dba, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 6);

    c.leq = seg7_create(v, small, "I88");
    seg7_set_colors(c.leq, kColText, ghost);
    seg7_set_text(c.leq, "i--");
    lv_obj_align(c.leq, LV_ALIGN_BOTTOM_RIGHT, -44, -66);
    lv_obj_t *cap_r = make_label(v, &lv_font_montserrat_16, kColDim, "LAeq 15m");
    lv_obj_align_to(cap_r, c.leq, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 6);

    lv_obj_t *btn = make_button(v, next_text, 128, 52, on_show_screen, (void *)(intptr_t)next);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -54);
}

void build_spectrum_view(lv_obj_t *scr)
{
    lv_obj_t *v = make_view(scr);
    ui.spec_view = v;

    // 64 bars x 6 px = 384 px wide. Bars rise from y = 340, just above the
    // corner readouts (their tops are at y = 358).
    ui.spec = spectrum_view_create(v, 384, 300, kSpecBands);
    lv_obj_align(ui.spec, LV_ALIGN_TOP_MID, 0, 40);

    build_corners(v, ui.spec_corners, "History", Screen::History);

    lv_obj_add_flag(v, LV_OBJ_FLAG_HIDDEN);
    lv_timer_create(spec_timer_cb, kSpecFrameMs, nullptr);
}

void build_history_view(lv_obj_t *scr)
{
    lv_obj_t *v = make_view(scr);
    ui.hist_view = v;

    // 40 px label column + 90 bars x 4 px = 400 px wide; same height and
    // baseline as the analyser.
    ui.hist = history_view_create(v, 400, 300, kHistBars);
    lv_obj_align(ui.hist, LV_ALIGN_TOP_MID, 0, 40);

    build_corners(v, ui.hist_corners, "Waterfall", Screen::Waterfall);

    lv_obj_add_flag(v, LV_OBJ_FLAG_HIDDEN);
}

void build_waterfall_view(lv_obj_t *scr)
{
    lv_obj_t *v = make_view(scr);
    ui.wf_view = v;

    // Same position and size as the analyser: 64 bands x 6 px = 384 px wide,
    // 300 px tall, bottom edge just above the corner readouts.
    constexpr int32_t kW = 384, kH = 300, kTop = 40;
    ui.wf = waterfall_view_create(v, kW, kH, kSpecBands, kWfRowPx);
    if (ui.wf) {
        lv_obj_align(ui.wf, LV_ALIGN_TOP_MID, 0, kTop);
    } else {
        ESP_LOGE(TAG, "No PSRAM for the waterfall; screen disabled");
    }

    // Frequency marks above the waterfall. The bands are log-spaced from
    // kSpecLowHz to kSpecHighHz, so f sits at log(f/low)/log(high/low) of the width.
    struct Mark { double hz; const char *text; };
    const Mark marks[] = {{100, "100"}, {1000, "1k"}, {10000, "10k"}};
    for (const Mark &m : marks) {
        const int32_t x = (int32_t)lround(kW * log(m.hz / kSpecLowHz) / log(kSpecHighHz / kSpecLowHz));
        lv_obj_t *l = make_label(v, &lv_font_montserrat_16, kColDim, m.text);
        lv_obj_set_width(l, 50);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, (BSP_LCD_H_RES - kW) / 2 + x - 25, kTop - 22);
    }

    build_corners(v, ui.wf_corners, "dB meter", Screen::Meter);

    lv_obj_add_flag(v, LV_OBJ_FLAG_HIDDEN);
}

void on_cal_open(lv_event_t *)
{
    ui.cal_backup = g_cal_offset;
    update_cal_label();
    lv_obj_remove_flag(ui.cal_panel, LV_OBJ_FLAG_HIDDEN);
}

void on_cal_step(lv_event_t *e)
{
    const int tenths = (int)(intptr_t)lv_event_get_user_data(e);
    g_cal_offset += tenths / 10.0f;
    update_cal_label();
}

void on_cal_save(lv_event_t *)
{
    save_calibration();
    lv_obj_add_flag(ui.cal_panel, LV_OBJ_FLAG_HIDDEN);
}

void on_cal_cancel(lv_event_t *)
{
    g_cal_offset = ui.cal_backup;
    lv_obj_add_flag(ui.cal_panel, LV_OBJ_FLAG_HIDDEN);
}

void build_cal_panel(lv_obj_t *scr)
{
    lv_obj_t *p = lv_obj_create(scr);
    ui.cal_panel = p;
    lv_obj_set_size(p, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_center(p);
    lv_obj_set_style_bg_color(p, kColBg, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = make_label(p, &lv_font_montserrat_24, kColText, "Calibration");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 60);

    lv_obj_t *help = make_label(p, &lv_font_montserrat_16, kColDim,
                                "Place beside a reference meter or in a\n"
                                "94 dB calibrator and adjust until the\n"
                                "live reading matches.");
    lv_obj_set_style_text_align(help, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(help, LV_ALIGN_TOP_MID, 0, 100);

    ui.cal_live = make_label(p, &lv_font_montserrat_24, kColGreen, "");
    lv_obj_align(ui.cal_live, LV_ALIGN_CENTER, 0, -56);

    ui.cal_mics = make_label(p, &lv_font_montserrat_16, kColDim, "");
    lv_obj_align(ui.cal_mics, LV_ALIGN_CENTER, 0, -24);

    ui.cal_value = make_label(p, &lv_font_montserrat_20, kColText, "");
    lv_obj_align(ui.cal_value, LV_ALIGN_CENTER, 0, 10);

    struct Step { const char *text; int tenths; };
    const Step steps[] = {{"-1", -10}, {"-0.1", -1}, {"+0.1", 1}, {"+1", 10}};
    for (int i = 0; i < 4; ++i) {
        lv_obj_t *b = make_button(p, steps[i].text, 84, 56, on_cal_step, (void *)(intptr_t)steps[i].tenths);
        lv_obj_align(b, LV_ALIGN_CENTER, -141 + i * 94, 60);
    }

    lv_obj_t *save = make_button(p, "Save", 130, 56, on_cal_save, nullptr);
    lv_obj_set_style_bg_color(save, lv_color_hex(0x1F6F43), 0);
    lv_obj_align(save, LV_ALIGN_CENTER, -72, 140);
    lv_obj_t *cancel = make_button(p, "Cancel", 130, 56, on_cal_cancel, nullptr);
    lv_obj_align(cancel, LV_ALIGN_CENTER, 72, 140);

    lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
}

void build_ui()
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, kColBg, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Main (sound level meter) view: a transparent full-screen container, one
    // of three (meter, analyser, history) that show_screen() switches between.
    lv_obj_t *mv = make_view(scr);
    ui.main_view = mv;

    // Gauge: 270 degree arc, opening at the bottom.
    ui.arc = lv_arc_create(mv);
    lv_obj_set_size(ui.arc, 440, 440);
    lv_obj_center(ui.arc);
    lv_arc_set_rotation(ui.arc, 135);
    lv_arc_set_bg_angles(ui.arc, 0, 270);
    lv_arc_set_range(ui.arc, (int32_t)(kArcMinDb * 10), (int32_t)(kArcMaxDb * 10));
    lv_arc_set_value(ui.arc, (int32_t)(kArcMinDb * 10));
    lv_obj_remove_style(ui.arc, nullptr, LV_PART_KNOB);
    lv_obj_remove_flag(ui.arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(ui.arc, 26, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ui.arc, 26, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui.arc, kColTrack, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui.arc, kColGreen, LV_PART_INDICATOR);

    /*lv_obj_t *lo = make_label(scr, &lv_font_montserrat_16, kColDim, "30");
    lv_obj_align(lo, LV_ALIGN_CENTER, -140, 172);
    lv_obj_t *hi = make_label(scr, &lv_font_montserrat_16, kColDim, "130");
    lv_obj_align(hi, LV_ALIGN_CENTER, 140, 172);*/

    // Vertical layout, offsets from the screen centre (reading 100 px tall):
    //   OVERLOAD -150 (-162..-138) | reading -60 (-110..-10) | mode 12 (0..24)
    //   | LAeq digits 60 (38..82) | LAeq caption 96 (88..104)
    //   | Analyser/Cal buttons 160 (134..186)
    //   | battery icon at the bottom edge
    ui.over = make_label(mv, &lv_font_montserrat_24, kColRed, "OVERLOAD");
    lv_obj_align(ui.over, LV_ALIGN_CENTER, 0, -150);
    lv_obj_add_flag(ui.over, LV_OBJ_FLAG_HIDDEN);

    // Big reading; tap it to switch Fast / Slow.
    // Seven-segment readout: narrow "1" cell for hundreds, then "88.8".
    slm::Seg7Geometry seg;  // defaults: 54 x 100 digits, 12 px segments
    ui.value = seg7_create(mv, seg, "I88");
    seg7_set_colors(ui.value, kColText, lv_color_hex(0x1C1C1C));
    seg7_set_text(ui.value, "i--");
    lv_obj_align(ui.value, LV_ALIGN_CENTER, 0, -60);
    lv_obj_add_flag(ui.value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(ui.value, 30);
    lv_obj_add_event_cb(ui.value, on_toggle_mode, LV_EVENT_CLICKED, nullptr);

    ui.mode = make_label(mv, &lv_font_montserrat_24, kColDim, "dB(A)  FAST");
    lv_obj_align(ui.mode, LV_ALIGN_CENTER, 0, 12);

    // LAeq 15m: smaller seven-segment readout (24 x 44 px digits, "I88" is
    // 68 px wide), centred, with its caption centred below.
    slm::Seg7Geometry leq_geo;
    leq_geo.digit_w = 24;
    leq_geo.digit_h = 44;
    leq_geo.thick = 6;
    leq_geo.gap = 1;
    leq_geo.spacing = 7;
    ui.leq = seg7_create(mv, leq_geo, "I88");
    seg7_set_colors(ui.leq, kColText, lv_color_hex(0x1C1C1C));
    seg7_set_text(ui.leq, "i--");
    lv_obj_align(ui.leq, LV_ALIGN_CENTER, 0, 60);
    lv_obj_t *leq_cap = make_label(mv, &lv_font_montserrat_16, kColDim, "LAeq 15m");
    lv_obj_align(leq_cap, LV_ALIGN_CENTER, 0, 96);

    lv_obj_t *analyser = make_button(mv, "Analyser", 120, 52, on_show_screen, (void *)(intptr_t)Screen::Analyser);
    lv_obj_align(analyser, LV_ALIGN_CENTER, -66, 160);
    lv_obj_t *cal = make_button(mv, "Cal", 120, 52, on_cal_open, nullptr);
    lv_obj_align(cal, LV_ALIGN_CENTER, 66, 160);

    build_spectrum_view(scr);
    build_history_view(scr);
    build_waterfall_view(scr);

    // Battery icon, bottom centre in the gap of the arc, dimmed to 33 %
    // (opacity over the black background = brightness).
    ui.battery = battery_icon_create(scr);
    lv_obj_align(ui.battery, LV_ALIGN_BOTTOM_MID, 0, -18);
    // 84 / 255 = 33 %. Must be opa_layered: plain `opa` is only applied by
    // LVGL's built-in widgets, not by this icon's custom drawing.
    lv_obj_set_style_opa_layered(ui.battery, 84, 0);

    build_cal_panel(scr);

    ui.power_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(ui.power_overlay);
    lv_obj_set_size(ui.power_overlay, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_style_bg_color(ui.power_overlay, kColBg, 0);
    lv_obj_set_style_bg_opa(ui.power_overlay, LV_OPA_COVER, 0);
    ui.power_label = make_label(ui.power_overlay, &lv_font_montserrat_24, kColText, "");
    lv_obj_center(ui.power_label);
    lv_obj_add_flag(ui.power_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_timer_create(ui_timer_cb, 100, nullptr);
}

}  // namespace

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load_calibration();

    // Microphones first, while internal DMA RAM is still plentiful.
    esp_codec_dev_handle_t mic = open_microphones();

    g_spec_ring = static_cast<float *>(heap_caps_calloc(kSpecN, sizeof(float), MALLOC_CAP_SPIRAM));

    lv_display_t *disp = bsp_display_start();
    if (disp == nullptr) {
        ESP_LOGE(TAG, "Display init failed");
    } else {
        bsp_display_backlight_on();
        bsp_display_lock(UINT32_MAX);
        build_ui();
#if CONFIG_SLM_AUTO_ROTATE
        auto_rotate_start();
#endif
        bsp_display_unlock();
    }

    // Audio on core 1 at high priority so I2S never overruns while LVGL renders.
    // Without microphones the UI keeps showing "Microphone not available".
    if (mic) xTaskCreatePinnedToCore(audio_task, "audio", 6 * 1024, mic, 10, nullptr, 1);
    // FFT runs only while the spectrum screen is shown (~2 ms per frame).
    if (g_spec_ring) xTaskCreatePinnedToCore(spectrum_task, "spectrum", 4 * 1024, nullptr, 4, nullptr, 1);
    else ESP_LOGE(TAG, "No PSRAM for spectrum buffer; analyser disabled");
    battery_start();
}
