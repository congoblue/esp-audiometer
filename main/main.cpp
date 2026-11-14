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

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "esp_codec_dev.h"
#include "lvgl.h"

#include "auto_rotate.hpp"
#include "level_meter.hpp"
#include "seg7_widget.hpp"
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
// Audio task
// ---------------------------------------------------------------------------
int32_t s_raw[kBlockFrames * 2];
float s_left[kBlockFrames];
float s_right[kBlockFrames];

void audio_task(void *)
{
    esp_codec_dev_handle_t mic = bsp_audio_codec_microphone_init();
    if (mic == nullptr) {
        ESP_LOGE(TAG, "ES7210 microphone init failed");
        vTaskDelete(nullptr);
        return;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = kSampleRate;
    fs.channel = 2;           // MIC1 = left slot, MIC2 = right slot
    fs.bits_per_sample = 32;  // 24-bit ADC data, MSB-aligned in 32-bit slots
    if (esp_codec_dev_open(mic, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to open microphone codec");
        vTaskDelete(nullptr);
        return;
    }
    esp_codec_dev_set_in_gain(mic, kMicGainDb);
    ESP_LOGI(TAG, "Microphones running: %" PRIu32 " Hz, 2 ch, 32 bit, PGA %.1f dB",
             kSampleRate, kMicGainDb);
    g_audio_ok = true;

    static slm::LevelMeter meter(kSampleRate);
    constexpr float kScale = 1.0f / 2147483648.0f;
    int64_t last_log = 0;

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
struct Ui {
    lv_obj_t *arc;
    lv_obj_t *value;
    lv_obj_t *mode;
    lv_obj_t *over;
    lv_obj_t *leq;
    lv_obj_t *maxmin;
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
        lv_label_set_text(ui.leq, "Microphone not available");
        lv_label_set_text(ui.maxmin, "");
        return;
    }

    const float main_db = to_spl(ui.slow ? l.las : l.laf);
    char buf[128], a[16], b[16], c[16];

    fmt_seg7(buf, sizeof(buf), main_db);
    seg7_set_text(ui.value, buf);

    const float clamped = main_db < kArcMinDb ? kArcMinDb : (main_db > kArcMaxDb ? kArcMaxDb : main_db);
    lv_arc_set_value(ui.arc, (int32_t)lroundf(clamped * 10.0f));
    lv_obj_set_style_arc_color(ui.arc, level_colour(main_db), LV_PART_INDICATOR);

    fmt_db(a, sizeof(a), to_spl(l.laeq));
    snprintf(buf, sizeof(buf), "LAeq 15m %s", a);  // rolling 15-minute window
    lv_label_set_text(ui.leq, buf);

    fmt_db(b, sizeof(b), to_spl(l.lafmax));
    fmt_db(c, sizeof(c), to_spl(l.lafmin));
    snprintf(buf, sizeof(buf), "Max %s   Min %s", b, c);
    lv_label_set_text(ui.maxmin, buf);

    if (l.clipping) lv_obj_remove_flag(ui.over, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(ui.over, LV_OBJ_FLAG_HIDDEN);

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
}

void on_reset(lv_event_t *) { g_reset_request = true; }

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

    // Gauge: 270 degree arc, opening at the bottom.
    ui.arc = lv_arc_create(scr);
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
    //   OVERLOAD -100 | reading -12 (-62..38) | mode 54 | LAeq 88 | Max/Min 114
    //   | Reset/Cal buttons 160 | battery icon at the bottom edge
    ui.over = make_label(scr, &lv_font_montserrat_24, kColRed, "OVERLOAD");
    lv_obj_align(ui.over, LV_ALIGN_CENTER, 0, -100);
    lv_obj_add_flag(ui.over, LV_OBJ_FLAG_HIDDEN);

    // Big reading; tap it to switch Fast / Slow.
    // Seven-segment readout: narrow "1" cell for hundreds, then "88.8".
    slm::Seg7Geometry seg;  // defaults: 54 x 100 digits, 12 px segments
    ui.value = seg7_create(scr, seg, "I88");
    seg7_set_colors(ui.value, kColText, lv_color_hex(0x1C1C1C));
    seg7_set_text(ui.value, "i--");
    lv_obj_align(ui.value, LV_ALIGN_CENTER, 0, -12);
    lv_obj_add_flag(ui.value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(ui.value, 30);
    lv_obj_add_event_cb(ui.value, on_toggle_mode, LV_EVENT_CLICKED, nullptr);

    ui.mode = make_label(scr, &lv_font_montserrat_24, kColDim, "dB(A)  FAST");
    lv_obj_align(ui.mode, LV_ALIGN_CENTER, 0, 54);

    ui.leq = make_label(scr, &lv_font_montserrat_20, kColText, "");
    lv_obj_align(ui.leq, LV_ALIGN_CENTER, 0, 88);

    ui.maxmin = make_label(scr, &lv_font_montserrat_20, kColText, "");
    lv_obj_align(ui.maxmin, LV_ALIGN_CENTER, 0, 114);

    lv_obj_t *reset = make_button(scr, "Reset", 120, 52, on_reset, nullptr);
    lv_obj_align(reset, LV_ALIGN_CENTER, -66, 160);
    lv_obj_t *cal = make_button(scr, "Cal", 120, 52, on_cal_open, nullptr);
    lv_obj_align(cal, LV_ALIGN_CENTER, 66, 160);

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
    xTaskCreatePinnedToCore(audio_task, "audio", 6 * 1024, nullptr, 10, nullptr, 1);
    battery_start();
}
