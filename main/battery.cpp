#include "battery.hpp"

#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "bsp/esp-bsp.h"

namespace {

const char *TAG = "battery";

// ---------------------------------------------------------------------------
// AXP2101 access
// ---------------------------------------------------------------------------
constexpr uint8_t kAxpAddr = 0x34;
constexpr uint8_t kRegStatus1 = 0x00;
constexpr uint8_t kRegStatus2 = 0x01;
constexpr uint8_t kRegGaugeCtrl = 0x18;   // bit3: fuel gauge enable
constexpr uint8_t kRegAdcCtrl = 0x30;     // bit0: battery voltage ADC, bit1: TS pin ADC
constexpr uint8_t kRegBatDetCtrl = 0x68;  // bit0: battery detection enable
constexpr uint8_t kRegBatPercent = 0xA4;
constexpr uint32_t kPollMs = 2000;

i2c_master_dev_handle_t s_pmu = nullptr;
std::mutex s_mutex;
slm::BatteryStatus s_status;

esp_err_t pmu_read(uint8_t reg, uint8_t &value)
{
    return i2c_master_transmit_receive(s_pmu, &reg, 1, &value, 1, 50);
}

esp_err_t pmu_update_bits(uint8_t reg, uint8_t set, uint8_t clear)
{
    uint8_t v;
    esp_err_t err = pmu_read(reg, v);
    if (err != ESP_OK) return err;
    const uint8_t nv = (v | set) & ~clear;
    if (nv == v) return ESP_OK;
    const uint8_t buf[2] = {reg, nv};
    return i2c_master_transmit(s_pmu, buf, sizeof(buf), 50);
}

bool pmu_init()
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (bus == nullptr || i2c_master_probe(bus, kAxpAddr, 50) != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 not found; battery icon hidden");
        return false;
    }
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = kAxpAddr;
    cfg.scl_speed_hz = 400000;
    if (i2c_master_bus_add_device(bus, &cfg, &s_pmu) != ESP_OK) return false;

    // Same settings as Waveshare's AXP2101 example: battery detection and
    // fuel gauge on, battery voltage ADC on, and the TS (battery thermistor)
    // ADC off because this board has no thermistor. With TS left on, the
    // charger can treat the battery as out of temperature range.
    bool ok = pmu_update_bits(kRegBatDetCtrl, 0x01, 0) == ESP_OK;
    ok &= pmu_update_bits(kRegGaugeCtrl, 0x08, 0) == ESP_OK;
    ok &= pmu_update_bits(kRegAdcCtrl, 0x01, 0x02) == ESP_OK;
    if (!ok) ESP_LOGW(TAG, "AXP2101 configuration incomplete");
    return true;
}

void battery_task(void *)
{
    if (!pmu_init()) {
        vTaskDelete(nullptr);
        return;
    }
    while (true) {
        uint8_t s1, s2, pct;
        if (pmu_read(kRegStatus1, s1) == ESP_OK && pmu_read(kRegStatus2, s2) == ESP_OK &&
            pmu_read(kRegBatPercent, pct) == ESP_OK) {
            const slm::BatteryStatus st = slm::decode_axp2101(s1, s2, pct);
            bool changed;
            {
                std::lock_guard<std::mutex> lock(s_mutex);
                changed = !s_status.valid || st.present != s_status.present ||
                          st.charging != s_status.charging || st.percent != s_status.percent;
                s_status = st;
            }
            if (changed) {
                ESP_LOGI(TAG, "battery %s, %d %%, %s, USB %s", st.present ? "present" : "absent", st.percent,
                         st.charging ? "charging" : "not charging", st.usb_power ? "yes" : "no");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}

// ---------------------------------------------------------------------------
// Icon widget (48 x 24 px)
// ---------------------------------------------------------------------------
constexpr int32_t kIconW = 48, kIconH = 24;
constexpr int32_t kBodyW = 42;   // body outline 0..41, terminal nub 42..47
constexpr int32_t kBorder = 2;
constexpr int32_t kInset = 4;    // border + 2 px gap to the fill

struct IconState {
    slm::BatteryStatus status;
};

void rect(lv_layer_t *layer, const lv_area_t &a, lv_color_t bg, lv_opa_t bg_opa, int32_t radius,
          int32_t border = 0, lv_color_t border_colour = lv_color_black())
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = bg;
    d.bg_opa = bg_opa;
    d.radius = radius;
    d.border_width = border;
    d.border_color = border_colour;
    d.border_opa = border ? LV_OPA_COVER : LV_OPA_TRANSP;
    lv_draw_rect(layer, &d, &a);
}

void triangle(lv_layer_t *layer, lv_color_t c, int32_t ox, int32_t oy, const int32_t (&p)[6])
{
    lv_draw_triangle_dsc_t d;
    lv_draw_triangle_dsc_init(&d);
    d.color = c;
    d.opa = LV_OPA_COVER;
    for (int i = 0; i < 3; ++i) {
        d.p[i].x = ox + p[2 * i];
        d.p[i].y = oy + p[2 * i + 1];
    }
    lv_draw_triangle(layer, &d);
}

lv_color_t level_colour(int percent)
{
    switch (slm::battery_level(percent)) {
    case slm::BatteryLevel::Ok: return lv_color_hex(0x2ECC71);
    case slm::BatteryLevel::Low: return lv_color_hex(0xF1C40F);
    default: return lv_color_hex(0xE74C3C);
    }
}

void icon_draw_cb(lv_event_t *e)
{
    const auto *st = static_cast<IconState *>(lv_event_get_user_data(e));
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(lv_event_get_target_obj(e), &c);
    const int32_t x = c.x1, y = c.y1;
    const lv_color_t outline = lv_color_hex(0xBBBBBB);

    // Body outline and terminal nub.
    rect(layer, {x, y, x + kBodyW - 1, y + kIconH - 1}, lv_color_black(), LV_OPA_TRANSP, 4, kBorder, outline);
    rect(layer, {x + kBodyW, y + 7, x + kIconW - 1, y + kIconH - 8}, outline, LV_OPA_COVER, 2);

    const slm::BatteryStatus &s = st->status;
    if (s.present) {
        const int32_t inner_w = kBodyW - 2 * kInset;
        const int32_t fw = slm::battery_fill_width(s.percent, inner_w);
        if (fw > 0) {
            rect(layer, {x + kInset, y + kInset, x + kInset + fw - 1, y + kIconH - kInset - 1},
                 level_colour(s.percent), LV_OPA_COVER, 1);
        }
    }

    if (s.charging) {
        // Lightning bolt from two overlapping triangles, with a black outline
        // (drawn first, offset by 1 px each way) so it reads over any fill.
        static const int32_t top[6] = {25, 2, 13, 13, 22, 13};
        static const int32_t bot[6] = {19, 11, 29, 11, 17, 22};
        const int32_t offs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto &o : offs) {
            triangle(layer, lv_color_black(), x + o[0], y + o[1], top);
            triangle(layer, lv_color_black(), x + o[0], y + o[1], bot);
        }
        triangle(layer, lv_color_white(), x, y, top);
        triangle(layer, lv_color_white(), x, y, bot);
    }

    if (!s.present) {
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.color = lv_color_hex(0xE74C3C);
        d.width = 3;
        d.opa = LV_OPA_COVER;
        d.round_start = 1;
        d.round_end = 1;
        d.p1.x = x + 3;
        d.p1.y = y + kIconH - 3;
        d.p2.x = x + kIconW - 6;
        d.p2.y = y + 2;
        lv_draw_line(layer, &d);
    }
}

void icon_delete_cb(lv_event_t *e) { delete static_cast<IconState *>(lv_event_get_user_data(e)); }

}  // namespace

void battery_start()
{
    xTaskCreatePinnedToCore(battery_task, "battery", 3 * 1024, nullptr, 2, nullptr, 0);
}

slm::BatteryStatus battery_get()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    return s_status;
}

lv_obj_t *battery_icon_create(lv_obj_t *parent)
{
    auto *st = new IconState;
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(obj, kIconW, kIconH);
    lv_obj_set_user_data(obj, st);
    lv_obj_add_event_cb(obj, icon_draw_cb, LV_EVENT_DRAW_MAIN, st);
    lv_obj_add_event_cb(obj, icon_delete_cb, LV_EVENT_DELETE, st);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);  // shown once the PMU has been read
    return obj;
}

void battery_icon_set(lv_obj_t *icon, const slm::BatteryStatus &status)
{
    if (!status.valid) return;
    auto *st = static_cast<IconState *>(lv_obj_get_user_data(icon));
    const slm::BatteryStatus &old = st->status;
    const bool first = !old.valid;
    if (!first && old.present == status.present && old.charging == status.charging &&
        old.percent == status.percent) {
        return;
    }
    st->status = status;
    if (first) lv_obj_remove_flag(icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(icon);
}
