// Auto-rotation: QMI8658 accelerometer -> OrientationDetector -> panel MADCTL
// rotation + touch remapping.
//
// Rotation is done by the display controller (MADCTL row/column exchange and
// mirroring through bsp_display_rotation_set), so LVGL renders exactly as
// before and nothing costs extra CPU. The panel is square (480x480), so the
// layout does not change. Touch points still arrive in the unrotated frame, so
// the LVGL pointer read callback is wrapped to rotate them back.

#include "auto_rotate.hpp"

#include <atomic>
#include <initializer_list>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"

#include "orientation.hpp"

namespace {

const char *TAG = "rotate";

// QMI8658 registers (datasheet rev. 0.9)
constexpr uint8_t kRegWhoAmI = 0x00;  // reads 0x05
constexpr uint8_t kRegCtrl1 = 0x02;   // serial interface config
constexpr uint8_t kRegCtrl2 = 0x03;   // accelerometer full scale + ODR
constexpr uint8_t kRegCtrl7 = 0x08;   // sensor enable
constexpr uint8_t kRegAxL = 0x35;     // AX_L .. AZ_H, little endian
constexpr uint8_t kRegReset = 0x60;
constexpr uint8_t kWhoAmIValue = 0x05;
constexpr float kLsbPerG = 16384.0f;  // +/-2 g range

constexpr uint32_t kPollMs = 50;

i2c_master_dev_handle_t s_imu = nullptr;
std::atomic<int> s_target{0};  // wanted orientation, written by the IMU task
int s_applied = 0;             // only touched in the LVGL task
lv_indev_read_cb_t s_orig_read_cb = nullptr;

esp_err_t imu_write(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_imu, buf, sizeof(buf), 50);
}

esp_err_t imu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_imu, &reg, 1, data, len, 50);
}

bool imu_init()
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (bus == nullptr) {
        ESP_LOGE(TAG, "BSP I2C bus not initialised");
        return false;
    }

    for (uint8_t addr : {uint8_t(0x6B), uint8_t(0x6A)}) {  // SA0 high / low
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;

        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addr;
        cfg.scl_speed_hz = 400000;
        if (i2c_master_bus_add_device(bus, &cfg, &s_imu) != ESP_OK) continue;

        uint8_t id = 0;
        if (imu_read(kRegWhoAmI, &id, 1) == ESP_OK && id == kWhoAmIValue) {
            imu_write(kRegReset, 0xB0);  // soft reset
            vTaskDelay(pdMS_TO_TICKS(20));
            bool ok = imu_write(kRegCtrl1, 0x40) == ESP_OK;  // register auto-increment
            ok &= imu_write(kRegCtrl2, 0x07) == ESP_OK;       // +/-2 g, ~62 Hz ODR
            ok &= imu_write(kRegCtrl7, 0x01) == ESP_OK;       // accelerometer on, gyro off
            vTaskDelay(pdMS_TO_TICKS(30));
            if (ok) {
                ESP_LOGI(TAG, "QMI8658 found at 0x%02X", addr);
                return true;
            }
        }
        i2c_master_bus_rm_device(s_imu);
        s_imu = nullptr;
    }
    ESP_LOGW(TAG, "QMI8658 not found; auto-rotation disabled");
    return false;
}

// Reads the accelerometer and returns the in-plane reading in the screen
// frame (+x = screen's right edge, +y = screen's top edge), in g.
bool read_accel_screen(float &sx, float &sy, float *az_out = nullptr)
{
    uint8_t b[6];
    if (imu_read(kRegAxL, b, sizeof(b)) != ESP_OK) return false;
    const float ax = int16_t(b[0] | (b[1] << 8)) / kLsbPerG;
    const float ay = int16_t(b[2] | (b[3] << 8)) / kLsbPerG;
    if (az_out) *az_out = int16_t(b[4] | (b[5] << 8)) / kLsbPerG;

    // On this board +X points to the screen's right edge and +Y to its top
    // edge (as in Waveshare's tilt demo); menuconfig options correct for a
    // different mounting.
    sx = ax;
    sy = ay;
#if CONFIG_SLM_IMU_MIRROR
    sx = -sx;
#endif
    return true;
}

// Detector output (0 = normal, as the UI was originally laid out) -> UI rotation.
int with_offset(int detected) { return (detected + CONFIG_SLM_IMU_ROTATION_OFFSET) & 3; }

// Rotates the panel. Call from the LVGL task or with the display lock held.
bool apply_rotation(int target)
{
    int panel = target;
#if CONFIG_SLM_PANEL_SWAP_90_270
    if (panel & 1) panel = 4 - panel;  // 90 <-> 270
#endif
    if (bsp_display_rotation_set(static_cast<bsp_display_rotation_t>(panel)) != ESP_OK) {
        ESP_LOGE(TAG, "Panel rotation failed");
        return false;
    }
    s_applied = target;
    lv_obj_invalidate(lv_screen_active());  // redraw everything in the new orientation
    ESP_LOGI(TAG, "Rotated to %d degrees", target * 90);
    return true;
}

void imu_task(void *arg)
{
    slm::OrientationDetector detector(static_cast<int>(reinterpret_cast<intptr_t>(arg)));
    uint32_t log_ms = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
        float sx, sy, az;
        if (!read_accel_screen(sx, sy, &az)) continue;
        const int o = with_offset(detector.update(sx, sy, kPollMs));
        s_target = o;

        log_ms += kPollMs;
        if (log_ms >= 2000) {
            log_ms = 0;
            ESP_LOGD(TAG, "accel x %.2f y %.2f z %.2f g -> orientation %d", sx, sy, az, o);
        }
    }
}

// Runs in the LVGL task: rotate the panel when the wanted orientation changes.
void apply_rotation_cb(lv_timer_t *)
{
    const int target = s_target;
    if (target != s_applied && !apply_rotation(target)) s_target = s_applied;
}

// Wraps the BSP's touch read callback and rotates the point to match the UI.
void rotated_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    s_orig_read_cb(indev, data);
    slm::rotate_touch_point(s_applied, BSP_LCD_H_RES - 1, data->point.x, data->point.y);
}

}  // namespace

void auto_rotate_start()
{
    for (lv_indev_t *indev = lv_indev_get_next(nullptr); indev; indev = lv_indev_get_next(indev)) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            s_orig_read_cb = lv_indev_get_read_cb(indev);
            if (s_orig_read_cb) lv_indev_set_read_cb(indev, rotated_touch_read_cb);
            break;
        }
    }
    if (s_orig_read_cb == nullptr) ESP_LOGW(TAG, "No touch input found; touch will not follow rotation");

    if (!imu_init()) {  // no IMU: fixed default orientation
        if (with_offset(0) != 0) apply_rotation(with_offset(0));
        return;
    }

    // Power-up: average a few readings and snap to the nearest orientation
    // straight away (no hold time), before LVGL draws the first frame. The
    // caller holds the display lock, so nothing renders meanwhile.
    float sum_x = 0, sum_y = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i) {
        float sx, sy;
        if (read_accel_screen(sx, sy)) {
            sum_x += sx;
            sum_y += sy;
            ++n;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    const int detected = n > 0 ? slm::OrientationDetector::initial(sum_x / n, sum_y / n) : 0;
    const int start = with_offset(detected);
    if (start != 0) apply_rotation(start);
    s_target = s_applied;
    ESP_LOGI(TAG, "Power-up orientation: %d degrees clockwise", s_applied * 90);

    lv_timer_create(apply_rotation_cb, 100, nullptr);
    xTaskCreatePinnedToCore(imu_task, "imu", 3 * 1024, reinterpret_cast<void *>(intptr_t(detected)),
                            2, nullptr, 0);
}
