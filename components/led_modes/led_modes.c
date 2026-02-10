#include "led_modes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "led.h"

static const char *TAG = "led_modes";

typedef struct {
    const char *name;
} led_mode_desc_t;

static const led_mode_desc_t s_modes[] = {
    { "off"          },
    { "solid_orange" },
    { "blink_white"  },
    { "blink_orange" },
    { "chase_white"  },
    { "chase_orange" },
    { "pulse_cyan"   },
};
static const int kModeCount = sizeof(s_modes) / sizeof(s_modes[0]);

static volatile int s_mode = 0;
static volatile bool s_enabled = false;
static TaskHandle_t s_task = NULL;
static bool s_sync_enabled = true;
static uint8_t s_brightness = 96;

#ifndef LED_CUSTOM_RENDER_PIXELS
#define LED_CUSTOM_RENDER_PIXELS 144
#endif

#ifndef LED_MODES_BASE_PERIOD_MS
#define LED_MODES_BASE_PERIOD_MS 33
#endif

#ifndef LED_MODES_IDLE_PERIOD_MS
#define LED_MODES_IDLE_PERIOD_MS 80
#endif

static inline int clamp_mode(int idx)
{
    if (idx < 0) return 0;
    if (idx >= kModeCount) return kModeCount - 1;
    return idx;
}

int led_modes_count(void) { return kModeCount; }
const char *led_modes_name(int idx) { return (idx >= 0 && idx < kModeCount) ? s_modes[idx].name : NULL; }
int led_modes_current(void) { return s_mode; }
void led_modes_set(int idx) { s_mode = clamp_mode(idx); }
void led_modes_enable(bool enabled) { s_enabled = enabled; }
bool led_modes_enabled(void) { return s_enabled; }
void led_modes_set_sync(bool enabled) { s_sync_enabled = enabled; }
bool led_modes_sync_enabled(void) { return s_sync_enabled; }
void led_modes_set_brightness(uint8_t level) { s_brightness = level; }
uint8_t led_modes_get_brightness(void) { return s_brightness; }

static size_t custom_render_pixels(void)
{
    size_t n = LED_CUSTOM_RENDER_PIXELS;
    if (n == 0 || n > LED_STRIP_LENGTH) n = LED_STRIP_LENGTH;
    return n;
}

static TickType_t custom_period_ticks(void)
{
    uint32_t period_ms = LED_MODES_BASE_PERIOD_MS;
    uint32_t wire_us = (uint32_t)LED_STRIP_PHYSICAL_LENGTH * 30u + 300u;
    uint32_t wire_ms = (wire_us + 2000u + 999u) / 1000u; // include a small margin
    if (period_ms < wire_ms) period_ms = wire_ms;
    TickType_t ticks = pdMS_TO_TICKS(period_ms);
    if (ticks == 0) ticks = 1;
    return ticks;
}

static void render_fill(uint8_t *buf, size_t count, uint8_t r, uint8_t g, uint8_t b)
{
    for (size_t i = 0; i < count; ++i) {
        led_set_pixel_rgb(buf, i, r, g, b);
    }
}

static void render_blink(uint8_t *buf, size_t count, float t_sec,
                         float on_sec, float off_sec,
                         uint8_t r, uint8_t g, uint8_t b)
{
    float cycle = on_sec + off_sec;
    if (cycle <= 0.001f) {
        render_fill(buf, count, r, g, b);
        return;
    }
    float phase = fmodf(t_sec, cycle);
    if (phase < 0.0f) phase += cycle;
    if (phase < on_sec) {
        render_fill(buf, count, r, g, b);
    } else {
        render_fill(buf, count, 0, 0, 0);
    }
}

static void render_chase(uint8_t *buf, size_t count, float t_sec,
                         uint8_t r, uint8_t g, uint8_t b)
{
    if (count == 0) return;
    float pos_f = fmodf(t_sec * 18.0f, (float)count);
    if (pos_f < 0.0f) pos_f += (float)count;
    size_t pos = (size_t)pos_f;
    led_set_pixel_rgb(buf, pos, r, g, b);
}

static void render_pulse_cyan(uint8_t *buf, size_t count, float t_sec)
{
    float w = 0.5f * (sinf(t_sec * 1.8f) + 1.0f);
    uint8_t r = 0;
    uint8_t g = (uint8_t)(8.0f + 42.0f * w);
    uint8_t b = (uint8_t)(12.0f + 74.0f * w);
    render_fill(buf, count, r, g, b);
}

static void led_modes_task(void *arg)
{
    (void)arg;
    static uint8_t frame[LED_STRIP_LENGTH * 3];
    const TickType_t period = custom_period_ticks();
    TickType_t idle_period = pdMS_TO_TICKS(LED_MODES_IDLE_PERIOD_MS);
    if (idle_period == 0) idle_period = 1;
    TickType_t last_wake = xTaskGetTickCount();
    float t_sec = 0.0f;
    const float dt_sec = (float)(period * portTICK_PERIOD_MS) / 1000.0f;
    bool was_enabled = false;

    while (1) {
        if (!s_enabled) {
            if (was_enabled) {
                memset(frame, 0, sizeof(frame));
                (void)led_show_pixels(frame, LED_STRIP_LENGTH);
                was_enabled = false;
            }
            vTaskDelay(idle_period);
            last_wake = xTaskGetTickCount();
            continue;
        }

        if (!was_enabled) {
            was_enabled = true;
            t_sec = 0.0f;
            last_wake = xTaskGetTickCount();
        }
        t_sec += dt_sec;

        size_t count = custom_render_pixels();
        memset(frame, 0, sizeof(frame));

        int mode = clamp_mode(s_mode);
        switch (mode) {
            case 0: render_fill(frame, count, 0, 0, 0); break;
            case 1: render_fill(frame, count, 200, 80, 0); break;
            case 2: render_blink(frame, count, t_sec, 0.16f, 0.55f, 160, 160, 160); break;
            case 3: render_blink(frame, count, t_sec, 0.20f, 0.65f, 200, 80, 0); break;
            case 4: render_chase(frame, count, t_sec, 180, 180, 180); break;
            case 5: render_chase(frame, count, t_sec, 200, 80, 0); break;
            case 6: render_pulse_cyan(frame, count, t_sec); break;
            default: render_fill(frame, count, 0, 0, 0); break;
        }

        uint8_t bscale = s_brightness;
        if (bscale < 255) {
            for (size_t i = 0; i < count * 3; ++i) {
                frame[i] = (uint8_t)((frame[i] * (uint16_t)bscale) / 255u);
            }
        }

        (void)led_show_pixels(frame, LED_STRIP_LENGTH);
        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t led_modes_start(void)
{
    if (s_task) return ESP_OK;

    esp_err_t err = led_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_init failed: %d", err);
        return err;
    }

    uint32_t period_ms = (uint32_t)(custom_period_ticks() * portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "custom renderer ready: pixels=%u period=%ums default_brightness=%u",
             (unsigned)custom_render_pixels(), (unsigned)period_ms, (unsigned)s_brightness);

    BaseType_t ok = xTaskCreatePinnedToCore(led_modes_task, "led_modes", 3072,
                                            NULL, 4, &s_task, tskNO_AFFINITY);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
