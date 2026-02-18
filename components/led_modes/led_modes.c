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
    { "off"     },
    { "solid"   },
    { "breathe" },
    { "scanner" },
    { "mirror"  },
    { "energy"  },
    { "blink"   },
    { "chase"   },
    { "twinkle" },
    { "glitch"  },
};
static const int kModeCount = sizeof(s_modes) / sizeof(s_modes[0]);

static volatile int s_mode = 0;
static volatile bool s_enabled = false;
static TaskHandle_t s_task = NULL;
static bool s_sync_enabled = true;
static uint8_t s_brightness = 96;
static uint8_t s_primary_r = 0;
static uint8_t s_primary_g = 180;
static uint8_t s_primary_b = 255;

#if configNUMBER_OF_CORES > 1
#define BG_TASK_CORE 0
#else
#define BG_TASK_CORE 0
#endif

#ifndef LED_CUSTOM_RENDER_PIXELS
#define LED_CUSTOM_RENDER_PIXELS LED_STRIP_LENGTH
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
void led_modes_set_primary_color(uint8_t r, uint8_t g, uint8_t b) {
    s_primary_r = r;
    s_primary_g = g;
    s_primary_b = b;
}
void led_modes_get_primary_color(uint8_t *r, uint8_t *g, uint8_t *b) {
    if (r) *r = s_primary_r;
    if (g) *g = s_primary_g;
    if (b) *b = s_primary_b;
}

static size_t custom_render_pixels(void)
{
    size_t n = LED_CUSTOM_RENDER_PIXELS;
    if (n == 0 || n > LED_STRIP_LENGTH) n = LED_STRIP_LENGTH;
    return n;
}

static TickType_t custom_period_ticks(void)
{
    uint32_t period_ms = LED_MODES_BASE_PERIOD_MS;
    uint32_t wire_us = (uint32_t)LED_STRIP_LENGTH * 30u + 300u;
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
    float pos_f = fmodf(t_sec * 20.0f, (float)count);
    if (pos_f < 0.0f) pos_f += (float)count;
    size_t pos = (size_t)pos_f;
    for (int t = 0; t < 10; ++t) {
        size_t idx = (size_t)(((int)pos - t + (int)count) % (int)count);
        float w = expf(-(float)t / 3.0f);
        led_set_pixel_rgb(buf, idx,
                          (uint8_t)((float)r * w),
                          (uint8_t)((float)g * w),
                          (uint8_t)((float)b * w));
    }
}

static void render_breathe(uint8_t *buf, size_t count, float t_sec,
                           uint8_t r, uint8_t g, uint8_t b)
{
    float w = 0.5f * (sinf(t_sec * 1.6f) + 1.0f);
    w = 0.15f + 0.85f * (w * w);
    render_fill(buf, count,
                (uint8_t)((float)r * w),
                (uint8_t)((float)g * w),
                (uint8_t)((float)b * w));
}

static void render_scanner(uint8_t *buf, size_t count, float t_sec,
                           uint8_t r, uint8_t g, uint8_t b)
{
    if (count == 0) return;
    float pos = fmodf(t_sec * 24.0f, (float)count);
    if (pos < 0.0f) pos += (float)count;
    for (size_t i = 0; i < count; ++i) {
        float d = fabsf((float)i - pos);
        float wrap = (float)count - d;
        if (wrap < d) d = wrap;
        float w = expf(-d / 5.0f);
        if (w < 0.01f) continue;
        led_set_pixel_rgb(buf, i,
                          (uint8_t)((float)r * w),
                          (uint8_t)((float)g * w),
                          (uint8_t)((float)b * w));
    }
}

static void render_mirror(uint8_t *buf, size_t count, float t_sec,
                          uint8_t r, uint8_t g, uint8_t b)
{
    if (count == 0) return;
    float head_a = fmodf(t_sec * 18.0f, (float)count);
    if (head_a < 0.0f) head_a += (float)count;
    float head_b = fmodf((float)count - head_a, (float)count);
    if (head_b < 0.0f) head_b += (float)count;
    for (size_t i = 0; i < count; ++i) {
        float da = fabsf((float)i - head_a);
        float db = fabsf((float)i - head_b);
        float wa = expf(-fminf(da, (float)count - da) / 4.0f);
        float wb = expf(-fminf(db, (float)count - db) / 4.0f);
        float w = wa + wb;
        if (w > 1.0f) w = 1.0f;
        if (w < 0.01f) continue;
        led_set_pixel_rgb(buf, i,
                          (uint8_t)((float)r * w),
                          (uint8_t)((float)g * w),
                          (uint8_t)((float)b * w));
    }
}

static float hashf(float x)
{
    return x - floorf(x);
}

static float noise2(float x, float y)
{
    return hashf(sinf(x * 12.9898f + y * 78.233f) * 43758.5453f);
}

static void render_twinkle(uint8_t *buf, size_t count, float t_sec,
                           uint8_t r, uint8_t g, uint8_t b)
{
    float frame = floorf(t_sec * 18.0f);
    for (size_t i = 0; i < count; ++i) {
        float n = noise2((float)i, frame);
        if (n < 0.90f) continue;
        float phase = t_sec * (6.0f + 10.0f * (n - 0.90f));
        float w = 0.5f + 0.5f * sinf(phase + (float)i * 0.13f);
        w *= w;
        led_set_pixel_rgb(buf, i,
                          (uint8_t)((float)r * w),
                          (uint8_t)((float)g * w),
                          (uint8_t)((float)b * w));
    }
}

static void render_glitch(uint8_t *buf, size_t count, float t_sec,
                          uint8_t r, uint8_t g, uint8_t b)
{
    if (count == 0) return;
    float head = fmodf(t_sec * 30.0f, (float)count);
    if (head < 0.0f) head += (float)count;
    float frame = floorf(t_sec * 26.0f);
    for (size_t i = 0; i < count; ++i) {
        float d = fabsf((float)i - head);
        float w = expf(-fminf(d, (float)count - d) / 8.0f);
        float n = noise2((float)i * 1.7f, frame);
        if (n > 0.992f) {
            w = 1.0f;
        } else if (n > 0.97f) {
            w = fmaxf(w, 0.65f);
        }
        if (w < 0.01f) continue;
        float white = (n > 0.992f) ? 1.0f : 0.0f;
        float rr = (float)r * w + 180.0f * white;
        float gg = (float)g * w + 180.0f * white;
        float bb = (float)b * w + 180.0f * white;
        if (rr > 255.0f) rr = 255.0f;
        if (gg > 255.0f) gg = 255.0f;
        if (bb > 255.0f) bb = 255.0f;
        led_set_pixel_rgb(buf, i, (uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
    }
}

static void render_energy(uint8_t *buf, size_t count, float t_sec,
                          uint8_t r, uint8_t g, uint8_t b)
{
    for (size_t i = 0; i < count; ++i) {
        float x = (float)i * 0.16f - t_sec * 8.5f;
        float carrier = 0.5f + 0.5f * sinf(x);
        float ripple = 0.5f + 0.5f * sinf(0.37f * x + 1.7f);
        float level = carrier * carrier * (0.65f + 0.35f * ripple);
        led_set_pixel_rgb(buf, i,
                          (uint8_t)((float)r * level),
                          (uint8_t)((float)g * level),
                          (uint8_t)((float)b * level));
    }
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
        uint8_t pr = s_primary_r;
        uint8_t pg = s_primary_g;
        uint8_t pb = s_primary_b;

        int mode = clamp_mode(s_mode);
        switch (mode) {
            case 0: render_fill(frame, count, 0, 0, 0); break;
            case 1: render_fill(frame, count, pr, pg, pb); break;
            case 2: render_breathe(frame, count, t_sec, pr, pg, pb); break;
            case 3: render_scanner(frame, count, t_sec, pr, pg, pb); break;
            case 4: render_mirror(frame, count, t_sec, pr, pg, pb); break;
            case 5: render_energy(frame, count, t_sec, pr, pg, pb); break;
            case 6: render_blink(frame, count, t_sec, 0.14f, 0.46f, pr, pg, pb); break;
            case 7: render_chase(frame, count, t_sec, pr, pg, pb); break;
            case 8: render_twinkle(frame, count, t_sec, pr, pg, pb); break;
            case 9: render_glitch(frame, count, t_sec, pr, pg, pb); break;
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
                                            NULL, 4, &s_task, BG_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
