#include "led_modes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "led.h"
//#include "fft.h"
#include "led_layout.h"

static const char *TAG = "led_modes";

typedef struct {
    const char *name;
} led_mode_desc_t;

static const led_mode_desc_t s_modes[] = {
    { "idle"     },
    { "breathe"  },
    { "rainbow"  },
    { "beatwave" },
    { "planes"   },
};
static const int kModeCount = sizeof(s_modes) / sizeof(s_modes[0]);

static volatile int s_mode = 0;
static TaskHandle_t s_task = NULL;
static bool s_sync_enabled = true;
static uint8_t s_brightness = 255;

// Small helper to clamp mode indices
static inline int clamp_mode(int idx){
    if (idx < 0) return 0;
    if (idx >= kModeCount) return kModeCount - 1;
    return idx;
}

int led_modes_count(void){ return kModeCount; }
const char *led_modes_name(int idx){ return (idx >= 0 && idx < kModeCount) ? s_modes[idx].name : NULL; }
int led_modes_current(void){ return s_mode; }
void led_modes_set(int idx){ s_mode = clamp_mode(idx); }
void led_modes_set_sync(bool enabled){ s_sync_enabled = enabled; }
bool led_modes_sync_enabled(void){ return s_sync_enabled; }
void led_modes_set_brightness(uint8_t level){ s_brightness = level; }
uint8_t led_modes_get_brightness(void){ return s_brightness; }

// ----------------------------------------------------------------------
// Rendering helpers
// ----------------------------------------------------------------------

static inline float clamp01(float v){
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b){
    h = fmodf(h, 1.0f);
    if (h < 0.0f) h += 1.0f;
    float i = floorf(h * 6.0f);
    float f = h * 6.0f - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);
    int ii = ((int)i) % 6;
    float rf=0, gf=0, bf=0;
    switch (ii) {
        case 0: rf=v; gf=t; bf=p; break;
        case 1: rf=q; gf=v; bf=p; break;
        case 2: rf=p; gf=v; bf=t; break;
        case 3: rf=p; gf=q; bf=v; break;
        case 4: rf=t; gf=p; bf=v; break;
        case 5: rf=v; gf=p; bf=q; break;
        default: rf=v; gf=t; bf=p; break;
    }
    *r = (uint8_t)(clamp01(rf) * 255.0f);
    *g = (uint8_t)(clamp01(gf) * 255.0f);
    *b = (uint8_t)(clamp01(bf) * 255.0f);
}

// ---- Beat pulses ----
typedef struct {
    float pos;
    float amp;
    float width;
    float vel;
    bool  active;
} beat_pulse_t;

static beat_pulse_t s_pulses[4];
static float s_last_beat_sec = -100.0f;
static float s_fake_timer = 0.0f;
/*
static void beat_spawn(float conf){
    // Conf drives brightness a bit; clamp to sane range.
    float a = 0.35f + 0.65f * clamp01(conf);
    beat_pulse_t *slot = NULL;
    float weakest = 1e9f;
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        beat_pulse_t *p = &s_pulses[i];
        if (!p->active){ slot = p; break; }
        float score = p->amp;
        if (score < weakest){ weakest = score; slot = p; }
    }
    if (!slot) return;
    slot->pos = 0.0f;
    slot->amp = a;
    slot->width = 18.0f;
    slot->vel = 260.0f; // leds/sec
    slot->active = true;
}
*/
static void beat_step(float dt, size_t count){
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        beat_pulse_t *p = &s_pulses[i];
        if (!p->active) continue;
        p->pos += p->vel * dt;
        p->amp *= expf(-dt * 1.3f);
        if (p->pos > (float)count + p->width || p->amp < 0.01f){
            p->active = false;
        }
    }
}

static void render_idle(uint8_t *buf, size_t count){
    memset(buf, 0, count * 3);
    // leave strip off
}

static void render_breathe(uint8_t *buf, size_t count, float t_sec){
    float w = (sinf(t_sec * 0.8f) + 1.0f) * 0.5f; // 0..1
    uint8_t g = (uint8_t)(6 + w * 32);            // teal-ish low brightness
    uint8_t r = (uint8_t)(2 + w * 16);
    uint8_t b = (uint8_t)(8 + w * 64);
    for (size_t i = 0; i < count; ++i){
        size_t idx = i * 3;
        buf[idx + 0] = g;
        buf[idx + 1] = r;
        buf[idx + 2] = b;
    }
}

static void render_rainbow(uint8_t *buf, size_t count, float t_sec){
    const float speed = 0.07f;
    for (size_t i = 0; i < count; ++i){
        float h = fmodf((float)i / (float)count + t_sec * speed, 1.0f);
        uint8_t r, g, b;
        hsv_to_rgb(h, 0.7f, 0.22f, &r, &g, &b);
        size_t idx = i * 3;
        buf[idx + 0] = g;
        buf[idx + 1] = r;
        buf[idx + 2] = b;
    }
}

static void render_beat(uint8_t *buf, size_t count, float t_sec, float dt_sec){
    (void)t_sec;
    // Pull beat events from FFT if available; fall back to a slow thump.
    //fft_beat_event_t evt;
    bool got = false;//s_sync_enabled ? fft_receive_beat(&evt, 0) : false;
    if (got){
        //beat_spawn(evt.confidence);
        s_last_beat_sec = t_sec;
        s_fake_timer = 0.0f;
    } else {
        s_fake_timer += dt_sec;
        if (s_fake_timer > 1.25f){
            //beat_spawn(0.25f);
            s_fake_timer = 0.0f;
        }
    }

    beat_step(dt_sec, count);

    // Base faint fill
    for (size_t i = 0; i < count; ++i){
        size_t idx = i * 3;
        buf[idx + 0] = 0;
        buf[idx + 1] = 2;
        buf[idx + 2] = 6;
    }

    // Pulse contribution
    for (size_t i = 0; i < count; ++i){
        float acc_r = 0.0f, acc_g = 0.0f, acc_b = 0.0f;
        float pos = (float)i;
        for (size_t p = 0; p < sizeof(s_pulses)/sizeof(s_pulses[0]); ++p){
            const beat_pulse_t *bp = &s_pulses[p];
            if (!bp->active) continue;
            float d = fabsf(pos - bp->pos);
            float w = expf(-d / bp->width) * bp->amp;
            acc_r += w * 255.0f;
            acc_g += w * 40.0f;
            acc_b += w * 120.0f;
        }
        size_t idx = i * 3;
        int g = buf[idx + 0] + (int)acc_g;
        int r = buf[idx + 1] + (int)acc_r;
        int b = buf[idx + 2] + (int)acc_b;
        if (g > 255) g = 255;
        if (r > 255) r = 255;
        if (b > 255) b = 255;
        buf[idx + 0] = (uint8_t)g;
        buf[idx + 1] = (uint8_t)r;
        buf[idx + 2] = (uint8_t)b;
    }
}

static void render_planes(uint8_t *buf, size_t count, float t_sec){
    const float plane_speed = 0.6f; // meters per second along +Z
    const float plane_gap   = 0.8f; // meters between planes
    const float thickness   = 0.12f;
    const float height_span = 1.2f; // scale Y into 0..1 gradient

    float phase = fmodf(t_sec * plane_speed / plane_gap, 1.0f);
    float plane_z = (-plane_gap * 0.5f) + phase * plane_gap; // sweeping plane center

    for (size_t i = 0; i < count; ++i){
        led_point_t p;
        if (!led_layout_get((int)i, &p)) break;
        float dz = fabsf(p.z - plane_z);
        float w = expf(-dz / thickness);
        float h = (p.y + height_span * 0.5f) / height_span; // normalize height
        h = clamp01(h);
        uint8_t r, g, b;
        hsv_to_rgb(h, 0.6f, clamp01(w * 0.8f), &r, &g, &b);
        size_t idx = i * 3;
        buf[idx + 0] = g;
        buf[idx + 1] = r;
        buf[idx + 2] = b;
    }
}

static void led_modes_task(void *arg){
    (void)arg;
    static uint8_t frame[LED_STRIP_LENGTH * 3];
    const TickType_t period = pdMS_TO_TICKS(40); // ~25 Hz
    TickType_t last_tick = xTaskGetTickCount();
    TickType_t last_wake = last_tick;
    float t_sec = 0.0f;

    while (1){
        TickType_t now = xTaskGetTickCount();
        float dt_sec = (float)(now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        if (dt_sec < 0.0f) dt_sec = 0.0f;
        last_tick = now;
        t_sec += dt_sec;

        int mode = clamp_mode(s_mode);
        switch (mode){
            case 0: render_idle(frame, LED_STRIP_LENGTH); break;
            case 1: render_breathe(frame, LED_STRIP_LENGTH, t_sec); break;
            case 2: render_rainbow(frame, LED_STRIP_LENGTH, t_sec); break;
            case 3: render_beat(frame, LED_STRIP_LENGTH, t_sec, dt_sec); break;
            case 4: render_planes(frame, LED_STRIP_LENGTH, t_sec); break;
            default: render_idle(frame, LED_STRIP_LENGTH); break;
        }

        // Apply global brightness scale
        uint8_t bscale = s_brightness;
        if (bscale < 255){
            for (size_t i = 0; i < LED_STRIP_LENGTH * 3; ++i){
                frame[i] = (uint8_t)((frame[i] * (uint16_t)bscale) / 255u);
            }
        }

        (void)led_show_pixels(frame, LED_STRIP_LENGTH);
        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t led_modes_start(void){
    if (s_task) return ESP_OK;

    esp_err_t err = led_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_init failed: %d", err);
        return err;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(led_modes_task, "led_modes", 4096, NULL, 4, &s_task, tskNO_AFFINITY);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
