// Accel-driven 2D fluid sim for the 128x64 OLED.
#include "fluid.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hw.h"
#include "mpu6500.h"
#include "oled.h"

#define W 64
#define H 32
#define N (W * H)

#define Q8             256
#define MAX_VEL        (4 * Q8) // ±4 cells/frame (Q8.8)
#define GRAVITY_SCALE  6
#define PRESSURE_ITERS 10

#define PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

// Axes can be flipped/swapped here if the board orientation differs.
#define GRAVITY_SIGN_X 1.0f
#define GRAVITY_SIGN_Y 1.0f

static const char *TAG = "fluid";

// Simulation fields (PSRAM)
static uint8_t *dens  = NULL;
static uint8_t *dens0 = NULL;
static int16_t *u     = NULL;
static int16_t *v     = NULL;
static int16_t *u0    = NULL;
static int16_t *v0    = NULL;
static int16_t *p     = NULL;
static int16_t *divv  = NULL;

// Framebuffer for shell-managed blits
static uint8_t s_fb[PANEL_W * PANEL_H / 8];

// IMU
static bool s_mpu_ready = false;
static mpu6500_t s_mpu = { .mux = portMUX_INITIALIZER_UNLOCKED };

// Input / UI state
static int s_emit_x = W / 2;
static int s_emit_y = H / 4;
static float s_emit_accum = 0.0f;
static float s_step_accum = 0.0f;
static uint64_t s_hint_until_us = 0;

// Gravity low-pass (float -> Q8.8)
static float s_gx_f = 0.0f;
static float s_gy_f = 0.0f;

// ------------------------------------------------------------------------
// Utilities
// ------------------------------------------------------------------------
static inline int idx(int x, int y) { return x + y * W; }

static inline int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int16_t clamp_q8(int32_t v)
{
    if (v > MAX_VEL) return MAX_VEL;
    if (v < -MAX_VEL) return -MAX_VEL;
    return (int16_t)v;
}

static inline int16_t lerp16(int16_t a, int16_t b, int16_t t)
{
    return (int16_t)(a + ((int32_t)(b - a) * t >> 8));
}

static inline void fb_clear(uint8_t *fb)
{
    memset(fb, 0, PANEL_W * PANEL_H / 8);
}

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int i = y * PANEL_W + x;
    fb[i >> 3] |= (uint8_t)(1u << (7 - (i & 7)));
}

// ------------------------------------------------------------------------
// Allocation / reset
// ------------------------------------------------------------------------
static bool alloc_fields(void)
{
    if (dens) return true; // already allocated

    dens  = (uint8_t *)heap_caps_malloc(N * sizeof(uint8_t), PSRAM_CAPS);
    dens0 = (uint8_t *)heap_caps_malloc(N * sizeof(uint8_t), PSRAM_CAPS);
    u     = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);
    v     = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);
    u0    = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);
    v0    = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);
    p     = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);
    divv  = (int16_t *)heap_caps_malloc(N * sizeof(int16_t), PSRAM_CAPS);

    if (!dens || !dens0 || !u || !v || !u0 || !v0 || !p || !divv) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        heap_caps_free(dens);  heap_caps_free(dens0);
        heap_caps_free(u);     heap_caps_free(v);
        heap_caps_free(u0);    heap_caps_free(v0);
        heap_caps_free(p);     heap_caps_free(divv);
        dens = dens0 = NULL;
        u = v = u0 = v0 = NULL;
        p = divv = NULL;
        return false;
    }
    return true;
}

static void zero_fields(void)
{
    if (!dens) return;
    memset(dens,  0, N);
    memset(dens0, 0, N);
    memset(u,     0, N * sizeof(int16_t));
    memset(v,     0, N * sizeof(int16_t));
    memset(u0,    0, N * sizeof(int16_t));
    memset(v0,    0, N * sizeof(int16_t));
    memset(p,     0, N * sizeof(int16_t));
    memset(divv,  0, N * sizeof(int16_t));
}

static void reset_sim(void)
{
    zero_fields();
    s_emit_x = W / 2;
    s_emit_y = H / 4;
    s_emit_accum = 0.0f;
    s_step_accum = 0.0f;
    s_gx_f = s_gy_f = 0.0f;
}

// ------------------------------------------------------------------------
// Sources / injection
// ------------------------------------------------------------------------
static void clamp_velocity_field(void)
{
    for (int i = 0; i < N; ++i) {
        u[i] = clamp_q8(u[i]);
        v[i] = clamp_q8(v[i]);
    }
}

static void zero_boundaries(void)
{
    for (int x = 0; x < W; ++x) {
        int it = idx(x, 0);
        int ib = idx(x, H - 1);
        u[it] = v[it] = 0;
        u[ib] = v[ib] = 0;
        dens[it] = dens[ib] = 0;
    }
    for (int y = 1; y < H - 1; ++y) {
        int il = idx(0, y);
        int ir = idx(W - 1, y);
        u[il] = v[il] = 0;
        u[ir] = v[ir] = 0;
        dens[il] = dens[ir] = 0;
    }
}

static void add_splat(int cx, int cy, int radius, int amount, int16_t push_x, int16_t push_y)
{
    if (!dens) return;
    int r2 = radius * radius;
    int x0 = clampi(cx - radius, 1, W - 2);
    int x1 = clampi(cx + radius, 1, W - 2);
    int y0 = clampi(cy - radius, 1, H - 2);
    int y1 = clampi(cy + radius, 1, H - 2);

    for (int y = y0; y <= y1; ++y) {
        int dy = y - cy;
        for (int x = x0; x <= x1; ++x) {
            int dx = x - cx;
            int d2 = dx * dx + dy * dy;
            if (d2 > r2) continue;

            int i = idx(x, y);
            int strength = amount;
            if (r2 > 0) {
                strength = (amount * (r2 - d2)) / r2; // taper toward edge
            }
            int val = dens[i] + strength;
            dens[i] = (uint8_t)(val > 255 ? 255 : val);

            u[i] = clamp_q8(u[i] + push_x);
            v[i] = clamp_q8(v[i] + push_y);
        }
    }
}

// ------------------------------------------------------------------------
// Core simulation steps
// ------------------------------------------------------------------------
static void add_gravity(int16_t gx, int16_t gy)
{
    for (int i = 0; i < N; ++i) {
        u[i] = clamp_q8(u[i] + gx * GRAVITY_SCALE);
        v[i] = clamp_q8(v[i] + gy * GRAVITY_SCALE);
    }
}

static void advect_velocity(void)
{
    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = idx(x, y);

            int16_t ux = u0[i];
            int16_t vy = v0[i];

            int16_t x0 = (int16_t)((x << 8) - ux);
            int16_t y0 = (int16_t)((y << 8) - vy);

            int xi = x0 >> 8;
            int yi = y0 >> 8;

            xi = clampi(xi, 0, W - 2);
            yi = clampi(yi, 0, H - 2);

            int16_t fx = (int16_t)(x0 & 0xFF);
            int16_t fy = (int16_t)(y0 & 0xFF);

            int i00 = idx(xi,     yi);
            int i10 = idx(xi + 1, yi);
            int i01 = idx(xi,     yi + 1);
            int i11 = idx(xi + 1, yi + 1);

            int16_t u0x = lerp16(u0[i00], u0[i10], fx);
            int16_t u1x = lerp16(u0[i01], u0[i11], fx);
            u[i] = clamp_q8(lerp16(u0x, u1x, fy));

            int16_t v0x = lerp16(v0[i00], v0[i10], fx);
            int16_t v1x = lerp16(v0[i01], v0[i11], fx);
            v[i] = clamp_q8(lerp16(v0x, v1x, fy));
        }
    }
    zero_boundaries();
}

static void advect_density(void)
{
    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = idx(x, y);

            int16_t ux = u[i];
            int16_t vy = v[i];

            int16_t x0 = (int16_t)((x << 8) - ux);
            int16_t y0 = (int16_t)((y << 8) - vy);

            int xi = x0 >> 8;
            int yi = y0 >> 8;

            xi = clampi(xi, 0, W - 2);
            yi = clampi(yi, 0, H - 2);

            int16_t fx = (int16_t)(x0 & 0xFF);
            int16_t fy = (int16_t)(y0 & 0xFF);

            int i00 = idx(xi,     yi);
            int i10 = idx(xi + 1, yi);
            int i01 = idx(xi,     yi + 1);
            int i11 = idx(xi + 1, yi + 1);

            int d00 = dens0[i00];
            int d10 = dens0[i10];
            int d01 = dens0[i01];
            int d11 = dens0[i11];

            int d0x = d00 + ((d10 - d00) * fx >> 8);
            int d1x = d01 + ((d11 - d01) * fx >> 8);
            int dxy = d0x + ((d1x - d0x) * fy >> 8);
            dens[i] = (uint8_t)clampi(dxy, 0, 255);
        }
    }
    zero_boundaries();
}

static void compute_divergence(void)
{
    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = idx(x, y);
            divv[i] = (int16_t)(
                (u[idx(x + 1, y)] - u[idx(x - 1, y)] +
                 v[idx(x, y + 1)] - v[idx(x, y - 1)])
                >> 1);
            p[i] = 0;
        }
    }
}

static void solve_pressure(void)
{
    for (int k = 0; k < PRESSURE_ITERS; ++k) {
        for (int y = 1; y < H - 1; ++y) {
            for (int x = 1; x < W - 1; ++x) {
                int i = idx(x, y);
                p[i] = (int16_t)(
                    (divv[i] +
                     p[idx(x - 1, y)] + p[idx(x + 1, y)] +
                     p[idx(x, y - 1)] + p[idx(x, y + 1)]) >> 2);
            }
        }
    }
}

static void project_velocity(void)
{
    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = idx(x, y);
            u[i] = clamp_q8(u[i] - ((p[idx(x + 1, y)] - p[idx(x - 1, y)]) >> 1));
            v[i] = clamp_q8(v[i] - ((p[idx(x, y + 1)] - p[idx(x, y - 1)]) >> 1));
        }
    }
    zero_boundaries();
}

static void fade_density(void)
{
    for (int i = 0; i < N; ++i) {
        dens[i] = (uint8_t)((dens[i] * 250) >> 8); // ~2-3% decay
    }
}

static void fluid_step(int16_t gx, int16_t gy)
{
    add_gravity(gx, gy);
    clamp_velocity_field();

    memcpy(u0, u, N * sizeof(int16_t));
    memcpy(v0, v, N * sizeof(int16_t));

    advect_velocity();
    clamp_velocity_field();

    compute_divergence();
    solve_pressure();
    project_velocity();
    clamp_velocity_field();

    memcpy(dens0, dens, N * sizeof(uint8_t));
    advect_density();
    fade_density();
}

// ------------------------------------------------------------------------
// IMU / gravity
// ------------------------------------------------------------------------
static void ensure_mpu(void)
{
    if (s_mpu_ready) return;

    if (hw_spi2_init_once() != ESP_OK) {
        ESP_LOGW(TAG, "spi2 init failed");
        return;
    }
    hw_spi2_idle_all_cs_high();

    const uint32_t freqs[] = { 8000000, 1000000 };
    for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
        spi_device_handle_t dev = NULL;
        esp_err_t add = hw_spi2_add_device((gpio_num_t)PIN_CS_IMU1, freqs[i], 0, 4, &dev);
        if (add != ESP_OK || !dev) continue;
        mpu6500_config_t cfg = { .spi = dev, .tag = "mpu_fluid" };
        if (mpu6500_init(&s_mpu, &cfg) == ESP_OK) {
            s_mpu_ready = true;
            break;
        }
    }
    if (!s_mpu_ready) {
        ESP_LOGW(TAG, "MPU init failed");
        return;
    }
    (void)mpu6500_start_sampler(&s_mpu, 120, 2, 1, 0);
    ESP_LOGI(TAG, "MPU online for fluid sim");
}

static void sample_gravity_q8(int16_t *out_gx, int16_t *out_gy)
{
    if (out_gx) *out_gx = 0;
    if (out_gy) *out_gy = 0;
    if (!s_mpu_ready || !out_gx || !out_gy) return;

    mpu6500_sample_t s = mpu6500_latest(&s_mpu);
    float gx = s.ax_g * GRAVITY_SIGN_X;
    float gy = s.ay_g * GRAVITY_SIGN_Y;
    float gz = s.az_g;

    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (mag > 0.01f) {
        gx /= mag;
        gy /= mag;
    }

    const float alpha = 0.18f; // low-pass gravity to keep things smooth
    s_gx_f = (1.0f - alpha) * s_gx_f + alpha * gx;
    s_gy_f = (1.0f - alpha) * s_gy_f + alpha * gy;

    int gxi = (int)lroundf(s_gx_f * Q8);
    int gyi = (int)lroundf(s_gy_f * Q8);

    *out_gx = (int16_t)clampi(gxi, -Q8 * 4, Q8 * 4);
    *out_gy = (int16_t)clampi(gyi, -Q8 * 4, Q8 * 4);
}

// ------------------------------------------------------------------------
// Rendering
// ------------------------------------------------------------------------
static const uint8_t BAYER4[4][4] = {
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5},
};

static void draw_density(uint8_t *fb, int x, int y, int w, int h)
{
    int scale_x = w / W;
    int scale_y = h / H;
    int scale = scale_x < scale_y ? scale_x : scale_y;
    if (scale <= 0) return;

    int ox = x + (w - W * scale) / 2;
    int oy = y + (h - H * scale) / 2;

    for (int cy = 0; cy < H; ++cy) {
        for (int cx = 0; cx < W; ++cx) {
            uint8_t d = dens[idx(cx, cy)];
            int px0 = ox + cx * scale;
            int py0 = oy + cy * scale;
            for (int yy = 0; yy < scale; ++yy) {
                int py = py0 + yy;
                for (int xx = 0; xx < scale; ++xx) {
                    int px = px0 + xx;
                    int thresh = BAYER4[py & 3][px & 3] << 4; // 0..240
                    if (d > thresh) {
                        fb_pset(fb, px, py);
                    }
                }
            }
        }
    }
}

// ------------------------------------------------------------------------
// Shell-facing API
// ------------------------------------------------------------------------
void fluid_app_init(void)
{
    if (!alloc_fields()) return;
    reset_sim();
    ensure_mpu();
    s_hint_until_us = esp_timer_get_time() + 4000000; // show hint for ~4 s

    // Seed a small blob so the screen isn't empty on entry.
    add_splat(W / 2, H / 2, 4, 140, 0, 0);
}

void fluid_app_deinit(void)
{
    // Keep allocations alive for quick re-entry; just quiet the sim.
    reset_sim();
}

void fluid_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;

    if (ev->type == INPUT_EVENT_LONG_PRESS) {
        if (ev->button == INPUT_BTN_D) {
            reset_sim();
            return;
        }
    }

    if (ev->type == INPUT_EVENT_PRESS) {
        switch (ev->button) {
            case INPUT_BTN_A: s_emit_x = clampi(s_emit_x - 1, 1, W - 2); break;
            case INPUT_BTN_B: s_emit_y = clampi(s_emit_y - 1, 1, H - 2); break;
            case INPUT_BTN_C: s_emit_x = clampi(s_emit_x + 1, 1, W - 2); break;
            case INPUT_BTN_D: s_emit_y = clampi(s_emit_y + 1, 1, H - 2); break;
            case INPUT_BTN_FAST_FALL:
                add_splat(s_emit_x, s_emit_y, 3, 220, 0, 0);
                break;
            default: break;
        }
        // Tiny dab so moving the emitter paints.
        if (ev->button == INPUT_BTN_A || ev->button == INPUT_BTN_B ||
            ev->button == INPUT_BTN_C || ev->button == INPUT_BTN_D) {
            add_splat(s_emit_x, s_emit_y, 2, 70, 0, 0);
        }
    }
}

void fluid_app_tick(float dt_sec)
{
    if (!dens) return;

    const float STEP_DT = 1.0f / 30.0f; // target 30 Hz sim step
    if (dt_sec <= 0.0f) dt_sec = STEP_DT;
    s_step_accum += dt_sec;

    // Regular emitter drip to keep the field alive.
    s_emit_accum += dt_sec;
    const float EMIT_PERIOD = 0.10f; // seconds
    while (s_emit_accum >= EMIT_PERIOD) {
        s_emit_accum -= EMIT_PERIOD;
        add_splat(s_emit_x, s_emit_y, 2, 80, 0, 0);
    }

    int16_t gx = 0, gy = 0;
    sample_gravity_q8(&gx, &gy);

    while (s_step_accum >= STEP_DT) {
        fluid_step(gx, gy);
        s_step_accum -= STEP_DT;
    }
}

void fluid_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb || !dens) return;
    fb_clear(fb);
    draw_density(fb, x, y, w, h);

    // Brief hint so users know the controls.
    uint64_t now = esp_timer_get_time();
    if (now < s_hint_until_us) {
        oled_draw_text3x5(fb, 2, 2, "A/B/C/D move, B+C ink, D long reset");
    }
}

// ------------------------------------------------------------------------
// Standalone runner (blocks until A is pressed)
// ------------------------------------------------------------------------
void fluid_run(void)
{
    fluid_app_init();
    uint64_t last = esp_timer_get_time();

    for (;;) {
        // Simple exit: any press of button A leaves.
        hw_button_id_t b = HW_BTN_NONE;
        if (hw_buttons_read(&b) == ESP_OK && b == HW_BTN_A) {
            break;
        }

        uint64_t now = esp_timer_get_time();
        float dt = (float)(now - last) / 1e6f;
        last = now;

        fluid_app_tick(dt);
        fluid_app_draw(s_fb, 0, 0, PANEL_W, PANEL_H);
        oled_blit_full(s_fb);

        vTaskDelay(pdMS_TO_TICKS(16));
    }
    fluid_app_deinit();
}
