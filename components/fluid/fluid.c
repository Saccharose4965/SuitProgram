// Accel-driven particle fluid sim for the OLED using
// double density relaxation (DDR), kept close to goatedfluidsim.txt.
#include "fluid.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_attr.h"

#include "hw.h"
#include "mpu6500.h"
#include "oled.h"

#define SIM_W 128
#define SIM_H 64
#define SIM_PIXELS (SIM_W * SIM_H)

// Tuned down for ESP32 runtime to avoid watchdog stalls.
#define PARTICLE_COUNT 420
#define SIM_DT 0.12f
#define SUBSTEPS 2
#define RELAX_ITERS 1

#define INTERACTION_RADIUS 4.0f
#define INTERACTION_RADIUS2 (INTERACTION_RADIUS * INTERACTION_RADIUS)
#define INV_INTERACTION_RADIUS (1.0f / INTERACTION_RADIUS)
#define CELL_SIZE 4
#define INV_CELL_SIZE (1.0f / (float)CELL_SIZE)
#define GRID_W (((SIM_W) + CELL_SIZE - 1) / CELL_SIZE + 1)
#define GRID_H (((SIM_H) + CELL_SIZE - 1) / CELL_SIZE + 1)
#define GRID_CELLS (GRID_W * GRID_H)
#define MAX_NEIGHBORS_PER_PARTICLE 48

#define REST_DENSITY 5.0f
#define PRESSURE_K 0.07f
#define PRESSURE_K_NEAR 0.22f

// Board axes:
// - x points toward the bottom of the screen
// - y points toward the right of the screen
// Internally we simulate screen coordinates (x=right, y=down), so we map:
//   screen_right <- board_y, screen_down <- board_x
#define GRAVITY_SIGN_X -1.0f
#define GRAVITY_SIGN_Y -1.0f
#define GRAVITY_ACCEL 20.0f

static const char *TAG = "fluid";

// Particle state
static float s_x[PARTICLE_COUNT];
static float s_y[PARTICLE_COUNT];
static float s_px[PARTICLE_COUNT];
static float s_py[PARTICLE_COUNT];
static float s_vx[PARTICLE_COUNT];
static float s_vy[PARTICLE_COUNT];

// Uniform grid for neighbor search
static int s_head[GRID_CELLS];
static int s_next[PARTICLE_COUNT];

// 1-bit liquid mask (with one-pass connectivity blur)
static EXT_RAM_BSS_ATTR uint8_t s_mask_a[SIM_PIXELS];
static EXT_RAM_BSS_ATTR uint8_t s_mask_b[SIM_PIXELS];
static uint8_t *s_mask = s_mask_a;
static uint8_t *s_mask_tmp = s_mask_b;

// Framebuffer for standalone runner blits
static uint8_t s_fb[PANEL_W * PANEL_H / 8];

// IMU
static bool s_mpu_ready = false;
static mpu6500_t s_mpu = { .mux = portMUX_INITIALIZER_UNLOCKED };

// ------------------------------------------------------------------------
// Utilities
// ------------------------------------------------------------------------
static inline int sim_idx(int x, int y) { return x + y * SIM_W; }
static inline int grid_idx(int gx, int gy) { return gx + gy * GRID_W; }

static inline int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

static inline float frand_unit(void)
{
    return (float)(esp_random() & 0xFFFFu) / 65535.0f;
}

// Fast inverse sqrt to reduce expensive libm sqrtf calls in DDR hot loops.
static inline float fast_rsqrtf(float x)
{
    union {
        float f;
        uint32_t i;
    } u;
    u.f = x;
    u.i = 0x5f3759dfu - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - 0.5f * x * y * y);
    return y;
}

static inline void clamp_xy(float *x, float *y)
{
    *x = clampf(*x, 1.0f, (float)(SIM_W - 1));
    *y = clampf(*y, 1.0f, (float)(SIM_H - 1));
}

// ------------------------------------------------------------------------
// Particle setup / injection
// ------------------------------------------------------------------------
static void seed_blob(float cx, float cy, float radius)
{
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        float a = frand_unit() * 6.28318530718f;
        float r = sqrtf(frand_unit()) * radius;
        float x = cx + cosf(a) * r;
        float y = cy + sinf(a) * r;
        clamp_xy(&x, &y);
        s_x[i] = s_px[i] = x;
        s_y[i] = s_py[i] = y;
        s_vx[i] = 0.0f;
        s_vy[i] = 0.0f;
    }
}

static void reset_sim(void)
{
    memset(s_mask_a, 0, sizeof(s_mask_a));
    memset(s_mask_b, 0, sizeof(s_mask_b));
    s_mask = s_mask_a;
    s_mask_tmp = s_mask_b;

    seed_blob((float)(SIM_W / 2), (float)(SIM_H / 2), 16.0f);
}

// ------------------------------------------------------------------------
// Neighbor grid / DDR
// ------------------------------------------------------------------------
static void build_grid(void)
{
    for (int i = 0; i < GRID_CELLS; ++i) s_head[i] = -1;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        int gx = clampi((int)(s_x[i] * INV_CELL_SIZE), 0, GRID_W - 1);
        int gy = clampi((int)(s_y[i] * INV_CELL_SIZE), 0, GRID_H - 1);
        int g = grid_idx(gx, gy);
        s_next[i] = s_head[g];
        s_head[g] = i;
    }
}

static void double_density_relaxation(void)
{
    for (int iter = 0; iter < RELAX_ITERS; ++iter) {
        build_grid();

        for (int i = 0; i < PARTICLE_COUNT; ++i) {
            float dens = 0.0f;
            float dens_near = 0.0f;
            int gx = clampi((int)(s_x[i] * INV_CELL_SIZE), 0, GRID_W - 1);
            int gy = clampi((int)(s_y[i] * INV_CELL_SIZE), 0, GRID_H - 1);
            int density_neighbors = 0;

            for (int yy = -1; yy <= 1; ++yy) {
                int ny = gy + yy;
                if ((unsigned)ny >= GRID_H) continue;
                for (int xx = -1; xx <= 1; ++xx) {
                    int nx = gx + xx;
                    if ((unsigned)nx >= GRID_W) continue;

                    int h = s_head[grid_idx(nx, ny)];
                    while (h != -1) {
                        int j = h;
                        h = s_next[h];
                        if (j == i) continue;

                        float dx = s_x[j] - s_x[i];
                        float dy = s_y[j] - s_y[i];
                        float d2 = dx * dx + dy * dy;
                        if (d2 >= INTERACTION_RADIUS2) continue;

                        float inv_d = fast_rsqrtf(d2);
                        float q = (d2 * inv_d) * INV_INTERACTION_RADIUS;
                        float omq = 1.0f - q;
                        dens += omq * omq;
                        dens_near += omq * omq * omq;
                        density_neighbors++;
                        if (density_neighbors >= MAX_NEIGHBORS_PER_PARTICLE) goto density_done;
                    }
                }
            }
density_done:
            ;

            float pressure = PRESSURE_K * (dens - REST_DENSITY);
            float near_pressure = PRESSURE_K_NEAR * dens_near;
            float dx_sum = 0.0f;
            float dy_sum = 0.0f;
            int disp_neighbors = 0;

            for (int yy = -1; yy <= 1; ++yy) {
                int ny = gy + yy;
                if ((unsigned)ny >= GRID_H) continue;
                for (int xx = -1; xx <= 1; ++xx) {
                    int nx = gx + xx;
                    if ((unsigned)nx >= GRID_W) continue;

                    int h = s_head[grid_idx(nx, ny)];
                    while (h != -1) {
                        int j = h;
                        h = s_next[h];
                        if (j == i) continue;

                        float dx = s_x[j] - s_x[i];
                        float dy = s_y[j] - s_y[i];
                        float d2 = dx * dx + dy * dy;
                        if (d2 >= INTERACTION_RADIUS2 || d2 == 0.0f) continue;

                        float inv_d = fast_rsqrtf(d2);
                        float q = (d2 * inv_d) * INV_INTERACTION_RADIUS;
                        float omq = 1.0f - q;
                        float D = (pressure * omq + near_pressure * omq * omq) * 0.5f;

                        float disp_x = dx * inv_d * D;
                        float disp_y = dy * inv_d * D;

                        s_x[j] += disp_x;
                        s_y[j] += disp_y;
                        dx_sum -= disp_x;
                        dy_sum -= disp_y;
                        disp_neighbors++;
                        if (disp_neighbors >= MAX_NEIGHBORS_PER_PARTICLE) goto disp_done;
                    }
                }
            }
disp_done:
            ;

            s_x[i] += dx_sum;
            s_y[i] += dy_sum;
            clamp_xy(&s_x[i], &s_y[i]);
        }

    }
}

static void step_sub(float hdt, float gx, float gy)
{
    float ax = gx * GRAVITY_ACCEL;
    float ay = gy * GRAVITY_ACCEL;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        s_vx[i] += ax * hdt;
        s_vy[i] += ay * hdt;

        s_px[i] = s_x[i];
        s_py[i] = s_y[i];

        s_x[i] += s_vx[i] * hdt;
        s_y[i] += s_vy[i] * hdt;
        clamp_xy(&s_x[i], &s_y[i]);
    }

    double_density_relaxation();

    float inv = 1.0f / hdt;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        s_vx[i] = (s_x[i] - s_px[i]) * inv;
        s_vy[i] = (s_y[i] - s_py[i]) * inv;
    }
}

// ------------------------------------------------------------------------
// Rasterization
// ------------------------------------------------------------------------
static void rasterize_particles(void)
{
    memset(s_mask, 0, SIM_PIXELS);

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        int ix = clampi((int)s_x[i], 0, SIM_W - 1);
        int iy = clampi((int)s_y[i], 0, SIM_H - 1);
        s_mask[sim_idx(ix, iy)] = 1;
    }

    memset(s_mask_tmp, 0, SIM_PIXELS);
    for (int y = 0; y < SIM_H; ++y) {
        for (int x = 0; x < SIM_W; ++x) {
            int sum = 0;
            for (int yy = -1; yy <= 1; ++yy) {
                int ny = y + yy;
                if ((unsigned)ny >= SIM_H) continue;
                for (int xx = -1; xx <= 1; ++xx) {
                    int nx = x + xx;
                    if ((unsigned)nx >= SIM_W) continue;
                    if (s_mask[sim_idx(nx, ny)]) ++sum;
                }
            }
            s_mask_tmp[sim_idx(x, y)] = (sum >= 2) ? 1 : 0;
        }
    }

    uint8_t *tmp = s_mask;
    s_mask = s_mask_tmp;
    s_mask_tmp = tmp;
}

static bool view_transform(int x, int y, int w, int h, int *out_ox, int *out_oy, int *out_scale)
{
    int sx = w / SIM_W;
    int sy = h / SIM_H;
    int scale = (sx < sy) ? sx : sy;
    if (scale <= 0) return false;

    if (out_ox) *out_ox = x + (w - SIM_W * scale) / 2;
    if (out_oy) *out_oy = y + (h - SIM_H * scale) / 2;
    if (out_scale) *out_scale = scale;
    return true;
}

static void draw_mask(uint8_t *fb, int ox, int oy, int scale)
{
    for (int y = 0; y < SIM_H; ++y) {
        for (int x = 0; x < SIM_W; ++x) {
            if (!s_mask[sim_idx(x, y)]) continue;
            int px0 = ox + x * scale;
            int py0 = oy + y * scale;
            for (int yy = 0; yy < scale; ++yy) {
                int py = py0 + yy;
                for (int xx = 0; xx < scale; ++xx) {
                    fb_pset(fb, px0 + xx, py);
                }
            }
        }
    }
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
    (void)mpu6500_start_sampler(&s_mpu, 160, 2, 1, 0);
    ESP_LOGI(TAG, "MPU online for fluid sim");
}

static void sample_gravity_xy(float *out_gx, float *out_gy)
{
    if (out_gx) *out_gx = 0.0f;
    if (out_gy) *out_gy = 0.0f;
    if (!out_gx || !out_gy || !s_mpu_ready) return;

    mpu6500_sample_t s = mpu6500_latest(&s_mpu);
    // Map board axes (x=down, y=right) to screen axes (x=right, y=down).
    float gx = s.ay_g * GRAVITY_SIGN_Y; // screen-right acceleration
    float gy = s.ax_g * GRAVITY_SIGN_X; // screen-down acceleration
    *out_gx = gx;
    *out_gy = gy;
}

// ------------------------------------------------------------------------
// Shell-facing API
// ------------------------------------------------------------------------
void fluid_app_init(void)
{
    reset_sim();
    ensure_mpu();
}

void fluid_app_deinit(void)
{
    reset_sim();
}

void fluid_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;

    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_D) {
        reset_sim();
        return;
    }
}

void fluid_app_tick(float dt_sec)
{
    (void)dt_sec;
    float gx = 0.0f;
    float gy = 0.0f;
    sample_gravity_xy(&gx, &gy);

    float hdt = SIM_DT / (float)SUBSTEPS;
    for (int s = 0; s < SUBSTEPS; ++s) {
        step_sub(hdt, gx, gy);
    }

    rasterize_particles();
}

void fluid_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb) return;
    fb_clear(fb);

    int ox = 0, oy = 0, scale = 0;
    if (view_transform(x, y, w, h, &ox, &oy, &scale)) {
        draw_mask(fb, ox, oy, scale);
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
