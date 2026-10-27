// Accelerometer-driven particle simulation and snapshot publisher.
#include "fluid_service.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_attr.h"

#include "orientation_service.h"

#define SIM_W FLUID_SIM_WIDTH
#define SIM_H FLUID_SIM_HEIGHT
#define SIM_PIXELS FLUID_SIM_PIXELS

// Tuned down for ESP32 runtime to avoid watchdog stalls.
#define PARTICLE_COUNT 260
#define SIM_DT 0.12f
#define FLUID_SUBSTEPS_DEFAULT 2
#define FLUID_SUBSTEPS_MIN 1
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
#define FLUID_SIM_PERIOD_MS 33
#define FLUID_TASK_STACK 4096
#define FLUID_TASK_PRIO 4
#define FLUID_TASK_CORE 1

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
static EXT_RAM_BSS_ATTR uint8_t s_mask_render[SIM_PIXELS];
static portMUX_TYPE s_mask_mux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t s_fluid_task = NULL;
static volatile bool s_fluid_task_stop = false;
static StaticSemaphore_t s_fluid_stopped_buffer;
static SemaphoreHandle_t s_fluid_stopped = NULL;

// ------------------------------------------------------------------------
// Utilities
// ------------------------------------------------------------------------
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
    memset(s_mask_render, 0, sizeof(s_mask_render));
    s_mask = s_mask_a;
    s_mask_tmp = s_mask_b;

    seed_blob((float)(SIM_W / 2), (float)(SIM_H / 2), 8.0f);
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
    // Build 3x3 neighborhood hit counts around occupied particle pixels.
    // This is equivalent to the prior full-image blur+threshold pass but
    // scales with particle count instead of SIM_W*SIM_H*9 work per frame.
    memset(s_mask_tmp, 0, SIM_PIXELS);

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        int ix = clampi((int)s_x[i], 0, SIM_W - 1);
        int iy = clampi((int)s_y[i], 0, SIM_H - 1);

        for (int yy = -1; yy <= 1; ++yy) {
            int ny = iy + yy;
            if ((unsigned)ny >= SIM_H) continue;
            int row = ny * SIM_W;
            for (int xx = -1; xx <= 1; ++xx) {
                int nx = ix + xx;
                if ((unsigned)nx >= SIM_W) continue;
                int idx = row + nx;
                if (s_mask_tmp[idx] < 255) {
                    s_mask_tmp[idx]++;
                }
            }
        }
    }

    for (int i = 0; i < SIM_PIXELS; ++i) {
        s_mask[i] = (s_mask_tmp[i] >= 2) ? 1 : 0;
    }
}

static void publish_mask_snapshot(void)
{
    taskENTER_CRITICAL(&s_mask_mux);
    memcpy(s_mask_render, s_mask, SIM_PIXELS);
    taskEXIT_CRITICAL(&s_mask_mux);
}

void fluid_service_copy_mask(uint8_t out[FLUID_SIM_PIXELS])
{
    if (!out) return;
    taskENTER_CRITICAL(&s_mask_mux);
    memcpy(out, s_mask_render, SIM_PIXELS);
    taskEXIT_CRITICAL(&s_mask_mux);
}

// ------------------------------------------------------------------------
// Orientation / gravity
// ------------------------------------------------------------------------
static void ensure_orientation(void)
{
    esp_err_t err = orientation_service_start();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Using shared orientation stream");
    } else {
        ESP_LOGW(TAG, "orientation unavailable; using neutral gravity: %s",
                 esp_err_to_name(err));
    }
}

static void sample_gravity_xy(float *out_gx, float *out_gy)
{
    if (out_gx) *out_gx = 0.0f;
    if (out_gy) *out_gy = 0.0f;
    if (!out_gx || !out_gy) return;

    mpu6500_sample_t s = {0};
    if (!orientation_service_read_sample(&s)) return;
    // Map board axes (x=down, y=right) to screen axes (x=right, y=down).
    float gx = s.ay_g * GRAVITY_SIGN_Y; // screen-right acceleration
    float gy = s.ax_g * GRAVITY_SIGN_X; // screen-down acceleration
    *out_gx = gx;
    *out_gy = gy;
}

static void fluid_step_once_with_substeps(int substeps)
{
    float gx = 0.0f;
    float gy = 0.0f;
    sample_gravity_xy(&gx, &gy);

    if (substeps < FLUID_SUBSTEPS_MIN) {
        substeps = FLUID_SUBSTEPS_MIN;
    }

    float hdt = SIM_DT / (float)substeps;
    for (int s = 0; s < substeps; ++s) {
        step_sub(hdt, gx, gy);
    }

    rasterize_particles();
    publish_mask_snapshot();
}

static void fluid_step_once(void)
{
    fluid_step_once_with_substeps(FLUID_SUBSTEPS_DEFAULT);
}

static void fluid_task_fn(void *arg)
{
    (void)arg;
    TickType_t period = pdMS_TO_TICKS(FLUID_SIM_PERIOD_MS);
    int substeps = FLUID_SUBSTEPS_DEFAULT;
    if (period < 1) period = 1;

    while (!s_fluid_task_stop) {
        TickType_t frame_start = xTaskGetTickCount();
        fluid_step_once_with_substeps(substeps);
        TickType_t elapsed = xTaskGetTickCount() - frame_start;

        if (elapsed >= period) {
            if (substeps > FLUID_SUBSTEPS_MIN) {
                substeps--;
                ESP_LOGW(TAG, "fluid over budget (%ums), reducing substeps to %d",
                         (unsigned)(elapsed * portTICK_PERIOD_MS), substeps);
            }
            // Never try to catch up forever; always yield so IDLE1 can run.
            vTaskDelay(1);
            continue;
        }

        if (substeps < FLUID_SUBSTEPS_DEFAULT && elapsed + 1 < period) {
            substeps++;
            ESP_LOGI(TAG, "fluid headroom recovered (%ums), restoring substeps to %d",
                     (unsigned)(elapsed * portTICK_PERIOD_MS), substeps);
        }

        vTaskDelay(period - elapsed);
    }

    s_fluid_task = NULL;
    xSemaphoreGive(s_fluid_stopped);
    vTaskDelete(NULL);
}

static void fluid_start_task(void)
{
    if (s_fluid_task) return;

    if (!s_fluid_stopped) {
        s_fluid_stopped = xSemaphoreCreateBinaryStatic(&s_fluid_stopped_buffer);
        if (!s_fluid_stopped) {
            ESP_LOGW(TAG, "fluid stop signal creation failed");
            return;
        }
    }
    (void)xSemaphoreTake(s_fluid_stopped, 0);

    s_fluid_task_stop = false;
    BaseType_t ok = xTaskCreatePinnedToCore(
        fluid_task_fn,
        "fluid_sim",
        FLUID_TASK_STACK,
        NULL,
        FLUID_TASK_PRIO,
        &s_fluid_task,
        FLUID_TASK_CORE
    );
    if (ok != pdPASS) {
        s_fluid_task = NULL;
        ESP_LOGW(TAG, "fluid task create failed on core %d", FLUID_TASK_CORE);
    } else {
        ESP_LOGI(TAG, "fluid task running on core %d", FLUID_TASK_CORE);
    }
}

static void fluid_stop_task(void)
{
    if (!s_fluid_task) return;

    s_fluid_task_stop = true;
    xSemaphoreTake(s_fluid_stopped, portMAX_DELAY);
    s_fluid_task_stop = false;
}

// ------------------------------------------------------------------------
// Public lifecycle
// ------------------------------------------------------------------------
void fluid_service_start(void)
{
    reset_sim();
    ensure_orientation();
    fluid_step_once();
    fluid_start_task();
}

void fluid_service_stop(void)
{
    fluid_stop_task();
    reset_sim();
}

void fluid_service_tick_fallback(void)
{
    // Keep the feature usable if task creation failed.
    if (!s_fluid_task) {
        fluid_step_once();
    }
}
