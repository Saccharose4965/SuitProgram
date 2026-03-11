#include "led_modes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "led.h"
#include "led_layout.h"

static const char *TAG = "led_modes";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    const char *name;
} led_mode_desc_t;

static const led_mode_desc_t s_modes[] = {
    { "off"     },
    { "fill"    },
    { "breathe" },
    { "scanner" },
    { "mirror"  },
    { "energy"  },
    { "blink"   },
    { "chase"   },
    { "twinkle" },
    { "glitch"  },
    { "plane"   },
    { "prism"   },
    { "ring"    },
    { "contour" },
    { "orbit"   },
    { "aurora"  },
    { "vortex"  },
    { "helix"   },
    { "random"  },
};
static const int kModeCount = sizeof(s_modes) / sizeof(s_modes[0]);
static const int kShuffleModeIndex = (int)(sizeof(s_modes) / sizeof(s_modes[0])) - 1;

static volatile int s_mode = 0;
static volatile bool s_enabled = false;
static TaskHandle_t s_task = NULL;
static bool s_sync_enabled = true;
static uint8_t s_brightness = 96;
static volatile uint8_t s_speed_percent = 100;
static uint8_t s_primary_r = 0;
static uint8_t s_primary_g = 180;
static uint8_t s_primary_b = 255;
static uint8_t s_secondary_r = 255;
static uint8_t s_secondary_g = 64;
static uint8_t s_secondary_b = 0;
static volatile led_color_cycle_t s_color_cycle = LED_COLOR_CYCLE_STATIC;
static volatile led_color_style_t s_color_style = LED_COLOR_STYLE_MONO;
static volatile led_highlight_mode_t s_highlight_mode = LED_HIGHLIGHT_OFF;
static EXT_RAM_BSS_ATTR led_layout_config_t s_render_layout = {0};

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

#ifndef LED_MODES_SHUFFLE_INTERVAL_US
#define LED_MODES_SHUFFLE_INTERVAL_US (10LL * 1000000LL) // random mode switch interval
#endif

static inline int clamp_mode(int idx)
{
    if (idx < 0) return 0;
    if (idx >= kModeCount) return kModeCount - 1;
    return idx;
}

static bool mode_uses_layout(int mode)
{
    return (mode == 4) || (mode >= 10 && mode < kShuffleModeIndex);
}

static int pick_shuffle_mode(int prev_mode)
{
    const int first = 2; // skip off and fill
    const int last = kShuffleModeIndex - 1; // exclude shuffle itself
    if (last < first) {
        return 1;
    }
    const int count = last - first + 1;
    int picked = first + (int)(esp_random() % (uint32_t)count);
    if (count > 1 && picked == prev_mode) {
        picked = first + ((picked - first + 1 + (int)(esp_random() % (uint32_t)(count - 1))) % count);
    }
    return picked;
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
void led_modes_set_speed_percent(uint8_t percent) {
    if (percent < 10u) percent = 10u;
    if (percent > 250u) percent = 250u;
    s_speed_percent = percent;
}
uint8_t led_modes_get_speed_percent(void) { return s_speed_percent; }
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
void led_modes_set_secondary_color(uint8_t r, uint8_t g, uint8_t b) {
    s_secondary_r = r;
    s_secondary_g = g;
    s_secondary_b = b;
}
void led_modes_get_secondary_color(uint8_t *r, uint8_t *g, uint8_t *b) {
    if (r) *r = s_secondary_r;
    if (g) *g = s_secondary_g;
    if (b) *b = s_secondary_b;
}
void led_modes_color_cycle_set(led_color_cycle_t mode) {
    if (mode < LED_COLOR_CYCLE_STATIC || mode > LED_COLOR_CYCLE_BIOHAZARD) {
        mode = LED_COLOR_CYCLE_STATIC;
    }
    s_color_cycle = mode;
}
led_color_cycle_t led_modes_color_cycle_get(void) { return s_color_cycle; }
void led_modes_color_style_set(led_color_style_t style) {
    if (style < LED_COLOR_STYLE_MONO || style > LED_COLOR_STYLE_PALETTE) {
        style = LED_COLOR_STYLE_MONO;
    }
    s_color_style = style;
}
led_color_style_t led_modes_color_style_get(void) { return s_color_style; }
void led_modes_highlight_mode_set(led_highlight_mode_t mode) {
    if (mode < LED_HIGHLIGHT_OFF || mode > LED_HIGHLIGHT_PEAKS) {
        mode = LED_HIGHLIGHT_OFF;
    }
    s_highlight_mode = mode;
}
led_highlight_mode_t led_modes_highlight_mode_get(void) { return s_highlight_mode; }

static size_t custom_render_pixels(void)
{
    size_t n = led_layout_count();
    if (n == 0 || n > LED_STRIP_LENGTH) n = LED_STRIP_LENGTH;
    if (LED_CUSTOM_RENDER_PIXELS > 0 && LED_CUSTOM_RENDER_PIXELS < n) {
        n = LED_CUSTOM_RENDER_PIXELS;
    }
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

typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    float avg_x;
    float avg_y;
    float avg_z;
    led_point_t ring_center;
    float ring_radius;
    float radial_max;
    float ring_wave_max;
} layout_stats_t;

static inline float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

static inline float angular_distance(float a, float b)
{
    float d = a - b;
    while (d > (float)M_PI) d -= 2.0f * (float)M_PI;
    while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return fabsf(d);
}

static float ring_distance(const layout_stats_t *stats, const led_point_t *p)
{
    if (!stats || !p) return 0.0f;

    float dx = p->x - stats->ring_center.x;
    float dy = p->y - stats->ring_center.y;
    float dz = p->z - stats->ring_center.z;
    float planar = sqrtf(dx * dx + dy * dy);

    if (stats->ring_radius < 0.0001f) {
        return sqrtf(planar * planar + dz * dz);
    }

    float radial_delta = planar - stats->ring_radius;
    return sqrtf(radial_delta * radial_delta + dz * dz);
}

static void add_rgb(uint8_t *buf, size_t idx, float add_r, float add_g, float add_b)
{
    if (!buf) return;
    uint8_t cur_r = 0, cur_g = 0, cur_b = 0;
    led_get_pixel_rgb(buf, idx, &cur_r, &cur_g, &cur_b);
    float next_r = (float)cur_r + add_r;
    float next_g = (float)cur_g + add_g;
    float next_b = (float)cur_b + add_b;
    if (next_r > 255.0f) next_r = 255.0f;
    if (next_g > 255.0f) next_g = 255.0f;
    if (next_b > 255.0f) next_b = 255.0f;
    if (next_r < 0.0f) next_r = 0.0f;
    if (next_g < 0.0f) next_g = 0.0f;
    if (next_b < 0.0f) next_b = 0.0f;
    led_set_pixel_rgb(buf, idx, (uint8_t)next_r, (uint8_t)next_g, (uint8_t)next_b);
}

static void add_mixed_color(uint8_t *buf, size_t idx,
                            uint8_t pr, uint8_t pg, uint8_t pb,
                            uint8_t sr, uint8_t sg, uint8_t sb,
                            float primary_w, float secondary_w, float white_w)
{
    float white_scale = (s_highlight_mode == LED_HIGHLIGHT_PEAKS) ? 0.55f : 0.0f;
    add_rgb(buf, idx,
            primary_w * (float)pr + secondary_w * (float)sr + white_scale * white_w * 255.0f,
            primary_w * (float)pg + secondary_w * (float)sg + white_scale * white_w * 255.0f,
            primary_w * (float)pb + secondary_w * (float)sb + white_scale * white_w * 255.0f);
}

static uint8_t lerp_u8(uint8_t a, uint8_t b, float t)
{
    float v = (1.0f - t) * (float)a + t * (float)b;
    if (v < 0.0f) v = 0.0f;
    if (v > 255.0f) v = 255.0f;
    return (uint8_t)(v + 0.5f);
}

static float ease_cosine01(float t)
{
    const float kPi = 3.14159265f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return 0.5f - 0.5f * cosf(kPi * t);
}

static void sample_palette3(uint8_t c0r, uint8_t c0g, uint8_t c0b,
                            uint8_t c1r, uint8_t c1g, uint8_t c1b,
                            uint8_t c2r, uint8_t c2g, uint8_t c2b,
                            float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    while (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;
    float seg = phase * 3.0f;
    int idx = (int)seg;
    float t = seg - (float)idx;
    if (idx <= 0) {
        if (r) *r = lerp_u8(c0r, c1r, t);
        if (g) *g = lerp_u8(c0g, c1g, t);
        if (b) *b = lerp_u8(c0b, c1b, t);
    } else if (idx == 1) {
        if (r) *r = lerp_u8(c1r, c2r, t);
        if (g) *g = lerp_u8(c1g, c2g, t);
        if (b) *b = lerp_u8(c1b, c2b, t);
    } else {
        if (r) *r = lerp_u8(c2r, c0r, t);
        if (g) *g = lerp_u8(c2g, c0g, t);
        if (b) *b = lerp_u8(c2b, c0b, t);
    }
}

static void sample_rainbow_phase(float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    static const uint8_t kAnchors[6][3] = {
        {255,   0,   0},
        {255, 220,   0},
        {  0, 255,   0},
        {  0, 255, 255},
        {  0,   0, 255},
        {255,   0, 255},
    };
    const float hold = 0.28f;
    const float move = 1.0f - hold;

    while (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;

    float seg = phase * 6.0f;
    int idx = (int)seg;
    float local = seg - (float)idx;
    idx %= 6;
    if (idx < 0) idx += 6;
    int next = (idx + 1) % 6;

    if (local <= hold || move <= 0.0001f) {
        if (r) *r = kAnchors[idx][0];
        if (g) *g = kAnchors[idx][1];
        if (b) *b = kAnchors[idx][2];
        return;
    }

    float t = ease_cosine01((local - hold) / move);
    if (r) *r = lerp_u8(kAnchors[idx][0], kAnchors[next][0], t);
    if (g) *g = lerp_u8(kAnchors[idx][1], kAnchors[next][1], t);
    if (b) *b = lerp_u8(kAnchors[idx][2], kAnchors[next][2], t);
}

static void sample_siren_phase(float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;

    if (phase < 0.08f || (phase >= 0.50f && phase < 0.58f)) {
        if (r) *r = 255;
        if (g) *g = 255;
        if (b) *b = 255;
    } else if (phase < 0.50f) {
        if (r) *r = 255;
        if (g) *g = 0;
        if (b) *b = 0;
    } else {
        if (r) *r = 0;
        if (g) *g = 48;
        if (b) *b = 255;
    }
}

static void led_modes_sample_palette_color(float t_sec, float phase_offset,
                                           uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (s_color_cycle) {
        case LED_COLOR_CYCLE_RAINBOW:
            sample_rainbow_phase(0.10f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_SIREN:
            sample_siren_phase(1.35f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_ARCADE:
            sample_palette3(255, 0, 255,
                            0, 250, 255,
                            255, 220, 0,
                            0.11f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_INFERNO:
            sample_palette3(255, 20, 0,
                            255, 110, 0,
                            255, 210, 24,
                            0.085f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_BIOHAZARD:
            sample_palette3(0, 255, 64,
                            190, 255, 0,
                            18, 110, 0,
                            0.09f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_STATIC:
        default:
            if (r) *r = s_primary_r;
            if (g) *g = s_primary_g;
            if (b) *b = s_primary_b;
            break;
    }
}

static void led_modes_sample_color(float t_sec, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_color_style == LED_COLOR_STYLE_PALETTE &&
        s_color_cycle != LED_COLOR_CYCLE_STATIC) {
        led_modes_sample_palette_color(t_sec, 0.0f, r, g, b);
        return;
    }
    if (r) *r = s_primary_r;
    if (g) *g = s_primary_g;
    if (b) *b = s_primary_b;
}

static void led_modes_sample_secondary_color(float t_sec, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_color_style == LED_COLOR_STYLE_PALETTE &&
        s_color_cycle != LED_COLOR_CYCLE_STATIC) {
        led_modes_sample_palette_color(t_sec, 0.33f, r, g, b);
        return;
    }
    if (s_color_style == LED_COLOR_STYLE_DUO) {
        if (r) *r = s_secondary_r;
        if (g) *g = s_secondary_g;
        if (b) *b = s_secondary_b;
        return;
    }
    if (r) *r = s_primary_r;
    if (g) *g = s_primary_g;
    if (b) *b = s_primary_b;
}

static void layout_stats_collect(const led_layout_config_t *cfg, size_t count, layout_stats_t *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    stats->min_x = stats->min_y = stats->min_z = 1e9f;
    stats->max_x = stats->max_y = stats->max_z = -1e9f;

    if (cfg) {
        for (size_t i = 0; i < cfg->section_count; ++i) {
            const led_layout_section_t *sec = &cfg->sections[i];
            if (sec->geom_kind == LED_LAYOUT_GEOM_ARC && strstr(sec->name, "ring") != NULL) {
                stats->ring_center = sec->center;
                stats->ring_radius = fabsf(sec->radius);
                break;
            }
        }
    }

    bool have_point = false;
    size_t point_count = 0;
    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        have_point = true;
        point_count++;
        stats->avg_x += p.x;
        stats->avg_y += p.y;
        stats->avg_z += p.z;
        if (p.x < stats->min_x) stats->min_x = p.x;
        if (p.x > stats->max_x) stats->max_x = p.x;
        if (p.y < stats->min_y) stats->min_y = p.y;
        if (p.y > stats->max_y) stats->max_y = p.y;
        if (p.z < stats->min_z) stats->min_z = p.z;
        if (p.z > stats->max_z) stats->max_z = p.z;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float dz = p.z - stats->ring_center.z;
        float radial = sqrtf(dx * dx + dy * dy);
        if (radial > stats->radial_max) stats->radial_max = radial;
        float ring_delta = radial - stats->ring_radius;
        float ring_wave = sqrtf(ring_delta * ring_delta + dz * dz);
        if (ring_wave > stats->ring_wave_max) stats->ring_wave_max = ring_wave;
    }

    if (!have_point) {
        stats->min_x = stats->min_y = stats->min_z = -1.0f;
        stats->max_x = stats->max_y = stats->max_z = 1.0f;
        stats->avg_x = 0.0f;
        stats->avg_y = 0.0f;
        stats->avg_z = 0.0f;
    } else if (point_count > 0u) {
        float inv_count = 1.0f / (float)point_count;
        stats->avg_x *= inv_count;
        stats->avg_y *= inv_count;
        stats->avg_z *= inv_count;
    }
    if (stats->radial_max < 1.0f) {
        float hx = 0.5f * (stats->max_x - stats->min_x);
        float hy = 0.5f * (stats->max_y - stats->min_y);
        stats->radial_max = sqrtf(hx * hx + hy * hy);
    }
    if (stats->ring_wave_max < 1.0f) {
        stats->ring_wave_max = stats->radial_max;
    }
}

static float layout_projection_extent(const layout_stats_t *stats,
                                      float nx, float ny, float nz)
{
    if (!stats) return 1.0f;
    float hx = 0.5f * (stats->max_x - stats->min_x);
    float hy = 0.5f * (stats->max_y - stats->min_y);
    float hz = 0.5f * (stats->max_z - stats->min_z);
    float extent = absf_local(nx) * hx + absf_local(ny) * hy + absf_local(nz) * hz;
    return (extent < 1.0f) ? 1.0f : extent;
}

static void render_plane(uint8_t *buf, size_t count,
                         const led_layout_config_t *cfg,
                         const layout_stats_t *stats,
                         float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    float angle = t_sec * 0.52f;
    float nx = cosf(angle);
    float ny = sinf(angle);
    float nz = 0.20f * sinf(t_sec * 0.23f);
    float inv_len = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    nx *= inv_len;
    ny *= inv_len;
    nz *= inv_len;

    float extent = layout_projection_extent(stats, nx, ny, nz);
    float width = 1.0f + 0.10f * stats->radial_max;
    float phase = fmodf(t_sec * 0.18f, 2.0f);
    if (phase < 0.0f) phase += 2.0f;
    float sweep = (phase <= 1.0f) ? phase : (2.0f - phase);
    float offset = (2.0f * sweep - 1.0f) * (extent + 0.35f * width);

    float cx = stats->avg_x;
    float cy = stats->avg_y;
    float cz = stats->avg_z;

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float proj = (p.x - cx) * nx + (p.y - cy) * ny + (p.z - cz) * nz;
        float d = proj - offset;
        float core = expf(-fabsf(d) / width);
        float ripple = 0.5f + 0.5f * sinf(1.4f * d - t_sec * 4.0f);
        ripple *= ripple;
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        core * (0.55f + 0.45f * ripple),
                        core * core * (0.06f + 0.28f * (1.0f - ripple)),
                        core * core * 0.08f);
    }
}

static void render_prism(uint8_t *buf, size_t count,
                         const led_layout_config_t *cfg,
                         const layout_stats_t *stats,
                         float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    (void)stats;
    float a0 = t_sec * 0.21f;
    float a1 = -0.7f + t_sec * 0.13f;
    float n0x = cosf(a0);
    float n0y = sinf(a0);
    float n1x = cosf(a1);
    float n1y = sinf(a1);

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float proj0 = p.x * n0x + p.y * n0y;
        float proj1 = p.x * n1x + p.y * n1y;
        float band = 0.5f + 0.5f * sinf(0.42f * proj0 - t_sec * 4.3f +
                                        1.5f * sinf(0.24f * proj1 + t_sec * 1.6f));
        float cut = 0.5f + 0.5f * sinf(0.75f * proj1 + t_sec * 2.0f);
        float sparkle = 0.5f + 0.5f * sinf(1.3f * proj1 - t_sec * 9.0f + 0.18f * proj0);
        band *= band;
        band *= band;
        sparkle *= sparkle;
        sparkle *= sparkle;
        sparkle *= sparkle;
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        band * (0.25f + 0.75f * cut),
                        (1.0f - band) * 0.28f + 0.45f * band * (1.0f - cut),
                        sparkle * 0.40f);
    }
}

static void render_ring(uint8_t *buf, size_t count,
                        const led_layout_config_t *cfg,
                        const layout_stats_t *stats,
                        float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    float cycle = stats->ring_wave_max + 4.0f;
    if (cycle < 1.0f) cycle = 1.0f;
    float front0 = fmodf(t_sec * 15.0f, cycle);
    float front1 = fmodf(front0 + 0.55f * cycle, cycle);
    float width0 = 0.55f + 0.035f * stats->ring_wave_max;
    float width1 = 1.05f + 0.055f * stats->ring_wave_max;
    float origin_width = 0.30f + 0.06f * stats->ring_radius;
    if (origin_width < 0.30f) origin_width = 0.30f;

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float ring_d = ring_distance(stats, &p);
        float origin = expf(-ring_d / origin_width);
        float wave0 = expf(-fabsf(ring_d - front0) / width0);
        float wave1 = expf(-fabsf(ring_d - front1) / width1);
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        0.18f * origin + 0.95f * wave0,
                        0.32f * wave1,
                        0.10f * wave0 * wave0);
    }
}

static void render_contour(uint8_t *buf, size_t count,
                           const led_layout_config_t *cfg,
                           const layout_stats_t *stats,
                           float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float radial = sqrtf(dx * dx + dy * dy);
        float iso = 0.22f * p.y + 0.14f * fabsf(p.x) + 0.18f * radial;
        float bands = 0.5f + 0.5f * sinf(iso - t_sec * 3.1f);
        float valley = 0.5f + 0.5f * sinf(0.13f * p.x + t_sec * 1.2f);
        float shimmer = 0.5f + 0.5f * sinf(0.31f * p.y - t_sec * 5.0f + 0.18f * p.x);
        bands *= bands;
        bands *= bands;
        shimmer *= shimmer;
        shimmer *= shimmer;
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        bands * (0.35f + 0.65f * valley),
                        (1.0f - bands) * 0.10f + shimmer * 0.35f,
                        shimmer * 0.10f);
    }
}

static void render_orbit(uint8_t *buf, size_t count,
                         const led_layout_config_t *cfg,
                         const layout_stats_t *stats,
                         float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    float head = t_sec * 1.2f;
    float orbit_radius = 0.62f * stats->radial_max;
    if (orbit_radius < stats->ring_radius + 1.0f) {
        orbit_radius = stats->ring_radius + 1.0f;
    }

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float radial = sqrtf(dx * dx + dy * dy);
        float ang = atan2f(dy, dx);
        float spoke = expf(-angular_distance(ang, head) / 0.30f);
        float counter = expf(-angular_distance(ang, head + 2.2f) / 0.55f);
        float radial_band = expf(-fabsf(radial - orbit_radius) / 3.8f);
        float core = expf(-fabsf(radial - stats->ring_radius) / 1.1f);
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        spoke * (0.25f + 0.75f * radial_band),
                        counter * (0.12f + 0.88f * core),
                        0.10f * spoke);
    }
}

static void render_aurora(uint8_t *buf, size_t count,
                          const led_layout_config_t *cfg,
                          const layout_stats_t *stats,
                          float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    (void)stats;
    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float curtain_a = 0.5f + 0.5f * sinf(0.18f * p.x + 0.42f * p.y - t_sec * 2.4f +
                                             1.4f * sinf(0.09f * p.y + t_sec * 0.7f));
        float curtain_b = 0.5f + 0.5f * sinf(-0.14f * p.x + 0.31f * p.y + t_sec * 1.8f);
        float sparkle = 0.5f + 0.5f * sinf(0.45f * p.y - t_sec * 9.0f + 0.20f * p.x);
        curtain_a *= curtain_a;
        curtain_b *= curtain_b;
        sparkle *= sparkle;
        sparkle *= sparkle;
        sparkle *= sparkle;
        sparkle *= sparkle;
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        curtain_a * (0.22f + 0.78f * curtain_b),
                        curtain_b * (0.12f + 0.48f * (1.0f - curtain_a)),
                        sparkle * 0.55f);
    }
}

static void render_vortex(uint8_t *buf, size_t count,
                          const led_layout_config_t *cfg,
                          const layout_stats_t *stats,
                          float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    float throat = 0.55f + 0.10f * stats->ring_radius;
    if (throat < 0.55f) throat = 0.55f;

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float radial = sqrtf(dx * dx + dy * dy);
        float ang = atan2f(dy, dx);
        float ring_d = ring_distance(stats, &p);

        float arm0 = 0.5f + 0.5f * sinf(3.4f * ang + 0.28f * radial - t_sec * 5.4f + 0.07f * p.y);
        float arm1 = 0.5f + 0.5f * sinf(-2.1f * ang + 0.19f * radial + t_sec * 4.0f - 0.05f * p.y);
        float core = expf(-ring_d / throat);
        float rim = expf(-fabsf(radial - (stats->ring_radius + 0.10f * fabsf(p.y))) / 5.0f);

        arm0 *= arm0;
        arm0 *= arm0;
        arm1 *= arm1;
        arm1 *= arm1;

        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        0.15f * core + 0.95f * arm0 * (0.25f + 0.75f * rim),
                        0.10f * core + 0.55f * arm1,
                        0.12f * core * core);
    }
}

static void render_helix(uint8_t *buf, size_t count,
                         const led_layout_config_t *cfg,
                         const layout_stats_t *stats,
                         float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float radial = sqrtf(dx * dx + dy * dy);
        float ang = atan2f(dy, dx);
        float y = p.y - stats->ring_center.y;

        float phase = 2.5f * ang + 0.18f * y - t_sec * 4.8f;
        float band0 = 0.5f + 0.5f * cosf(phase);
        float band1 = 0.5f + 0.5f * cosf(phase + (float)M_PI);
        float ring_band = expf(-fabsf(radial - (stats->ring_radius + 0.12f * fabsf(y))) / 4.2f);
        float shimmer = 0.5f + 0.5f * sinf(0.33f * y - t_sec * 6.0f + 0.12f * dx);

        band0 *= band0;
        band0 *= band0;
        band1 *= band1;
        band1 *= band1;
        shimmer *= shimmer;
        shimmer *= shimmer;

        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        band0 * (0.18f + 0.82f * ring_band),
                        band1 * (0.12f + 0.68f * ring_band) + 0.18f * shimmer,
                        0.08f * (band0 + band1) * shimmer);
    }
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

static void render_mirror(uint8_t *buf, size_t count,
                          const led_layout_config_t *cfg,
                          const layout_stats_t *stats,
                          float t_sec, uint8_t r, uint8_t g, uint8_t b)
{
    if (count == 0) return;
    if (!cfg || !stats || cfg->section_count == 0) {
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
        return;
    }

    uint8_t sr = 0, sg = 0, sb = 0;
    led_modes_sample_secondary_color(t_sec, &sr, &sg, &sb);

    float plane_x = stats->ring_center.x;
    float x_extent = fmaxf(fabsf(stats->min_x - plane_x), fabsf(stats->max_x - plane_x));
    if (x_extent < 1.0f) x_extent = 1.0f;

    float phase = fmodf(t_sec * 0.28f, 2.0f);
    if (phase < 0.0f) phase += 2.0f;
    float sweep = (phase <= 1.0f) ? phase : (2.0f - phase);
    float pos = sweep * x_extent;
    float width = 0.75f + 0.05f * stats->radial_max;

    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        float dx = fabsf(p.x - plane_x);
        float crest = expf(-fabsf(dx - pos) / width);
        if (crest < 0.01f) continue;
        float shimmer = 0.5f + 0.5f * sinf(0.20f * (p.y - stats->avg_y) - t_sec * 3.4f);
        shimmer *= shimmer;
        add_mixed_color(buf, i, r, g, b, sr, sg, sb,
                        crest * (0.70f + 0.30f * shimmer),
                        crest * crest * (0.12f + 0.28f * (1.0f - shimmer)),
                        crest * crest * 0.08f);
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
        float white = (s_highlight_mode == LED_HIGHLIGHT_PEAKS && n > 0.992f) ? 1.0f : 0.0f;
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
    int shuffle_mode = -1;
    int64_t shuffle_next_switch_us = 0;
    float shuffle_t_sec = 0.0f;

    while (1) {
        if (!s_enabled) {
            if (was_enabled) {
                memset(frame, 0, sizeof(frame));
                (void)led_show_pixels(frame, LED_STRIP_LENGTH);
                was_enabled = false;
                shuffle_mode = -1;
                shuffle_next_switch_us = 0;
                shuffle_t_sec = 0.0f;
            }
            vTaskDelay(idle_period);
            last_wake = xTaskGetTickCount();
            continue;
        }

        if (!was_enabled) {
            was_enabled = true;
            t_sec = 0.0f;
            shuffle_mode = -1;
            shuffle_next_switch_us = 0;
            shuffle_t_sec = 0.0f;
            last_wake = xTaskGetTickCount();
        }
        float speed_scale = (float)s_speed_percent / 100.0f;
        if (speed_scale < 0.10f) speed_scale = 0.10f;
        if (speed_scale > 2.50f) speed_scale = 2.50f;
        t_sec += dt_sec * speed_scale;

        size_t count = custom_render_pixels();
        memset(frame, 0, sizeof(frame));
        uint8_t pr = 0, pg = 0, pb = 0;
        led_modes_sample_color(t_sec, &pr, &pg, &pb);

        int mode = clamp_mode(s_mode);
        int effective_mode = mode;
        float render_t_sec = t_sec;
        if (mode == kShuffleModeIndex) {
            int64_t now_us = esp_timer_get_time();
            if (shuffle_mode < 0 || now_us >= shuffle_next_switch_us) {
                shuffle_mode = pick_shuffle_mode(shuffle_mode);
                shuffle_next_switch_us = now_us + LED_MODES_SHUFFLE_INTERVAL_US;
                shuffle_t_sec = 0.0f;
            } else {
                shuffle_t_sec += dt_sec * speed_scale;
            }
            effective_mode = shuffle_mode;
            render_t_sec = shuffle_t_sec;
        } else {
            shuffle_mode = -1;
            shuffle_next_switch_us = 0;
            shuffle_t_sec = 0.0f;
        }
        layout_stats_t stats = {0};
        bool use_layout_mode = mode_uses_layout(effective_mode);
        if (use_layout_mode) {
            led_layout_snapshot(&s_render_layout);
            if (s_render_layout.total_leds > 0 && count > s_render_layout.total_leds) {
                count = s_render_layout.total_leds;
            }
            layout_stats_collect(&s_render_layout, count, &stats);
        }
        switch (effective_mode) {
            case 0: render_fill(frame, count, 0, 0, 0); break;
            case 1: render_fill(frame, count, pr, pg, pb); break;
            case 2: render_breathe(frame, count, render_t_sec, pr, pg, pb); break;
            case 3: render_scanner(frame, count, render_t_sec, pr, pg, pb); break;
            case 4: render_mirror(frame, count,
                                  use_layout_mode ? &s_render_layout : NULL,
                                  use_layout_mode ? &stats : NULL,
                                  render_t_sec, pr, pg, pb); break;
            case 5: render_energy(frame, count, render_t_sec, pr, pg, pb); break;
            case 6: render_blink(frame, count, render_t_sec, 0.14f, 0.46f, pr, pg, pb); break;
            case 7: render_chase(frame, count, render_t_sec, pr, pg, pb); break;
            case 8: render_twinkle(frame, count, render_t_sec, pr, pg, pb); break;
            case 9: render_glitch(frame, count, render_t_sec, pr, pg, pb); break;
            case 10: render_plane(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 11: render_prism(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 12: render_ring(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 13: render_contour(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 14: render_orbit(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 15: render_aurora(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 16: render_vortex(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            case 17: render_helix(frame, count, &s_render_layout, &stats, render_t_sec, pr, pg, pb); break;
            default: render_fill(frame, count, 0, 0, 0); break;
        }

        float output_scale = (float)s_brightness / 255.0f;
        if (output_scale < 0.0f) output_scale = 0.0f;
        if (output_scale > 1.0f) output_scale = 1.0f;
        if (output_scale < 0.999f) {
            for (size_t i = 0; i < count * 3; ++i) {
                frame[i] = (uint8_t)((float)frame[i] * output_scale);
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
