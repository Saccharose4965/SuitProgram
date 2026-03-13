#include "led_layout.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "storage_sd.h"

#define LED_LAYOUT_FILE_NAME "led_layout.txt"
#define LAYOUT_POINT_EPSILON 0.0001f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "led_layout";

static EXT_RAM_BSS_ATTR led_layout_config_t s_layout = {0};
static bool s_ready = false;
static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock = NULL;

static void layout_fill_default(led_layout_config_t *cfg);
static void layout_finalize(led_layout_config_t *cfg);

typedef struct {
    const char *name;
    uint8_t strip;
    uint16_t length;
    bool reversed;
} led_layout_default_section_t;

static const led_layout_default_section_t k_default_sections[] = {
    { "front_left_top",        0, 27, false },
    { "front_left_ring",       0, 35, false },
    { "front_left_rib",        0, 47, false },
    { "front_left_abs",        0, 31, false },
    { "front_left_belt",       0, 39, false },
    { "back_left_belt",        0, 31, false },
    { "back_left_vertebra",    0, 28, false },
    { "back_left_rib",         0, 37, false },
    { "back_left_top",         0, 35, false },
    { "left_upper_arm",        0, 45, false },
    { "left_forearm",          0, 23, false },
    { "front_right_top",       1, 27, true  },
    { "front_right_rib",       1, 47, true  },
    { "front_right_abs",       1, 31, true  },
    { "front_right_belt",      1, 39, true  },
    { "back_right_belt",       1, 31, true  },
    { "back_right_vertebra",   1, 28, true  },
    { "back_right_rib",        1, 37, true  },
    { "back_right_top",        1, 35, true  },
    { "right_upper_arm",       1, 45, true  },
    { "right_forearm",         1, 23, true  },
};

typedef struct {
    const char *old_name;
    const char *new_name;
} led_layout_legacy_name_t;

static const led_layout_legacy_name_t k_legacy_section_names[] = {
    { "left_front_low",   "front_left_top" },
    { "left_ring",        "front_left_ring" },
    { "left_front_high",  "front_left_rib" },
    { "left_side_a",      "front_left_abs" },
    { "left_side_b",      "front_left_belt" },
    { "back_left_1",      "back_left_belt" },
    { "back_left_2",      "back_left_vertebra" },
    { "back_left_3",      "back_left_rib" },
    { "back_left_4",      "back_left_top" },
    { "right_front_low",  "front_right_top" },
    { "right_front_high", "front_right_rib" },
    { "right_side_a",     "front_right_abs" },
    { "right_side_b",     "front_right_belt" },
    { "back_right_1",     "back_right_belt" },
    { "back_right_2",     "back_right_vertebra" },
    { "back_right_3",     "back_right_rib" },
    { "back_right_4",     "back_right_top" },
    { "left_upper",       "front_left_top" },
    { "center_ring",      "front_left_ring" },
    { "left_outer",       "front_left_rib" },
    { "left_inner",       "front_left_abs" },
    { "left_belt",        "front_left_belt" },
    { "right_upper",      "front_right_top" },
    { "right_outer",      "front_right_rib" },
    { "right_inner",      "front_right_abs" },
    { "right_belt",       "front_right_belt" },
};

static const led_point_t k_left_upper_arm_points_legacy[] = {
    { 16.0f, -6.0f, -2.0f },
    { 17.5f, -1.5f, -2.0f },
    { 20.0f,  6.0f,  0.0f },
    { 18.5f,  4.0f,  1.5f },
    { 17.0f,  0.5f,  2.0f },
};

static const led_point_t k_left_forearm_points_legacy[] = {
    { 20.0f,  6.0f, 0.0f },
    { 23.0f, 15.5f, 0.0f },
};

static const led_point_t k_right_upper_arm_points_legacy[] = {
    { -16.0f, -6.0f, -2.0f },
    { -17.5f, -1.5f, -2.0f },
    { -20.0f,  6.0f,  0.0f },
    { -18.5f,  4.0f,  1.5f },
    { -17.0f,  0.5f,  2.0f },
};

static const led_point_t k_right_forearm_points_legacy[] = {
    { -20.0f,  6.0f, 0.0f },
    { -23.0f, 15.5f, 0.0f },
};

static const led_point_t k_left_upper_arm_points_measured[] = {
    { 10.5f, 48.5f, -3.0f },
    { 15.5f, 39.5f, -2.0f },
    { 21.5f, 16.0f,  0.0f },
    { 19.5f, 19.5f,  2.0f },
    { 17.0f, 23.0f,  3.0f },
};

static const led_point_t k_left_forearm_points_measured[] = {
    { 17.0f, 23.0f, 3.0f },
    { 20.0f,  4.0f, 2.0f },
    { 22.0f, -15.0f, 1.0f },
};

static const led_point_t k_right_upper_arm_points_measured[] = {
    { -10.5f, 48.5f, -3.0f },
    { -15.5f, 39.5f, -2.0f },
    { -21.5f, 16.0f,  0.0f },
    { -19.5f, 19.5f,  2.0f },
    { -17.0f, 23.0f,  3.0f },
};

static const led_point_t k_right_forearm_points_measured[] = {
    { -17.0f, 23.0f, 3.0f },
    { -20.0f,  4.0f, 2.0f },
    { -22.0f, -15.0f, 1.0f },
};

static const char *k_canonical_section_order[] = {
    "front_left_top",
    "front_left_ring",
    "front_left_rib",
    "front_left_abs",
    "front_left_belt",
    "back_left_belt",
    "back_left_vertebra",
    "back_left_rib",
    "back_left_top",
    "left_upper_arm",
    "left_forearm",
    "front_right_top",
    "front_right_rib",
    "front_right_abs",
    "front_right_belt",
    "back_right_belt",
    "back_right_vertebra",
    "back_right_rib",
    "back_right_top",
    "right_upper_arm",
    "right_forearm",
};

static SemaphoreHandle_t layout_lock_handle(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }
    return s_lock;
}

static void layout_ensure_ready_locked(void)
{
    if (s_ready) return;
    layout_fill_default(&s_layout);
    layout_finalize(&s_layout);
    s_ready = true;
}

static bool layout_lock_take(void)
{
    SemaphoreHandle_t lock = layout_lock_handle();
    return lock && xSemaphoreTake(lock, portMAX_DELAY) == pdTRUE;
}

static void layout_lock_give(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

static void copy_name(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) return;
    if (!src) src = "";
    snprintf(dst, dst_sz, "%s", src);
}

static void layout_upgrade_legacy_names(led_layout_config_t *cfg)
{
    if (!cfg) return;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        for (size_t m = 0; m < sizeof(k_legacy_section_names) / sizeof(k_legacy_section_names[0]); ++m) {
            if (strcmp(cfg->sections[i].name, k_legacy_section_names[m].old_name) == 0) {
                copy_name(cfg->sections[i].name,
                          sizeof(cfg->sections[i].name),
                          k_legacy_section_names[m].new_name);
                break;
            }
        }
    }
}

static void layout_normalize_section_order(led_layout_config_t *cfg)
{
    if (!cfg || cfg->section_count < 2) return;

    led_layout_section_t ordered[LED_LAYOUT_MAX_SECTIONS] = {0};
    bool used[LED_LAYOUT_MAX_SECTIONS] = {0};
    size_t out = 0;

    for (size_t name_idx = 0;
         name_idx < sizeof(k_canonical_section_order) / sizeof(k_canonical_section_order[0]) &&
         out < cfg->section_count;
         ++name_idx) {
        const char *name = k_canonical_section_order[name_idx];
        for (size_t sec_idx = 0; sec_idx < cfg->section_count; ++sec_idx) {
            if (used[sec_idx]) continue;
            if (strcmp(cfg->sections[sec_idx].name, name) != 0) continue;
            ordered[out++] = cfg->sections[sec_idx];
            used[sec_idx] = true;
            break;
        }
    }

    for (size_t sec_idx = 0; sec_idx < cfg->section_count && out < cfg->section_count; ++sec_idx) {
        if (used[sec_idx]) continue;
        ordered[out++] = cfg->sections[sec_idx];
    }

    for (size_t i = 0; i < cfg->section_count; ++i) {
        cfg->sections[i] = ordered[i];
    }
}

static char *trim(char *s)
{
    if (!s) return s;
    while (*s && isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) {
        end--;
    }
    *end = '\0';
    return s;
}

static bool parse_u32(const char *s, uint32_t *out)
{
    if (!s || !*s || !out) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s) return false;
    while (end && *end) {
        if (!isspace((unsigned char)*end)) return false;
        ++end;
    }
    *out = (uint32_t)v;
    return true;
}

static bool parse_f32(const char *s, float *out)
{
    if (!s || !*s || !out) return false;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s) return false;
    while (end && *end) {
        if (!isspace((unsigned char)*end)) return false;
        ++end;
    }
    *out = v;
    return true;
}

static bool parse_bool(const char *s, bool *out)
{
    if (!s || !out) return false;
    if (strcmp(s, "1") == 0 ||
        strcasecmp(s, "true") == 0 ||
        strcasecmp(s, "yes") == 0 ||
        strcasecmp(s, "on") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(s, "0") == 0 ||
        strcasecmp(s, "false") == 0 ||
        strcasecmp(s, "no") == 0 ||
        strcasecmp(s, "off") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool parse_point3(const char *s, led_point_t *out)
{
    if (!s || !out) return false;
    char buf[96];
    copy_name(buf, sizeof(buf), s);

    char *p0 = trim(buf);
    char *c1 = strchr(p0, ',');
    if (!c1) return false;
    *c1 = '\0';
    char *p1 = trim(c1 + 1);
    char *c2 = strchr(p1, ',');

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (!c2) {
        if (!parse_f32(p0, &x) || !parse_f32(p1, &y)) return false;
    } else {
        *c2 = '\0';
        char *p2 = trim(c2 + 1);
        if (!parse_f32(p0, &x) || !parse_f32(p1, &y) || !parse_f32(p2, &z)) return false;
    }

    out->x = x;
    out->y = y;
    out->z = z;
    return true;
}

static const char *geom_kind_name(uint8_t geom_kind)
{
    switch ((led_layout_geom_t)geom_kind) {
        case LED_LAYOUT_GEOM_POLYLINE: return "polyline";
        case LED_LAYOUT_GEOM_ARC: return "arc";
        case LED_LAYOUT_GEOM_NONE:
        default:
            return "none";
    }
}

static uint8_t geom_kind_from_str(const char *s)
{
    if (!s) return LED_LAYOUT_GEOM_NONE;
    if (strcasecmp(s, "polyline") == 0 || strcasecmp(s, "line") == 0) {
        return LED_LAYOUT_GEOM_POLYLINE;
    }
    if (strcasecmp(s, "arc") == 0 || strcasecmp(s, "ring") == 0 || strcasecmp(s, "circle") == 0) {
        return LED_LAYOUT_GEOM_ARC;
    }
    return LED_LAYOUT_GEOM_NONE;
}

static float point_distance(const led_point_t *a, const led_point_t *b)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    float dz = b->z - a->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static led_point_t point_lerp(const led_point_t *a, const led_point_t *b, float t)
{
    led_point_t out = {
        .x = a->x + (b->x - a->x) * t,
        .y = a->y + (b->y - a->y) * t,
        .z = a->z + (b->z - a->z) * t,
    };
    return out;
}

static void section_clear_geometry(led_layout_section_t *sec)
{
    if (!sec) return;
    sec->geom_kind = LED_LAYOUT_GEOM_NONE;
    sec->point_count = 0;
    memset(sec->points, 0, sizeof(sec->points));
    sec->center = (led_point_t){0};
    sec->radius = 0.0f;
    sec->start_deg = 0.0f;
    sec->sweep_deg = 0.0f;
    sec->connected_to_prev = false;
}

static void section_set_polyline2(led_layout_section_t *sec,
                                  bool connected_to_prev,
                                  led_point_t p0,
                                  led_point_t p1)
{
    if (!sec) return;
    section_clear_geometry(sec);
    sec->geom_kind = LED_LAYOUT_GEOM_POLYLINE;
    sec->connected_to_prev = connected_to_prev;
    sec->point_count = 2;
    sec->points[0] = p0;
    sec->points[1] = p1;
}

static void section_set_polyline_points(led_layout_section_t *sec,
                                        bool connected_to_prev,
                                        const led_point_t *points,
                                        size_t point_count)
{
    if (!sec || !points || point_count < 2 || point_count > LED_LAYOUT_MAX_SECTION_POINTS) return;
    section_clear_geometry(sec);
    sec->geom_kind = LED_LAYOUT_GEOM_POLYLINE;
    sec->connected_to_prev = connected_to_prev;
    sec->point_count = (uint8_t)point_count;
    for (size_t i = 0; i < point_count; ++i) {
        sec->points[i] = points[i];
    }
}

static void section_set_arc(led_layout_section_t *sec,
                            bool connected_to_prev,
                            led_point_t center,
                            float radius,
                            float start_deg,
                            float sweep_deg)
{
    if (!sec) return;
    section_clear_geometry(sec);
    sec->geom_kind = LED_LAYOUT_GEOM_ARC;
    sec->connected_to_prev = connected_to_prev;
    sec->center = center;
    sec->radius = radius;
    sec->start_deg = start_deg;
    sec->sweep_deg = sweep_deg;
}

static inline led_point_t point3(float x, float y, float z)
{
    led_point_t p = {
        .x = x,
        .y = y,
        .z = z,
    };
    return p;
}

static bool points_match_exactish(const led_point_t *a, const led_point_t *b, size_t count)
{
    if (!a || !b) return false;
    for (size_t i = 0; i < count; ++i) {
        if (fabsf(a[i].x - b[i].x) > 0.05f ||
            fabsf(a[i].y - b[i].y) > 0.05f ||
            fabsf(a[i].z - b[i].z) > 0.05f) {
            return false;
        }
    }
    return true;
}

static bool section_matches_polyline(const led_layout_section_t *sec,
                                     const led_point_t *points,
                                     size_t point_count)
{
    if (!sec || !points) return false;
    if (sec->geom_kind != LED_LAYOUT_GEOM_POLYLINE) return false;
    if (sec->point_count != point_count) return false;
    return points_match_exactish(sec->points, points, point_count);
}

static bool layout_assign_measured_geometry(led_layout_section_t *sec)
{
    if (!sec) return false;

    const float front_z = 0.0f;
    const float back_z = 0.0f;
    const led_point_t ring_center = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    const float ring_radius = 4.25f;

    if (strcmp(sec->name, "front_left_ring") == 0) {
        // 0 deg is the viewer-right point, which is left when worn. Use a
        // negative sweep because torso coordinates use +y downward.
        section_set_arc(sec, false, ring_center, ring_radius, 0.0f, -360.0f);
        return true;
    }

    if (strcmp(sec->name, "front_left_top") == 0) {
        const led_point_t pts[] = {
            { 11.5f, -13.0f, front_z },
            { 10.5f,  -6.0f, front_z },
            {  6.5f,  -3.5f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_left_rib") == 0) {
        const led_point_t pts[] = {
            {  6.0f,  9.0f, front_z },
            { 13.0f, 15.0f, front_z },
            { 24.0f, 17.5f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_left_abs") == 0) {
        const led_point_t pts[] = {
            { 8.5f, 16.5f, front_z },
            { 6.5f, 18.0f, front_z },
            { 5.0f, 29.0f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_left_belt") == 0) {
        const led_point_t pts[] = {
            {  8.5f, 33.5f, front_z },
            { 25.0f, 30.0f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 2);
        return true;
    }
    if (strcmp(sec->name, "back_left_belt") == 0) {
        const led_point_t pts[] = {
            { 21.0f, 0.0f, back_z },
            {  7.5f, 0.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 2);
        return true;
    }
    if (strcmp(sec->name, "back_left_vertebra") == 0) {
        const led_point_t pts[] = {
            { 12.0f,  5.5f, back_z },
            {  7.0f,  9.0f, back_z },
            {  7.0f, 15.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "back_left_rib") == 0) {
        const led_point_t pts[] = {
            { 27.0f, 18.0f, back_z },
            { 16.0f, 23.0f, back_z },
            { 12.0f, 23.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "back_left_top") == 0) {
        const led_point_t pts[] = {
            {  7.0f, 34.0f, back_z },
            {  7.0f, 43.0f, back_z },
            { 10.0f, 48.5f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "left_upper_arm") == 0) {
        section_set_polyline_points(sec,
                                    false,
                                    k_left_upper_arm_points_measured,
                                    sizeof(k_left_upper_arm_points_measured) /
                                        sizeof(k_left_upper_arm_points_measured[0]));
        return true;
    }
    if (strcmp(sec->name, "left_forearm") == 0) {
        section_set_polyline_points(sec,
                                    true,
                                    k_left_forearm_points_measured,
                                    sizeof(k_left_forearm_points_measured) /
                                        sizeof(k_left_forearm_points_measured[0]));
        return true;
    }

    if (strcmp(sec->name, "front_right_top") == 0) {
        const led_point_t pts[] = {
            { -11.5f, -13.0f, front_z },
            { -10.5f,  -6.0f, front_z },
            {  -6.5f,  -3.5f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_right_rib") == 0) {
        const led_point_t pts[] = {
            {  -6.0f,  9.0f, front_z },
            { -13.0f, 15.0f, front_z },
            { -24.0f, 17.5f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_right_abs") == 0) {
        const led_point_t pts[] = {
            { -8.5f, 16.5f, front_z },
            { -6.5f, 18.0f, front_z },
            { -5.0f, 29.0f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "front_right_belt") == 0) {
        const led_point_t pts[] = {
            {  -8.5f, 33.5f, front_z },
            { -25.0f, 30.0f, front_z },
        };
        section_set_polyline_points(sec, false, pts, 2);
        return true;
    }
    if (strcmp(sec->name, "back_right_belt") == 0) {
        const led_point_t pts[] = {
            { -21.0f, 0.0f, back_z },
            {  -7.5f, 0.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 2);
        return true;
    }
    if (strcmp(sec->name, "back_right_vertebra") == 0) {
        const led_point_t pts[] = {
            { -12.0f,  5.5f, back_z },
            {  -7.0f,  9.0f, back_z },
            {  -7.0f, 15.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "back_right_rib") == 0) {
        const led_point_t pts[] = {
            { -27.0f, 18.0f, back_z },
            { -16.0f, 23.0f, back_z },
            { -12.0f, 23.0f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "back_right_top") == 0) {
        const led_point_t pts[] = {
            {  -7.0f, 34.0f, back_z },
            {  -7.0f, 43.0f, back_z },
            { -10.0f, 48.5f, back_z },
        };
        section_set_polyline_points(sec, false, pts, 3);
        return true;
    }
    if (strcmp(sec->name, "right_upper_arm") == 0) {
        section_set_polyline_points(sec,
                                    false,
                                    k_right_upper_arm_points_measured,
                                    sizeof(k_right_upper_arm_points_measured) /
                                        sizeof(k_right_upper_arm_points_measured[0]));
        return true;
    }
    if (strcmp(sec->name, "right_forearm") == 0) {
        section_set_polyline_points(sec,
                                    true,
                                    k_right_forearm_points_measured,
                                    sizeof(k_right_forearm_points_measured) /
                                        sizeof(k_right_forearm_points_measured[0]));
        return true;
    }

    return false;
}

static bool layout_geometry_looks_placeholder(const led_layout_config_t *cfg)
{
    if (!cfg || cfg->section_count == 0) return false;

    bool saw_known = false;
    float max_abs = 0.0f;
    float max_radius = 0.0f;

    for (size_t i = 0; i < cfg->section_count; ++i) {
        const led_layout_section_t *sec = &cfg->sections[i];
        if (strncmp(sec->name, "front_", 6) == 0 ||
            strncmp(sec->name, "back_", 5) == 0 ||
            strstr(sec->name, "_arm") != NULL ||
            strstr(sec->name, "forearm") != NULL) {
            saw_known = true;
        }
        for (size_t p = 0; p < sec->point_count && p < LED_LAYOUT_MAX_SECTION_POINTS; ++p) {
            float ax = fabsf(sec->points[p].x);
            float ay = fabsf(sec->points[p].y);
            float az = fabsf(sec->points[p].z);
            if (ax > max_abs) max_abs = ax;
            if (ay > max_abs) max_abs = ay;
            if (az > max_abs) max_abs = az;
        }
        if (fabsf(sec->center.x) > max_abs) max_abs = fabsf(sec->center.x);
        if (fabsf(sec->center.y) > max_abs) max_abs = fabsf(sec->center.y);
        if (fabsf(sec->center.z) > max_abs) max_abs = fabsf(sec->center.z);
        if (fabsf(sec->radius) > max_radius) max_radius = fabsf(sec->radius);
    }

    return saw_known && max_abs < 2.0f && max_radius < 1.0f;
}

static void layout_upgrade_placeholder_geometry(led_layout_config_t *cfg)
{
    if (!cfg || !layout_geometry_looks_placeholder(cfg)) return;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        layout_assign_measured_geometry(&cfg->sections[i]);
    }
}

static const led_layout_section_t *layout_find_section(const led_layout_config_t *cfg,
                                                       const char *name)
{
    if (!cfg || !name) return NULL;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        if (strcmp(cfg->sections[i].name, name) == 0) {
            return &cfg->sections[i];
        }
    }
    return NULL;
}

static bool layout_geometry_uses_screen_handedness(const led_layout_config_t *cfg)
{
    const led_layout_section_t *left = layout_find_section(cfg, "front_left_top");
    const led_layout_section_t *right = layout_find_section(cfg, "front_right_top");
    if (left && right && left->point_count > 0 && right->point_count > 0) {
        return left->points[0].x < -0.5f && right->points[0].x > 0.5f;
    }

    left = layout_find_section(cfg, "back_left_belt");
    right = layout_find_section(cfg, "back_right_belt");
    if (left && right && left->point_count > 0 && right->point_count > 0) {
        return left->points[0].x < -0.5f && right->points[0].x > 0.5f;
    }

    return false;
}

static void layout_upgrade_worn_handedness(led_layout_config_t *cfg)
{
    if (!cfg || !layout_geometry_uses_screen_handedness(cfg)) return;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        (void)layout_assign_measured_geometry(&cfg->sections[i]);
    }
}

static void layout_upgrade_ring_orientation(led_layout_config_t *cfg)
{
    if (!cfg) return;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        led_layout_section_t *sec = &cfg->sections[i];
        if (sec->geom_kind != LED_LAYOUT_GEOM_ARC) continue;
        if (strstr(sec->name, "ring") == NULL) continue;
        if (fabsf(fabsf(sec->sweep_deg) - 360.0f) > 0.01f) continue;
        if (sec->sweep_deg > 0.0f) {
            sec->sweep_deg = -fabsf(sec->sweep_deg);
        }
    }
}

static void layout_upgrade_arm_geometry(led_layout_config_t *cfg)
{
    if (!cfg) return;

    for (size_t i = 0; i < cfg->section_count; ++i) {
        led_layout_section_t *sec = &cfg->sections[i];
        if (strcmp(sec->name, "left_upper_arm") == 0 &&
            section_matches_polyline(sec,
                                     k_left_upper_arm_points_legacy,
                                     sizeof(k_left_upper_arm_points_legacy) /
                                         sizeof(k_left_upper_arm_points_legacy[0]))) {
            section_set_polyline_points(sec,
                                        false,
                                        k_left_upper_arm_points_measured,
                                        sizeof(k_left_upper_arm_points_measured) /
                                            sizeof(k_left_upper_arm_points_measured[0]));
            continue;
        }
        if (strcmp(sec->name, "left_forearm") == 0 &&
            section_matches_polyline(sec,
                                     k_left_forearm_points_legacy,
                                     sizeof(k_left_forearm_points_legacy) /
                                         sizeof(k_left_forearm_points_legacy[0]))) {
            section_set_polyline_points(sec,
                                        true,
                                        k_left_forearm_points_measured,
                                        sizeof(k_left_forearm_points_measured) /
                                            sizeof(k_left_forearm_points_measured[0]));
            continue;
        }
        if (strcmp(sec->name, "right_upper_arm") == 0 &&
            section_matches_polyline(sec,
                                     k_right_upper_arm_points_legacy,
                                     sizeof(k_right_upper_arm_points_legacy) /
                                         sizeof(k_right_upper_arm_points_legacy[0]))) {
            section_set_polyline_points(sec,
                                        false,
                                        k_right_upper_arm_points_measured,
                                        sizeof(k_right_upper_arm_points_measured) /
                                            sizeof(k_right_upper_arm_points_measured[0]));
            continue;
        }
        if (strcmp(sec->name, "right_forearm") == 0 &&
            section_matches_polyline(sec,
                                     k_right_forearm_points_legacy,
                                     sizeof(k_right_forearm_points_legacy) /
                                         sizeof(k_right_forearm_points_legacy[0]))) {
            section_set_polyline_points(sec,
                                        true,
                                        k_right_forearm_points_measured,
                                        sizeof(k_right_forearm_points_measured) /
                                            sizeof(k_right_forearm_points_measured[0]));
        }
    }
}

static void layout_assign_default_geometry(led_layout_section_t *sec, size_t idx)
{
    if (!sec) return;
    if (layout_assign_measured_geometry(sec)) return;

    float base_x = (sec->strip == 0) ? 0.26f : -0.26f;
    float start_y = -0.55f + 0.10f * (float)(idx % 4u);
    float end_y = start_y + 0.35f;
    float z = 0.20f;

    if (strstr(sec->name, "back") != NULL) {
        z = -0.18f;
        start_y = -0.10f + 0.12f * (float)((idx / 2u) % 4u);
        end_y = start_y + 0.28f;
    } else if (strstr(sec->name, "forearm") != NULL) {
        base_x = (sec->strip == 0) ? 0.58f : -0.58f;
        start_y = 0.00f + 0.08f * (float)(idx % 2u);
        end_y = start_y + 0.32f;
        z = 0.00f;
    } else if (strstr(sec->name, "_arm") != NULL) {
        base_x = (sec->strip == 0) ? 0.50f : -0.50f;
        start_y = -0.28f;
        end_y = 0.08f;
        z = 0.00f;
    } else if (strstr(sec->name, "side") != NULL) {
        z = 0.05f;
        start_y = -0.25f + 0.08f * (float)(idx % 4u);
        end_y = start_y + 0.24f;
    } else if (strstr(sec->name, "ring") != NULL) {
        float center_x = (sec->strip == 0) ? 0.10f : -0.10f;
        section_set_arc(sec, false, point3(center_x, 0.0f, 0.28f),
                        0.08f, 0.0f, -360.0f);
        return;
    }

    section_set_polyline2(sec,
                          (idx > 0),
                          point3(base_x, start_y, z),
                          point3(base_x, end_y, z));
}

static void layout_fill_default(led_layout_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    copy_name(cfg->profile, sizeof(cfg->profile), "chestplate");
    cfg->strip_count = 2;
    copy_name(cfg->strip_names[0], sizeof(cfg->strip_names[0]), "left");
    copy_name(cfg->strip_names[1], sizeof(cfg->strip_names[1]), "right");
    cfg->section_count = (uint8_t)(sizeof(k_default_sections) / sizeof(k_default_sections[0]));
    for (size_t i = 0; i < cfg->section_count; ++i) {
        copy_name(cfg->sections[i].name, sizeof(cfg->sections[i].name), k_default_sections[i].name);
        cfg->sections[i].strip = k_default_sections[i].strip;
        cfg->sections[i].length = k_default_sections[i].length;
        cfg->sections[i].reversed = k_default_sections[i].reversed;
        layout_assign_default_geometry(&cfg->sections[i], i);
    }
}

static void layout_enforce_pixel_limit(led_layout_config_t *cfg)
{
    if (!cfg) return;
    uint32_t total = 0;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        total += cfg->sections[i].length;
    }
    if (total <= LED_LAYOUT_MAX_PIXELS) return;

    uint32_t overflow = total - LED_LAYOUT_MAX_PIXELS;
    for (int i = (int)cfg->section_count - 1; i >= 0 && overflow > 0; --i) {
        led_layout_section_t *sec = &cfg->sections[i];
        uint16_t min_len = 1;
        if (sec->length <= min_len) continue;
        uint16_t shrink = (uint16_t)(sec->length - min_len);
        if (shrink > overflow) shrink = (uint16_t)overflow;
        sec->length = (uint16_t)(sec->length - shrink);
        overflow -= shrink;
    }
}

static void layout_finalize(led_layout_config_t *cfg)
{
    if (!cfg) return;

    if (cfg->strip_count == 0 || cfg->strip_count > LED_LAYOUT_MAX_STRIPS) {
        cfg->strip_count = LED_LAYOUT_MAX_STRIPS;
    }
    for (size_t strip = 0; strip < cfg->strip_count; ++strip) {
        if (!cfg->strip_names[strip][0]) {
            snprintf(cfg->strip_names[strip], sizeof(cfg->strip_names[strip]), "strip%u", (unsigned)strip);
        }
        cfg->strip_lengths[strip] = 0;
    }
    if (!cfg->profile[0]) {
        copy_name(cfg->profile, sizeof(cfg->profile), "chestplate");
    }

    if (cfg->section_count == 0) {
        layout_fill_default(cfg);
    }
    if (cfg->section_count > LED_LAYOUT_MAX_SECTIONS) {
        cfg->section_count = LED_LAYOUT_MAX_SECTIONS;
    }

    layout_normalize_section_order(cfg);
    layout_enforce_pixel_limit(cfg);

    uint16_t logical_start = 0;
    uint16_t strip_offsets[LED_LAYOUT_MAX_STRIPS] = {0};
    for (size_t i = 0; i < cfg->section_count; ++i) {
        led_layout_section_t *sec = &cfg->sections[i];
        if (!sec->name[0]) {
            snprintf(sec->name, sizeof(sec->name), "sec%02u", (unsigned)i);
        }
        if (sec->strip >= cfg->strip_count) {
            sec->strip = (uint8_t)(cfg->strip_count - 1);
        }
        if (sec->length == 0) {
            sec->length = 1;
        }
        if (sec->geom_kind == LED_LAYOUT_GEOM_POLYLINE) {
            if (sec->point_count < 2 || sec->point_count > LED_LAYOUT_MAX_SECTION_POINTS) {
                layout_assign_default_geometry(sec, i);
            }
        } else if (sec->geom_kind == LED_LAYOUT_GEOM_ARC) {
            if (fabsf(sec->radius) < LAYOUT_POINT_EPSILON || fabsf(sec->sweep_deg) < LAYOUT_POINT_EPSILON) {
                layout_assign_default_geometry(sec, i);
            }
        } else {
            layout_assign_default_geometry(sec, i);
        }
        sec->logical_start = logical_start;
        logical_start = (uint16_t)(logical_start + sec->length);
        sec->physical_start = strip_offsets[sec->strip];
        strip_offsets[sec->strip] = (uint16_t)(strip_offsets[sec->strip] + sec->length);
    }

    cfg->total_leds = logical_start;
    for (size_t strip = 0; strip < cfg->strip_count; ++strip) {
        cfg->strip_lengths[strip] = strip_offsets[strip];
    }
}

bool led_layout_map_logical_from_config(const led_layout_config_t *cfg,
                                        size_t logical_index,
                                        led_layout_mapping_t *out)
{
    if (!cfg || logical_index >= cfg->total_leds) return false;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        const led_layout_section_t *sec = &cfg->sections[i];
        size_t start = sec->logical_start;
        size_t end = start + sec->length;
        if (logical_index < start || logical_index >= end) continue;
        size_t offset = logical_index - start;
        size_t physical = sec->physical_start + (sec->reversed
            ? ((size_t)sec->length - 1u - offset)
            : offset);
        if (out) {
            out->strip = sec->strip;
            out->physical_index = (uint16_t)physical;
            out->section_index = (uint16_t)i;
            out->section_offset = (uint16_t)offset;
        }
        return true;
    }
    return false;
}

static esp_err_t layout_build_path(char *out, size_t out_sz)
{
    return storage_sd_make_path(out, out_sz, LED_LAYOUT_FILE_NAME);
}

static esp_err_t layout_save_to_sd_locked(const led_layout_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount sd");

    char path[96];
    ESP_RETURN_ON_ERROR(layout_build_path(path, sizeof(path)), TAG, "path");

    char tmp_path[104];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE *f = fopen(tmp_path, "w");
    if (!f) {
        ESP_LOGW(TAG, "open write failed: %s", tmp_path);
        return ESP_FAIL;
    }

    fprintf(f, "version=2\n");
    fprintf(f, "profile=%s\n", cfg->profile);
    fprintf(f, "strip_count=%u\n", (unsigned)cfg->strip_count);
    for (size_t strip = 0; strip < cfg->strip_count; ++strip) {
        fprintf(f, "strip%u_name=%s\n", (unsigned)strip, cfg->strip_names[strip]);
    }
    fprintf(f, "section_count=%u\n", (unsigned)cfg->section_count);
    for (size_t i = 0; i < cfg->section_count; ++i) {
        const led_layout_section_t *sec = &cfg->sections[i];
        fprintf(f, "section%u_name=%s\n", (unsigned)i, sec->name);
        fprintf(f, "section%u_strip=%u\n", (unsigned)i, (unsigned)sec->strip);
        fprintf(f, "section%u_length=%u\n", (unsigned)i, (unsigned)sec->length);
        fprintf(f, "section%u_reversed=%u\n", (unsigned)i, sec->reversed ? 1u : 0u);
        fprintf(f, "section%u_connected=%u\n", (unsigned)i, sec->connected_to_prev ? 1u : 0u);
        fprintf(f, "section%u_geom=%s\n", (unsigned)i, geom_kind_name(sec->geom_kind));
        if (sec->geom_kind == LED_LAYOUT_GEOM_POLYLINE) {
            fprintf(f, "section%u_point_count=%u\n", (unsigned)i, (unsigned)sec->point_count);
            for (size_t p = 0; p < sec->point_count && p < LED_LAYOUT_MAX_SECTION_POINTS; ++p) {
                fprintf(f, "section%u_p%u=%.5f,%.5f,%.5f\n",
                        (unsigned)i, (unsigned)p,
                        sec->points[p].x, sec->points[p].y, sec->points[p].z);
            }
        } else if (sec->geom_kind == LED_LAYOUT_GEOM_ARC) {
            fprintf(f, "section%u_center=%.5f,%.5f,%.5f\n",
                    (unsigned)i, sec->center.x, sec->center.y, sec->center.z);
            fprintf(f, "section%u_radius=%.5f\n", (unsigned)i, sec->radius);
            fprintf(f, "section%u_start_deg=%.5f\n", (unsigned)i, sec->start_deg);
            fprintf(f, "section%u_sweep_deg=%.5f\n", (unsigned)i, sec->sweep_deg);
        }
    }

    fflush(f);
    int fd = fileno(f);
    if (fd >= 0) {
        fsync(fd);
    }
    fclose(f);

    if (rename(tmp_path, path) != 0) {
        int rename_errno = errno;

        // FAT/VFS often refuses rename-over-existing; remove old file and retry.
        if (unlink(path) == 0) {
            if (rename(tmp_path, path) == 0) {
                return ESP_OK;
            }
            rename_errno = errno;
        }

        unlink(tmp_path);
        ESP_LOGW(TAG, "rename failed: %s -> %s (errno=%d)", tmp_path, path, rename_errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static bool layout_parse_key_index(const char *key,
                                   const char *prefix,
                                   const char **out_suffix,
                                   uint32_t *out_index)
{
    if (!key || !prefix || !out_suffix || !out_index) return false;
    size_t prefix_len = strlen(prefix);
    if (strncmp(key, prefix, prefix_len) != 0) return false;
    const char *p = key + prefix_len;
    if (!isdigit((unsigned char)*p)) return false;
    char *end = NULL;
    unsigned long idx = strtoul(p, &end, 10);
    if (end == p || !end || *end != '_') return false;
    *out_suffix = end + 1;
    *out_index = (uint32_t)idx;
    return true;
}

static bool layout_load_from_sd_locked(led_layout_config_t *cfg)
{
    if (!cfg) return false;

    char path[96];
    if (layout_build_path(path, sizeof(path)) != ESP_OK) {
        return false;
    }

    FILE *f = fopen(path, "r");
    if (!f) return false;

    led_layout_config_t tmp;
    layout_fill_default(&tmp);

    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *p = trim(line);
        if (*p == '\0' || *p == '#' || *p == ';') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);

        uint32_t index = 0;
        const char *suffix = NULL;
        if (strcmp(key, "profile") == 0) {
            copy_name(tmp.profile, sizeof(tmp.profile), val);
        } else if (strcmp(key, "strip_count") == 0) {
            uint32_t count = 0;
            if (parse_u32(val, &count) && count > 0) {
                tmp.strip_count = (uint8_t)count;
            }
        } else if (strcmp(key, "section_count") == 0) {
            uint32_t count = 0;
            if (parse_u32(val, &count) && count > 0) {
                tmp.section_count = (uint8_t)count;
            }
        } else if (layout_parse_key_index(key, "strip", &suffix, &index)) {
            if (index < LED_LAYOUT_MAX_STRIPS && strcmp(suffix, "name") == 0) {
                copy_name(tmp.strip_names[index], sizeof(tmp.strip_names[index]), val);
            }
        } else if (layout_parse_key_index(key, "section", &suffix, &index)) {
            if (index >= LED_LAYOUT_MAX_SECTIONS) continue;
            if (index + 1 > tmp.section_count) {
                tmp.section_count = (uint8_t)(index + 1);
            }
            led_layout_section_t *sec = &tmp.sections[index];
            if (strcmp(suffix, "name") == 0) {
                copy_name(sec->name, sizeof(sec->name), val);
            } else if (strcmp(suffix, "strip") == 0) {
                uint32_t strip = 0;
                if (parse_u32(val, &strip)) sec->strip = (uint8_t)strip;
            } else if (strcmp(suffix, "length") == 0) {
                uint32_t length = 0;
                if (parse_u32(val, &length)) sec->length = (uint16_t)length;
            } else if (strcmp(suffix, "reversed") == 0) {
                bool reversed = false;
                if (parse_bool(val, &reversed)) sec->reversed = reversed;
            } else if (strcmp(suffix, "connected") == 0) {
                bool connected = false;
                if (parse_bool(val, &connected)) sec->connected_to_prev = connected;
            } else if (strcmp(suffix, "geom") == 0) {
                sec->geom_kind = geom_kind_from_str(val);
            } else if (strcmp(suffix, "point_count") == 0) {
                uint32_t point_count = 0;
                if (parse_u32(val, &point_count)) sec->point_count = (uint8_t)point_count;
            } else if (strcmp(suffix, "center") == 0) {
                (void)parse_point3(val, &sec->center);
            } else if (strcmp(suffix, "radius") == 0) {
                (void)parse_f32(val, &sec->radius);
            } else if (strcmp(suffix, "start_deg") == 0) {
                (void)parse_f32(val, &sec->start_deg);
            } else if (strcmp(suffix, "sweep_deg") == 0) {
                (void)parse_f32(val, &sec->sweep_deg);
            } else if (suffix[0] == 'p' && isdigit((unsigned char)suffix[1])) {
                uint32_t point_idx = 0;
                if (parse_u32(suffix + 1, &point_idx) &&
                    point_idx < LED_LAYOUT_MAX_SECTION_POINTS) {
                    (void)parse_point3(val, &sec->points[point_idx]);
                }
            }
        }
    }

    fclose(f);
    layout_upgrade_legacy_names(&tmp);
    layout_upgrade_placeholder_geometry(&tmp);
    layout_upgrade_worn_handedness(&tmp);
    layout_upgrade_arm_geometry(&tmp);
    layout_upgrade_ring_orientation(&tmp);
    layout_finalize(&tmp);
    *cfg = tmp;
    return true;
}

static bool section_sample_polyline(const led_layout_section_t *sec, float t, led_point_t *out)
{
    if (!sec || !out || sec->point_count == 0 || sec->point_count > LED_LAYOUT_MAX_SECTION_POINTS) {
        return false;
    }
    if (sec->point_count == 1) {
        *out = sec->points[0];
        return true;
    }

    float total = 0.0f;
    float seg_len[LED_LAYOUT_MAX_SECTION_POINTS - 1] = {0};
    for (size_t i = 0; i + 1 < sec->point_count; ++i) {
        seg_len[i] = point_distance(&sec->points[i], &sec->points[i + 1]);
        total += seg_len[i];
    }
    if (total < LAYOUT_POINT_EPSILON) {
        *out = sec->points[0];
        return true;
    }

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float target = t * total;
    float walked = 0.0f;
    for (size_t i = 0; i + 1 < sec->point_count; ++i) {
        float next = walked + seg_len[i];
        if (target <= next || i + 2 == sec->point_count) {
            float local = (seg_len[i] < LAYOUT_POINT_EPSILON) ? 0.0f : (target - walked) / seg_len[i];
            *out = point_lerp(&sec->points[i], &sec->points[i + 1], local);
            return true;
        }
        walked = next;
    }

    *out = sec->points[sec->point_count - 1];
    return true;
}

static bool section_sample_arc(const led_layout_section_t *sec, float t, led_point_t *out)
{
    if (!sec || !out || fabsf(sec->radius) < LAYOUT_POINT_EPSILON) return false;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float angle_deg = sec->start_deg + sec->sweep_deg * t;
    float angle_rad = angle_deg * ((float)M_PI / 180.0f);
    out->x = sec->center.x + sec->radius * cosf(angle_rad);
    out->y = sec->center.y + sec->radius * sinf(angle_rad);
    out->z = sec->center.z;
    return true;
}

static bool section_sample_point(const led_layout_section_t *sec, size_t offset, led_point_t *out)
{
    if (!sec || !out || sec->length == 0) return false;
    float t = 0.5f;
    if (sec->length > 1) {
        bool closed_arc = sec->geom_kind == LED_LAYOUT_GEOM_ARC &&
                          fabsf(fabsf(sec->sweep_deg) - 360.0f) < 0.001f;
        t = closed_arc
            ? ((float)offset / (float)sec->length)
            : ((float)offset / (float)(sec->length - 1u));
    }

    if (sec->geom_kind == LED_LAYOUT_GEOM_ARC) {
        return section_sample_arc(sec, t, out);
    }
    if (sec->geom_kind == LED_LAYOUT_GEOM_POLYLINE) {
        return section_sample_polyline(sec, t, out);
    }
    return false;
}

esp_err_t led_layout_init(void)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    if (s_ready) {
        layout_lock_give();
        return ESP_OK;
    }

    layout_ensure_ready_locked();

    esp_err_t mount_err = storage_mount_sd();
    if (mount_err == ESP_OK) {
        led_layout_config_t loaded;
        if (layout_load_from_sd_locked(&loaded)) {
            s_layout = loaded;
        } else {
            esp_err_t save_err = layout_save_to_sd_locked(&s_layout);
            if (save_err != ESP_OK) {
                ESP_LOGW(TAG, "default layout save failed: %s", esp_err_to_name(save_err));
            }
        }
    } else {
        ESP_LOGW(TAG, "SD mount failed (%s); using built-in layout", esp_err_to_name(mount_err));
        layout_lock_give();
        return mount_err;
    }

    layout_lock_give();
    return ESP_OK;
}

bool led_layout_ready(void)
{
    return s_ready;
}

void led_layout_snapshot(led_layout_config_t *out)
{
    if (!out) return;
    if (!layout_lock_take()) {
        layout_fill_default(out);
        layout_finalize(out);
        return;
    }
    layout_ensure_ready_locked();
    *out = s_layout;
    layout_lock_give();
}

size_t led_layout_count(void)
{
    if (!layout_lock_take()) return 0;
    layout_ensure_ready_locked();
    size_t total = s_layout.total_leds;
    layout_lock_give();
    return total;
}

size_t led_layout_strip_count(void)
{
    if (!layout_lock_take()) return 0;
    layout_ensure_ready_locked();
    size_t count = s_layout.strip_count;
    layout_lock_give();
    return count;
}

size_t led_layout_strip_length(int strip)
{
    if (!layout_lock_take()) return 0;
    layout_ensure_ready_locked();
    size_t len = 0;
    if (strip >= 0 && strip < s_layout.strip_count) {
        len = s_layout.strip_lengths[strip];
    }
    layout_lock_give();
    return len;
}

bool led_layout_map_logical(size_t logical_index, led_layout_mapping_t *out)
{
    if (!layout_lock_take()) return false;
    layout_ensure_ready_locked();
    bool ok = led_layout_map_logical_from_config(&s_layout, logical_index, out);
    layout_lock_give();
    return ok;
}

bool led_layout_get(int idx, led_point_t *out)
{
    if (!out || idx < 0) return false;
    if (!layout_lock_take()) return false;
    layout_ensure_ready_locked();
    bool ok = led_layout_get_from_config(&s_layout, idx, out);
    layout_lock_give();
    return ok;
}

bool led_layout_get_from_config(const led_layout_config_t *cfg, int idx, led_point_t *out)
{
    if (!cfg || !out || idx < 0) return false;

    led_layout_mapping_t map;
    if (!led_layout_map_logical_from_config(cfg, (size_t)idx, &map)) return false;

    const led_layout_section_t *sec = &cfg->sections[map.section_index];
    return section_sample_point(sec, map.section_offset, out);
}

esp_err_t led_layout_set_section_strip(size_t idx, uint8_t strip)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    if (idx >= s_layout.section_count || strip >= s_layout.strip_count) {
        layout_lock_give();
        return ESP_ERR_INVALID_ARG;
    }
    s_layout.sections[idx].strip = strip;
    layout_finalize(&s_layout);
    layout_lock_give();
    return ESP_OK;
}

esp_err_t led_layout_set_section_length(size_t idx, uint16_t length)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    if (idx >= s_layout.section_count) {
        layout_lock_give();
        return ESP_ERR_INVALID_ARG;
    }

    if (length == 0) length = 1;

    uint32_t other_total = s_layout.total_leds - s_layout.sections[idx].length;
    uint32_t max_len = LED_LAYOUT_MAX_PIXELS - other_total;
    if (max_len == 0) max_len = 1;
    if (length > max_len) length = (uint16_t)max_len;

    s_layout.sections[idx].length = length;
    layout_finalize(&s_layout);
    layout_lock_give();
    return ESP_OK;
}

esp_err_t led_layout_set_section_reversed(size_t idx, bool reversed)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    if (idx >= s_layout.section_count) {
        layout_lock_give();
        return ESP_ERR_INVALID_ARG;
    }
    s_layout.sections[idx].reversed = reversed;
    layout_finalize(&s_layout);
    layout_lock_give();
    return ESP_OK;
}

esp_err_t led_layout_set_section_start_deg(size_t idx, float start_deg)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    if (idx >= s_layout.section_count) {
        layout_lock_give();
        return ESP_ERR_INVALID_ARG;
    }

    led_layout_section_t *sec = &s_layout.sections[idx];
    if (sec->geom_kind != LED_LAYOUT_GEOM_ARC) {
        layout_lock_give();
        return ESP_ERR_INVALID_ARG;
    }

    while (start_deg < 0.0f) start_deg += 360.0f;
    while (start_deg >= 360.0f) start_deg -= 360.0f;

    sec->start_deg = start_deg;
    layout_finalize(&s_layout);
    layout_lock_give();
    return ESP_OK;
}

esp_err_t led_layout_save(void)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    esp_err_t err = layout_save_to_sd_locked(&s_layout);
    layout_lock_give();
    return err;
}

esp_err_t led_layout_reload(void)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_ensure_ready_locked();
    esp_err_t mount_err = storage_mount_sd();
    if (mount_err != ESP_OK) {
        layout_lock_give();
        return mount_err;
    }
    led_layout_config_t loaded;
    if (!layout_load_from_sd_locked(&loaded)) {
        layout_lock_give();
        return ESP_ERR_NOT_FOUND;
    }
    s_layout = loaded;
    layout_lock_give();
    return ESP_OK;
}

esp_err_t led_layout_reset_default(bool persist)
{
    if (!layout_lock_take()) return ESP_ERR_INVALID_STATE;
    layout_fill_default(&s_layout);
    layout_finalize(&s_layout);
    s_ready = true;
    esp_err_t err = ESP_OK;
    if (persist) {
        err = layout_save_to_sd_locked(&s_layout);
    }
    layout_lock_give();
    return err;
}
