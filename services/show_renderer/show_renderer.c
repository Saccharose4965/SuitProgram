#include "show_renderer.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"

#include "led.h"
#include "led_layout.h"
#include "show_layout_ids.h"
#include "show_formation.h"
#include "show_geometry.h"
#include "show_animation.h"
#include "show_color_grid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SHOW_RENDER_MAX_ACTIVE 96u

typedef struct {
    float r;
    float g;
    float b;
    float a;
} show_color_t;

struct show_renderer {
    const show_package_program_t *program;
    uint8_t role_id;
    uint16_t formation;
    size_t led_count;
    show_section_id_t section_ids[LED_LAYOUT_MAX_PIXELS];
    led_point_t points[LED_LAYOUT_MAX_PIXELS];
    led_point_t world_points[LED_LAYOUT_MAX_PIXELS];
    float noise_xy[LED_LAYOUT_MAX_PIXELS][2];
    float noise_min_x, noise_min_y, noise_max_y;
    float section_u[LED_LAYOUT_MAX_PIXELS];
    float radial[LED_LAYOUT_MAX_PIXELS];
    show_color_t colors[LED_LAYOUT_MAX_PIXELS];
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    float role_spacing_x;
    float world_min_x, world_max_x, world_min_z, world_max_z;
    led_point_t global_center;
    led_point_t ring_center;
    led_point_t local_ring_center;
};

static float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static float wrap01(float value)
{
    value -= floorf(value);
    return value < 0.0f ? value + 1.0f : value;
}

static float smoothstep01(float value)
{
    value = clamp01(value);
    return value * value * (3.0f - 2.0f * value);
}

static float param_u16(uint16_t value)
{
    return (float)value / 1024.0f;
}

static uint32_t hash_u32(uint32_t seed, uint8_t role_id, size_t led_index,
                         int32_t cycle_index, uint32_t salt)
{
    uint32_t value = seed ^
        ((uint32_t)(role_id + 1u) * UINT32_C(0x9E3779B9)) ^
        ((uint32_t)(led_index + 1u) * UINT32_C(0x85EBCA6B)) ^
        ((uint32_t)(cycle_index + 1) * UINT32_C(0xC2B2AE35)) ^ salt;
    value ^= value >> 16;
    value *= UINT32_C(0x7FEB352D);
    value ^= value >> 15;
    value *= UINT32_C(0x846CA68B);
    value ^= value >> 16;
    return value;
}

static bool section_is_front(show_section_id_t id)
{
    return (id >= SHOW_SECTION_FRONT_LEFT_TOP && id <= SHOW_SECTION_FRONT_LEFT_BELT) ||
           (id >= SHOW_SECTION_FRONT_RIGHT_TOP && id <= SHOW_SECTION_FRONT_RIGHT_BELT) ||
           id == SHOW_SECTION_LEFT_THIGH_FRONT ||
           id == SHOW_SECTION_LEFT_SHIN_F_IN || id == SHOW_SECTION_LEFT_SHIN_F_OUT ||
           id == SHOW_SECTION_RIGHT_THIGH_FRONT ||
           id == SHOW_SECTION_RIGHT_SHIN_F_IN || id == SHOW_SECTION_RIGHT_SHIN_F_OUT;
}

static bool section_is_back(show_section_id_t id)
{
    return (id >= SHOW_SECTION_BACK_LEFT_BELT && id <= SHOW_SECTION_BACK_LEFT_TOP) ||
           (id >= SHOW_SECTION_BACK_RIGHT_BELT && id <= SHOW_SECTION_BACK_RIGHT_TOP) ||
           id == SHOW_SECTION_LEFT_THIGH_BACK ||
           id == SHOW_SECTION_LEFT_SHIN_B_IN || id == SHOW_SECTION_LEFT_SHIN_B_OUT ||
           id == SHOW_SECTION_RIGHT_THIGH_BACK ||
           id == SHOW_SECTION_RIGHT_SHIN_B_IN || id == SHOW_SECTION_RIGHT_SHIN_B_OUT;
}

static bool section_is_left(show_section_id_t id)
{
    return (id >= SHOW_SECTION_FRONT_LEFT_TOP && id <= SHOW_SECTION_LEFT_FOREARM) ||
           (id >= SHOW_SECTION_LEFT_THIGH_FRONT && id <= SHOW_SECTION_LEFT_SHIN_B_OUT);
}

static bool section_is_right(show_section_id_t id)
{
    return (id >= SHOW_SECTION_FRONT_RIGHT_TOP && id <= SHOW_SECTION_RIGHT_FOREARM) ||
           (id >= SHOW_SECTION_RIGHT_THIGH_FRONT && id <= SHOW_SECTION_RIGHT_SHIN_B_OUT);
}

static bool section_is_torso(show_section_id_t id)
{
    return (id >= SHOW_SECTION_FRONT_LEFT_TOP && id <= SHOW_SECTION_BACK_LEFT_TOP) ||
           (id >= SHOW_SECTION_FRONT_RIGHT_TOP && id <= SHOW_SECTION_BACK_RIGHT_TOP);
}

static bool section_is_leg(show_section_id_t id)
{
    return id >= SHOW_SECTION_LEFT_THIGH_FRONT && id <= SHOW_SECTION_RIGHT_SHIN_B_OUT;
}

static bool group_contains(show_group_id_t group, show_section_id_t id)
{
    switch (group) {
        case SHOW_GROUP_ALL: return id != SHOW_SECTION_INVALID;
        case SHOW_GROUP_FRONT_ALL: return section_is_front(id);
        case SHOW_GROUP_BACK_ALL: return section_is_back(id);
        case SHOW_GROUP_LEFT_ALL: return section_is_left(id);
        case SHOW_GROUP_RIGHT_ALL: return section_is_right(id);
        case SHOW_GROUP_TORSO_ALL: return section_is_torso(id);
        case SHOW_GROUP_ARMS_ALL:
            return id == SHOW_SECTION_LEFT_UPPER_ARM || id == SHOW_SECTION_LEFT_FOREARM ||
                   id == SHOW_SECTION_RIGHT_UPPER_ARM || id == SHOW_SECTION_RIGHT_FOREARM;
        case SHOW_GROUP_LEFT_ARM_ALL:
            return id == SHOW_SECTION_LEFT_UPPER_ARM || id == SHOW_SECTION_LEFT_FOREARM;
        case SHOW_GROUP_RIGHT_ARM_ALL:
            return id == SHOW_SECTION_RIGHT_UPPER_ARM || id == SHOW_SECTION_RIGHT_FOREARM;
        case SHOW_GROUP_RING: return id == SHOW_SECTION_FRONT_LEFT_RING;
        case SHOW_GROUP_BELT_ALL:
            return id == SHOW_SECTION_FRONT_LEFT_BELT || id == SHOW_SECTION_BACK_LEFT_BELT ||
                   id == SHOW_SECTION_FRONT_RIGHT_BELT || id == SHOW_SECTION_BACK_RIGHT_BELT;
        case SHOW_GROUP_SPINE:
            return id == SHOW_SECTION_BACK_LEFT_VERTEBRA ||
                   id == SHOW_SECTION_BACK_RIGHT_VERTEBRA;
        case SHOW_GROUP_LEGS_ALL: return section_is_leg(id);
        case SHOW_GROUP_LEFT_LEG_ALL:
            return id >= SHOW_SECTION_LEFT_THIGH_FRONT && id <= SHOW_SECTION_LEFT_SHIN_B_OUT;
        case SHOW_GROUP_RIGHT_LEG_ALL:
            return id >= SHOW_SECTION_RIGHT_THIGH_FRONT && id <= SHOW_SECTION_RIGHT_SHIN_B_OUT;
        case SHOW_GROUP_LEFT_THIGH_ALL:
            return id == SHOW_SECTION_LEFT_THIGH_FRONT || id == SHOW_SECTION_LEFT_THIGH_BACK;
        case SHOW_GROUP_LEFT_SHIN_ALL:
            return id == SHOW_SECTION_LEFT_SHIN_F_IN || id == SHOW_SECTION_LEFT_SHIN_F_OUT ||
                   id == SHOW_SECTION_LEFT_SHIN_B_IN || id == SHOW_SECTION_LEFT_SHIN_B_OUT;
        case SHOW_GROUP_RIGHT_THIGH_ALL:
            return id == SHOW_SECTION_RIGHT_THIGH_FRONT || id == SHOW_SECTION_RIGHT_THIGH_BACK;
        case SHOW_GROUP_RIGHT_SHIN_ALL:
            return id == SHOW_SECTION_RIGHT_SHIN_F_IN || id == SHOW_SECTION_RIGHT_SHIN_F_OUT ||
                   id == SHOW_SECTION_RIGHT_SHIN_B_IN || id == SHOW_SECTION_RIGHT_SHIN_B_OUT;
        default: return false;
    }
}

static bool clip_targets_led(const show_renderer_t *renderer,
                             const show_clip_v1_t *clip, size_t led)
{
    if (clip->target_kind == SHOW_TARGET_KIND_ALL) return true;
    show_section_id_t section = renderer->section_ids[led];
    if (clip->target_kind == SHOW_TARGET_KIND_SECTION) {
        return section == (show_section_id_t)clip->target_id;
    }
    if (clip->target_kind == SHOW_TARGET_KIND_GROUP) {
        return group_contains((show_group_id_t)clip->target_id, section);
    }
    return false;
}

static float role_world_x(const show_renderer_t *renderer, size_t led)
{
    return renderer->world_points[led].x;
}

static float role_world_z(const show_renderer_t *renderer, size_t led)
{
    return renderer->world_points[led].z;
}

static float local_axis_value(const show_renderer_t *renderer, size_t led,
                              uint8_t axis, float axis_cos, float axis_sin)
{
    const led_point_t *point = &renderer->points[led];
    switch (axis) {
        case SHOW_AXIS_X: return point->x;
        case SHOW_AXIS_Z: return point->z;
        case SHOW_AXIS_SECTION_U: return renderer->section_u[led];
        case SHOW_AXIS_RADIAL: return renderer->radial[led];
        case SHOW_AXIS_RANDOM_XY: return point->x * axis_cos + point->y * axis_sin;
        case SHOW_AXIS_Y:
        default: return point->y;
    }
}

static float world_axis_value(const show_renderer_t *renderer, size_t led,
                              uint8_t axis, float axis_cos, float axis_sin)
{
    float x = role_world_x(renderer, led);
    float z = role_world_z(renderer, led);
    const led_point_t *point = &renderer->points[led];
    switch (axis) {
        case SHOW_AXIS_X: return x;
        case SHOW_AXIS_Z: return z;
        case SHOW_AXIS_SECTION_U: return renderer->section_u[led];
        case SHOW_AXIS_RADIAL:
            return sqrtf(x * x + point->y * point->y + z * z);
        case SHOW_AXIS_RANDOM_XY: return x * axis_cos + point->y * axis_sin;
        case SHOW_AXIS_Y:
        default: return point->y;
    }
}

static void local_target_bounds(const show_renderer_t *renderer,
                                const show_clip_v1_t *clip, uint8_t axis,
                                float axis_cos, float axis_sin,
                                float *lo, float *hi)
{
    bool have = false;
    float minv = 0.0f, maxv = 1.0f;
    for (size_t led = 0; led < renderer->led_count; ++led) {
        if (!clip_targets_led(renderer, clip, led)) continue;
        float value = local_axis_value(renderer, led, axis, axis_cos, axis_sin);
        if (!have) minv = maxv = value;
        else {
            if (value < minv) minv = value;
            if (value > maxv) maxv = value;
        }
        have = true;
    }
    *lo = minv;
    *hi = maxv;
}

static void global_axis_bounds(const show_renderer_t *renderer, uint8_t axis,
                               float axis_cos, float axis_sin,
                               float *lo, float *hi)
{
    float min_x = renderer->world_min_x;
    float max_x = renderer->world_max_x;
    switch (axis) {
        case SHOW_AXIS_X: *lo = min_x; *hi = max_x; return;
        case SHOW_AXIS_Y: *lo = renderer->min_y; *hi = renderer->max_y; return;
        case SHOW_AXIS_Z: *lo = renderer->world_min_z; *hi = renderer->world_max_z; return;
        case SHOW_AXIS_SECTION_U: *lo = 0.0f; *hi = 1.0f; return;
        case SHOW_AXIS_RANDOM_XY: {
            float x0 = min_x * axis_cos;
            float x1 = max_x * axis_cos;
            float y0 = renderer->min_y * axis_sin;
            float y1 = renderer->max_y * axis_sin;
            *lo = fminf(x0, x1) + fminf(y0, y1);
            *hi = fmaxf(x0, x1) + fmaxf(y0, y1);
            return;
        }
        case SHOW_AXIS_RADIAL:
        default: {
            *lo = 0.0f;
            float dx = fmaxf(fabsf(min_x), fabsf(max_x));
            float dy = fmaxf(fabsf(renderer->min_y), fabsf(renderer->max_y));
            float dz = fmaxf(fabsf(renderer->world_min_z), fabsf(renderer->world_max_z));
            *hi = sqrtf(dx * dx + dy * dy + dz * dz);
            return;
        }
    }
}

static void blend_pixel(show_color_t *dst, uint8_t mode,
                        float r, float g, float b, float a)
{
    r = clamp01(r);
    g = clamp01(g);
    b = clamp01(b);
    a = clamp01(a);
    if (mode == SHOW_BLEND_REPLACE) {
        *dst = (show_color_t){r, g, b, a};
    } else if (mode == SHOW_BLEND_ADD) {
        dst->r = clamp01(dst->r + r * a);
        dst->g = clamp01(dst->g + g * a);
        dst->b = clamp01(dst->b + b * a);
        dst->a = fmaxf(dst->a, a);
    } else if (mode == SHOW_BLEND_MAX) {
        dst->r = fmaxf(dst->r, r * a);
        dst->g = fmaxf(dst->g, g * a);
        dst->b = fmaxf(dst->b, b * a);
        dst->a = fmaxf(dst->a, a);
    } else {
        float inv = 1.0f - a;
        dst->r = clamp01(r * a + dst->r * inv);
        dst->g = clamp01(g * a + dst->g * inv);
        dst->b = clamp01(b * a + dst->b * inv);
        dst->a = clamp01(a + dst->a * inv);
    }
}

static float clip_phase(const show_renderer_t *renderer,
                        const show_clip_v1_t *clip,
                        const show_param_spatial_v1_t *param,
                        uint32_t playhead_ms, int32_t *cycle_index)
{
    float rate = param_u16(param->frequency_1024);
    float phase = param_u16(param->phase_1024);
    float cycles;
    if (clip->flags & SHOW_CLIP_FLAG_FIT_CLIP) {
        cycles=(float)(playhead_ms-clip->start_ms)/fmaxf(1,clip->end_ms-clip->start_ms)*rate+phase;
    } else if ((clip->flags & SHOW_CLIP_FLAG_TEMPO_SYNC) &&
        renderer->program->info.tempo_millibpm > 0) {
        float bpm = (float)renderer->program->info.tempo_millibpm / 1000.0f;
        float beat_ms = 60000.0f / bpm;
        cycles = (((float)playhead_ms - (float)renderer->program->info.beat_offset_ms) /
                  beat_ms) * fmaxf(rate, 0.0625f) + phase;
    } else {
        float elapsed_sec = (float)(playhead_ms - clip->start_ms) / 1000.0f;
        cycles = elapsed_sec * rate + phase;
    }
    if (cycle_index) *cycle_index = (int32_t)floorf(cycles);
    return wrap01(cycles);
}

static float clip_envelope(const show_clip_v1_t *clip, uint32_t playhead_ms)
{
    float envelope = 1.0f;
    uint32_t fade_in_ms = (uint32_t)clip->fade_in_ms_div10 * 10u;
    uint32_t fade_out_ms = (uint32_t)clip->fade_out_ms_div10 * 10u;
    if (fade_in_ms > 0 && playhead_ms < clip->start_ms + fade_in_ms) {
        envelope = fminf(envelope, (float)(playhead_ms - clip->start_ms) / fade_in_ms);
    }
    if (fade_out_ms > 0 && playhead_ms + fade_out_ms > clip->end_ms) {
        envelope = fminf(envelope, (float)(clip->end_ms - playhead_ms) / fade_out_ms);
    }
    return clamp01(envelope);
}

static show_color_t clip_color_at(const show_renderer_t *renderer,
                                  const show_clip_v1_t *clip,
                                  const void *base_param, size_t base_bytes,
                                  uint8_t r, uint8_t g, uint8_t b, uint8_t a,
                                  uint32_t playhead_ms, show_color_grid_t *grid)
{
    grid->rows=0;
    show_color_t fallback = {
        .r = (float)r / 255.0f,
        .g = (float)g / 255.0f,
        .b = (float)b / 255.0f,
        .a = (float)a / 255.0f,
    };
    if (!base_param || clip->param_bytes < base_bytes + sizeof(show_color_anim_v1_1_t)) {
        return fallback;
    }
    if (((const uint8_t *)base_param)[base_bytes]==4 || ((const uint8_t *)base_param)[base_bytes]==5) {
        float elapsed=(playhead_ms-clip->start_ms)/1000.0f;
        float duration=(clip->end_ms-clip->start_ms)/1000.0f;
        float beats=((float)playhead_ms-renderer->program->info.beat_offset_ms)/60000*(renderer->program->info.tempo_millibpm/1000.0f);
        show_grid_prepare(grid,(const uint8_t *)base_param+base_bytes,elapsed,duration,beats);
        return (show_color_t){grid->color[0][0],grid->color[0][1],grid->color[0][2],grid->color[0][3]};
    }

    show_color_anim_v1_1_t animation;
    memcpy(&animation, (const uint8_t *)base_param + base_bytes, sizeof(animation));
    size_t required = base_bytes + sizeof(animation) +
                      (size_t)animation.stop_count * sizeof(show_color_stop_v1_1_t);
    if (animation.mode < SHOW_COLOR_MODE_LINEAR ||
        animation.mode > SHOW_COLOR_MODE_CYCLE || animation.stop_count == 0 ||
        animation.stop_count > 16u || required != clip->param_bytes) {
        return fallback;
    }

    float duration = (float)(clip->end_ms - clip->start_ms);
    float clip_t = clamp01((float)(playhead_ms - clip->start_ms) /
                           fmaxf(duration, 1.0f));
    float color_t = clip_t;
    if (animation.mode == SHOW_COLOR_MODE_SMOOTH) {
        color_t = smoothstep01(color_t);
    } else if (animation.mode == SHOW_COLOR_MODE_CYCLE) {
        float rate = (float)animation.rate_million / 1000000.0f;
        if (animation.flags & SHOW_COLOR_FLAG_FIT_CLIP) {
            color_t = clip_t * fmaxf(rate, 0.0625f);
        } else if ((animation.flags & SHOW_COLOR_FLAG_TEMPO_SYNC) &&
                   renderer->program->info.tempo_millibpm > 0) {
            float bpm = (float)renderer->program->info.tempo_millibpm / 1000.0f;
            float beat_ms = 60000.0f / bpm;
            color_t = (((float)playhead_ms -
                        (float)renderer->program->info.beat_offset_ms) / beat_ms) *
                      fmaxf(rate, 0.0625f);
        } else if (rate > 0.0f) {
            color_t = ((float)(playhead_ms - clip->start_ms) / 1000.0f) * rate;
        }
        color_t = wrap01(color_t);
    }

    const uint8_t *stop_data = (const uint8_t *)base_param + base_bytes +
                               sizeof(animation);
    show_color_stop_v1_1_t left;
    memcpy(&left, stop_data, sizeof(left));
    if (animation.stop_count == 1 || color_t <= param_u16(left.position_1024)) {
        return (show_color_t){(float)left.r / 255.0f, (float)left.g / 255.0f,
                              (float)left.b / 255.0f, (float)left.a / 255.0f};
    }
    for (uint8_t i = 1; i < animation.stop_count; ++i) {
        show_color_stop_v1_1_t right;
        memcpy(&right, stop_data + i * sizeof(right), sizeof(right));
        float right_t = param_u16(right.position_1024);
        if (color_t <= right_t || i + 1u == animation.stop_count) {
            float left_t = param_u16(left.position_1024);
            float mix = right_t <= left_t ? 0.0f
                : clamp01((color_t - left_t) / (right_t - left_t));
            return (show_color_t){
                lerpf((float)left.r / 255.0f, (float)right.r / 255.0f, mix),
                lerpf((float)left.g / 255.0f, (float)right.g / 255.0f, mix),
                lerpf((float)left.b / 255.0f, (float)right.b / 255.0f, mix),
                lerpf((float)left.a / 255.0f, (float)right.a / 255.0f, mix),
            };
        }
        left = right;
    }
    return fallback;
}

static void render_uniform(show_renderer_t *renderer, const show_clip_v1_t *clip,
                           float r, float g, float b, float a, float scale)
{
    show_color_t source = {r * scale, g * scale, b * scale, a * scale};
    show_color_t previous = {0}, blended = {0};
    bool cached = false;
    if (clip->blend_mode == SHOW_BLEND_REPLACE) {
        blend_pixel(&blended, clip->blend_mode, source.r, source.g, source.b, source.a);
    }
    for (size_t led = 0; led < renderer->led_count; ++led) {
        if (!clip_targets_led(renderer, clip, led)) continue;
        show_color_t *dst = &renderer->colors[led];
        // Uniform source over identical destinations has an identical result.
        // Keep layer/alpha semantics, including REPLACE with a zero blink level.
        if (clip->blend_mode != SHOW_BLEND_REPLACE &&
            (!cached || memcmp(dst, &previous, sizeof(previous)) != 0)) {
            previous = *dst;
            blended = *dst;
            blend_pixel(&blended, clip->blend_mode, source.r, source.g, source.b, source.a);
            cached = true;
        }
        *dst = blended;
    }
}

static void pack_colors(const show_color_t *colors, size_t count,
                        uint8_t brightness, uint8_t *frame)
{
    float scale = (float)brightness / 255.0f;
    show_color_t previous = {0};
    uint8_t r = 0, g = 0, b = 0;
    for (size_t led = 0; led < count; ++led) {
        show_color_t color = colors[led];
        // Alpha is already composited. Reuse RGB conversion across flat spans;
        // spatial effects still use the exact same clamp and rounding formula.
        if (!led || color.r != previous.r || color.g != previous.g || color.b != previous.b) {
            r = (uint8_t)lroundf(clamp01(color.r) * 255.0f * scale);
            g = (uint8_t)lroundf(clamp01(color.g) * 255.0f * scale);
            b = (uint8_t)lroundf(clamp01(color.b) * 255.0f * scale);
            previous = color;
        }
        led_set_pixel_rgb(frame, led, r, g, b);
    }
}

static led_point_t ray_origin(const show_renderer_t *r, const show_clip_v1_t *clip,
                              const show_param_ray_v1_3_t *ray)
{
    if (!ray) return r->ring_center;
    if (ray->origin == 6) return (led_point_t){ray->x, ray->y, ray->z};
    if (ray->origin == 1) {
        led_point_t lo = {0}, hi = {0};
        bool first = true;
        for (uint8_t role = 0; role < (ray->space ? 1 : SHOW_MAX_ROLES); ++role) {
            for (size_t led = 0; led < r->led_count; ++led) {
                if (!clip_targets_led(r, clip, led)) continue;
                led_point_t p = r->points[led];
                if (!ray->space) p = show_formation_point(p, r->formation, role, r->role_spacing_x);
                if (first) { lo = hi = p; first = false; }
                else {
                    lo.x = fminf(lo.x, p.x); hi.x = fmaxf(hi.x, p.x);
                    lo.y = fminf(lo.y, p.y); hi.y = fmaxf(hi.y, p.y);
                    lo.z = fminf(lo.z, p.z); hi.z = fmaxf(hi.z, p.z);
                }
            }
        }
        return (led_point_t){(lo.x+hi.x)*0.5f, (lo.y+hi.y)*0.5f, (lo.z+hi.z)*0.5f};
    }
    if (ray->space) return r->local_ring_center;
    if (ray->origin == 0) {
        led_point_t sum = {0};
        for (uint8_t role = 0; role < SHOW_MAX_ROLES; ++role) {
            led_point_t p = show_formation_point(r->local_ring_center, r->formation, role, r->role_spacing_x);
            sum.x += p.x; sum.y += p.y; sum.z += p.z;
        }
        return (led_point_t){sum.x/3, sum.y/3, sum.z/3};
    }
    uint8_t role = ray->origin >= 3 && ray->origin <= 5 ? ray->origin - 3 : SHOW_ROLE_B;
    return show_formation_point(r->local_ring_center, r->formation, role, r->role_spacing_x);
}

/* Compute once per clip/frame; only 3D distance remains inside the LED loop. */
static led_point_t orb_pose(const show_renderer_t *r, const show_clip_v1_t *clip,
                            const show_param_spatial_v1_t *p, float travel,
                            int32_t cycle, float width, float softness,
                            float dx, float dy, float lo, float hi,
                            float *radius, float *feather)
{
    bool v14 = r->program->info.version_minor >= 4;
    bool diagonal = p->axis_kind == SHOW_AXIS_DIAGONAL_XY;
    bool random = p->axis_kind == SHOW_AXIS_RANDOM_XY;
    bool cylinder = v14 && r->formation == SHOW_FORMATION_CIRCLE &&
        (p->axis_kind == SHOW_AXIS_X || p->axis_kind == SHOW_AXIS_Y || diagonal || random);
    if (diagonal) { dx = dy = sqrtf(0.5f); }
    else if (!random) { dx = p->axis_kind == SHOW_AXIS_Y ? 0 : 1; dy = p->axis_kind == SHOW_AXIS_Y ? 1 : 0; }
    if (cylinder || (v14 && (diagonal || random))) {
        /* Use the Line-layout extent for size, including exact diagonal bounds. */
        bool first = true;
        for (size_t i = 0; i < r->led_count; ++i) {
            if (r->section_ids[i] == SHOW_SECTION_INVALID) continue;
            float value = r->points[i].x * dx + r->points[i].y * dy;
            if (first) { lo = hi = value; first = false; }
            else { lo = fminf(lo, value); hi = fmaxf(hi, value); }
        }
        lo -= r->role_spacing_x * fabsf(dx);
        hi += r->role_spacing_x * fabsf(dx);
    }
    float span = fmaxf(1.0f, hi - lo);
    *radius = span * width * 0.5f;
    *feather = span * softness * 0.5f;
    float margin = *radius + *feather * 0.5f;
    bool random_start = p->axis_mix_1024 & SHOW_SPATIAL_OPTION_RANDOM_CROSS_X;
    float unit = random_start ? (float)hash_u32(clip->seed, 0, 0, cycle, UINT32_C(0x54A9D93B)) / 4294967295.0f : 0;
    if (cylinder) {
        float path_radius = fmaxf(1.0f, r->role_spacing_x / sqrtf(3.0f) - (r->min_z + r->max_z) * 0.5f);
        float angle, y;
        if (dy == 0.0f) {
            angle = (unit + copysignf(travel, dx)) * (float)(2.0 * M_PI);
            y = (r->min_y + r->max_y) * 0.5f;
        } else {
            float height = r->max_y - r->min_y + 2.0f * margin;
            float turns = dx == 0.0f ? 0.0f : copysignf(
                fmaxf(0.5f, fminf(1.5f, fabsf(dx / dy) * height / ((float)(2.0 * M_PI) * path_radius))), dx);
            angle = (unit + (travel - 0.5f) * turns) * (float)(2.0 * M_PI);
            y = (r->min_y + r->max_y) * 0.5f + (travel - 0.5f) * copysignf(height, dy);
        }
        return (led_point_t){path_radius * sinf(angle), y, path_radius * cosf(angle)};
    }
    float dz = 0;
    if (p->axis_kind == SHOW_AXIS_Z) { dx = dy = 0; dz = 1; }
    led_point_t anchor = r->global_center;
    if (random_start) anchor.x = lerpf(r->world_min_x, r->world_max_x, unit);
    float projection = anchor.x * dx + anchor.y * dy + anchor.z * dz;
    float head = lerpf(lo - margin, hi + margin, travel);
    return (led_point_t){anchor.x + dx * (head - projection),
                        anchor.y + dy * (head - projection),
                        anchor.z + dz * (head - projection)};
}

static size_t apply_animations(const show_renderer_t *r, const show_clip_v1_t *clip,
                              const void *raw, size_t offset, uint32_t ms,
                              float *values, float *cycles, const show_spatial_color_t **gradient)
{
    if (r->program->info.version_minor < 5) return offset;
    show_animation_header_t header;
    memcpy(&header, (const uint8_t *)raw+offset, sizeof(header));
    offset += sizeof(header);
    float elapsed=(ms-clip->start_ms)/1000.0f, duration=(clip->end_ms-clip->start_ms)/1000.0f;
    float bpm=r->program->info.tempo_millibpm/1000.0f;
    for (unsigned i=0; i<header.count; ++i) {
        show_animation_t a;
        memcpy(&a,(const uint8_t *)raw+offset,sizeof(a));
        offset += sizeof(a);
        if (a.mode == 4) {
            show_amplitude_curve_t curve;
            memcpy(&curve,(const uint8_t *)raw+offset,sizeof(curve));
            offset += sizeof(curve);
            values[a.key]=show_amplitude_value(&a,(const uint8_t *)raw+offset,curve,ms-clip->start_ms);
            offset += (size_t)curve.count*2;
            continue;
        }
        float integral;
        if (r->program->info.version_minor<8 && a.shape<=1) a.duty=1;
        float t=elapsed;
        if (a.key == 9 && a.tempo_sync)
            t=((float)ms-r->program->info.beat_offset_ms)/1000;
        if (a.key == 9 && a.shape <= 1 && a.rate>0)
            t+=a.duty/(2*a.rate*(a.tempo_sync ? bpm/60 : 1));
        values[a.key]=show_animation_value(&a,t,duration,bpm,&integral);
        if (r->program->info.version_minor<8 && a.key==9 && a.shape<=1 && a.rate==0) values[a.key]=a.to;
        if (a.key == 3 && cycles) {
            if (clip->flags & SHOW_CLIP_FLAG_FIT_CLIP) integral/=fmaxf(.001f,duration);
            else if (clip->flags & SHOW_CLIP_FLAG_TEMPO_SYNC)
                integral=integral*bpm/60 + ((float)clip->start_ms-r->program->info.beat_offset_ms)/1000*bpm/60*a.from;
            *cycles=integral;
        }
    }
    *gradient=(const show_spatial_color_t *)((const uint8_t *)raw+offset);
    return offset+sizeof(show_spatial_color_t);
}

static show_color_t spatial_color_at(const show_renderer_t *r, const show_spatial_color_t *g,
                                    show_color_t color, size_t led, float u,
                                    const show_color_grid_t *grid)
{
    if (!g || !g->enabled) return color;
    led_point_t p=r->points[led];
    if (!g->space) p=show_formation_point(p,r->formation,r->role_id,r->role_spacing_x);
    if (g->mapping==1) u=r->section_u[led];
    else if (g->mapping>=2 && g->mapping<=4) {
        float lo,hi,v;
        if (g->mapping==2) {lo=g->space?r->min_x:r->world_min_x;hi=g->space?r->max_x:r->world_max_x;v=p.x;}
        else if (g->mapping==3) {lo=r->min_y;hi=r->max_y;v=p.y;}
        else {lo=g->space?r->min_z:r->world_min_z;hi=g->space?r->max_z:r->world_max_z;v=p.z;}
        u=(v-lo)/fmaxf(.001f,hi-lo);
    } else if (g->mapping==6) {
        float unit=grid->noise.scale/fmaxf(.001f,r->noise_max_y-r->noise_min_y);
        u=show_noise_contour((r->noise_xy[led][0]-r->noise_min_x)*unit+.371f,
                             (r->noise_xy[led][1]-r->noise_min_y)*unit+.619f,
                             grid->drift+.217f,grid->noise.width);
    }
    u=clamp01(u); if(g->reverse) u=1-u;
    if (grid->rows>1) {
        float rgba[4]; show_grid_sample(grid,u,rgba);
        return (show_color_t){rgba[0],rgba[1],rgba[2],rgba[3]};
    }
    return (show_color_t){lerpf(color.r,g->r/255.0f,u),lerpf(color.g,g->g/255.0f,u),
                          lerpf(color.b,g->b/255.0f,u),lerpf(color.a,g->a/255.0f,u)};
}

static void render_spatial(show_renderer_t *renderer, const show_clip_v1_t *clip,
                           const show_param_spatial_v1_t *param,
                           uint32_t playhead_ms)
{
    float duration = (float)(clip->end_ms - clip->start_ms);
    float clip_t = clamp01((float)(playhead_ms - clip->start_ms) / fmaxf(duration, 1.0f));
    const show_param_ray_v1_3_t *ray = NULL;
    size_t base_bytes = sizeof(*param);
    if ((clip->effect_kind == SHOW_EFFECT_RADIAL_RAY && renderer->program->info.version_minor >= 3)) {
        ray = (const show_param_ray_v1_3_t *)((const uint8_t *)param + base_bytes);
        base_bytes += sizeof(*ray);
    }
    float values[SHOW_ANIMATION_KEYS] = {
        param_u16(param->intensity_1024), param_u16(param->width_1024), param_u16(param->softness_1024),
        param_u16(param->frequency_1024), param_u16(param->phase_1024), param->repeats,
        ray ? ray->x : 0, ray ? ray->y : 0, ray ? ray->z : 0};
    float animated_cycles=NAN;
    const show_spatial_color_t *gradient=NULL;
    base_bytes=apply_animations(renderer,clip,param,base_bytes,playhead_ms,values,&animated_cycles,&gradient);
    show_color_grid_t grid;
    show_color_t color = clip_color_at(renderer, clip, param, base_bytes,
                                       param->r, param->g, param->b, param->a,
                                       playhead_ms, &grid);
    float intensity = values[0] * clip_envelope(clip, playhead_ms);
    float width = fmaxf(0.02f, values[1]);
    float softness = fmaxf(0.001f, values[2]);
    show_param_spatial_v1_t evaluated=*param;
    show_param_ray_v1_3_t evaluated_ray;
    if (ray) {
        evaluated_ray=*ray;
        evaluated_ray.x=values[6]; evaluated_ray.y=values[7]; evaluated_ray.z=values[8];
        ray=&evaluated_ray;
    }
    evaluated.phase_1024=(uint16_t)lroundf(values[4]*1024);
    param=&evaluated;
    int32_t cycle_index = 0;
    float phase = clip_phase(renderer, clip, param, playhead_ms, &cycle_index);
    if (isfinite(animated_cycles)) {
        float cycles=animated_cycles+values[4];
        cycle_index=(int32_t)floorf(cycles);
        phase=wrap01(cycles);
    }
    bool orb_v14 = clip->effect_kind == SHOW_EFFECT_TRAVELING_ORB && renderer->program->info.version_minor >= 4;
    if (orb_v14 && !(clip->flags & (SHOW_CLIP_FLAG_TEMPO_SYNC | SHOW_CLIP_FLAG_CYCLE_MOTION))) cycle_index = 0;
    int32_t axis_cycle = orb_v14 && cycle_index < 0 ? 0 : cycle_index;
    float axis_angle = ((float)hash_u32(clip->seed, 0, 0, axis_cycle,
                                        UINT32_C(0x41C6CE57)) /
                        4294967295.0f) * (float)(2.0 * M_PI);
    float axis_cos = cosf(axis_angle);
    float axis_sin = sinf(axis_angle);
    float local_lo = 0.0f, local_hi = 1.0f;
    local_target_bounds(renderer, clip, param->axis_kind, axis_cos, axis_sin,
                        &local_lo, &local_hi);
    float local_inv = fabsf(local_hi - local_lo) < 1e-6f
        ? 0.0f : 1.0f / (local_hi - local_lo);
    float global_lo = 0.0f, global_hi = 1.0f;
    global_axis_bounds(renderer, param->axis_kind, axis_cos, axis_sin,
                       &global_lo, &global_hi);
    float global_inv = fabsf(global_hi - global_lo) < 1e-6f
        ? 0.0f : 1.0f / (global_hi - global_lo);
    float travel = (clip->flags & (SHOW_CLIP_FLAG_TEMPO_SYNC | SHOW_CLIP_FLAG_CYCLE_MOTION)) ? phase : clip_t;
    if (clip->effect_kind == SHOW_EFFECT_SPARKLE ||
        clip->effect_kind == SHOW_EFFECT_FANOUT ||
        clip->effect_kind == SHOW_EFFECT_RADIAL_RAY) {
        travel = phase;
    }
    if (param->reverse) travel = 1.0f - travel;

    float core_half = width * 0.5f;
    float margin = core_half + softness;
    float head = lerpf(-margin, 1.0f + margin, travel);
    uint16_t repeats = (uint16_t)fmaxf(1, floorf(values[5]+.5f));
    led_point_t ray_center = clip->effect_kind == SHOW_EFFECT_RADIAL_RAY ? ray_origin(renderer, clip, ray) : (led_point_t){0};
    float orb_radius = 0, orb_feather = 0;
    led_point_t orb_center = {0};
    float orb_front_d2 = 0, orb_back_d2 = 0, orb_outer2 = 0;
    if (clip->effect_kind == SHOW_EFFECT_TRAVELING_ORB) {
        orb_center = orb_pose(renderer, clip, param, travel, cycle_index, width, softness,
                              axis_cos, axis_sin, global_lo, global_hi, &orb_radius, &orb_feather);
        orb_center = show_formation_local(orb_center, renderer->formation,
                                          renderer->role_id, renderer->role_spacing_x);
        orb_front_d2 = (12.0f-orb_center.z)*(12.0f-orb_center.z);
        orb_back_d2 = (-12.0f-orb_center.z)*(-12.0f-orb_center.z);
        orb_outer2 = (orb_radius+orb_feather)*(orb_radius+orb_feather);
    }

    for (size_t led = 0; led < renderer->led_count; ++led) {
        if (!clip_targets_led(renderer, clip, led)) continue;
        float falloff = 1.0f;
        float core=fabsf(renderer->section_u[led]-.5f)*2;

        if (clip->effect_kind == SHOW_EFFECT_SWEEP) {
            float value = local_axis_value(renderer, led, param->axis_kind, axis_cos, axis_sin);
            float pos = param->axis_kind == SHOW_AXIS_SECTION_U
                ? renderer->section_u[led]
                : (local_inv == 0.0f ? 0.5f : (value - local_lo) * local_inv);
            float distance = fabsf(pos - head);
            if (repeats>1 || (param->axis_mix_1024 & SHOW_SPATIAL_OPTION_WRAP)) {
                float local=wrap01(pos*repeats-travel);
                distance=fminf(local,1-local);
            }
            core=distance/core_half;
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_FANOUT) {
            float pos = fabsf(renderer->section_u[led] - 0.5f) * 2.0f;
            float distance = fabsf(pos - head);
            core=distance/core_half;
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_SPARKLE) {
            uint32_t hash = hash_u32(clip->seed, renderer->role_id, led,
                                     cycle_index, UINT32_C(0x5A17));
            float offset = (float)(hash & 0xFFFFu) / 65535.0f;
            float amplitude = 0.35f + 0.65f *
                ((float)(((hash >> 16) ^ hash) & 0xFFFFu) / 65535.0f);
            float local = wrap01(travel + offset);
            falloff = fmaxf(0.0f, 1.0f - fabsf(local * 2.0f - 1.0f));
            falloff = falloff * falloff * amplitude;
        } else if (clip->effect_kind == SHOW_EFFECT_GLOBAL_SWEEP) {
            float value = world_axis_value(renderer, led, param->axis_kind, axis_cos, axis_sin);
            float pos = global_inv == 0.0f ? 0.5f : (value - global_lo) * global_inv;
            float distance = fabsf(pos - head);
            core=distance/core_half;
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_RADIAL_RAY) {
            led_point_t p = renderer->points[led];
            if (!ray || !ray->space)
                p = show_formation_point(p, renderer->formation, renderer->role_id, renderer->role_spacing_x);
            float dx = p.x - ray_center.x, dy = p.y - ray_center.y, dz = p.z - ray_center.z;
            float a = ray && ray->plane == 2 ? dz : dx;
            float b = ray && ray->plane == 1 ? dz : -dy;
            // Coplanar points must not pick opposite angles from roundoff or -0.
            if (fabsf(a) < 1e-5f) a = 0.0f;
            if (fabsf(b) < 1e-5f) b = 0.0f;
            float angle = atan2f(a, b);
            angle = wrap01(angle / (float)(2.0 * M_PI));
            float distance = fabsf(wrap01(angle - travel + 0.5f) - 0.5f);
            core=distance/core_half;
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_TRAVELING_ORB) {
            float dx = renderer->points[led].x - orb_center.x;
            float dy = renderer->points[led].y - orb_center.y;
            float z = renderer->points[led].z;
            float dz = z - orb_center.z;
            // Sphere/plane intersection: only in-panel X/Y vary per LED.
            float depth2 = z == 12.0f ? orb_front_d2 :
                           z == -12.0f ? orb_back_d2 : dz*dz;
            float distance2 = dx*dx + dy*dy + depth2;
            float distance = distance2 > orb_outer2
                ? orb_radius+fmaxf(orb_feather, 1e-6f) : sqrtf(distance2);
            core=distance/fmaxf(1e-6f,orb_radius);
            falloff = distance <= orb_radius ? 1.0f
                : clamp01(1.0f - (distance - orb_radius) / fmaxf(orb_feather, 1e-6f));
        } else if (clip->effect_kind == SHOW_EFFECT_GROUND_ENERGY) {
            float edge = fmaxf(1e-6f, (renderer->max_y - renderer->min_y) * softness);
            float level = lerpf(renderer->max_y + edge, renderer->min_y, travel);
            float y = renderer->points[led].y;
            core=(renderer->max_y-y)/fmaxf(1e-6f,renderer->max_y-level);
            falloff = y >= level ? 1.0f : clamp01(1.0f - (level - y) / edge);
        }

        float scale = intensity * falloff;
        show_color_t shaded=spatial_color_at(renderer,gradient,color,led,renderer->program->info.version_minor>=9 ? core : 1-falloff,&grid);
        blend_pixel(&renderer->colors[led], clip->blend_mode,
                    shaded.r * scale, shaded.g * scale, shaded.b * scale, shaded.a * scale);
    }
}

static float fill_mask(const show_renderer_t *r, size_t led, bool local, const float *values, float *core)
{
    led_point_t p=r->points[led];
    if (!local) p=show_formation_point(p,r->formation,r->role_id,r->role_spacing_x);
    float coords[3]={p.x,p.y,p.z};
    float lo[3]={local?r->min_x:r->world_min_x,r->min_y,local?r->min_z:r->world_min_z};
    float hi[3]={local?r->max_x:r->world_max_x,r->max_y,local?r->max_z:r->world_max_z};
    float mask=1; *core=0;
    for (unsigned a=0;a<3;++a) {
        float lower=values[10+a*2],upper=values[11+a*2];
        if (lower>=upper) return 0;
        float pos=hi[a]-lo[a]>.001f ? (coords[a]-lo[a])/(hi[a]-lo[a]) : .5f;
        if (a==1) pos=1-pos; /* Fill Y: feet=0, head=1. */
        float outside=fmaxf(0,fmaxf(lower-pos,pos-upper));
        mask=fminf(mask,values[2]>0 ? clamp01(1-outside/values[2]) : (outside==0 ? 1 : 0));
        if (lower!=0 || upper!=1) *core=fmaxf(*core,fabsf(pos-(lower+upper)*.5f)/fmaxf(.001f,(upper-lower)*.5f));
    }
    return mask;
}

show_renderer_t *show_renderer_create(void)
{
    show_renderer_t *renderer = heap_caps_calloc(1, sizeof(*renderer),
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return renderer ? renderer : heap_caps_calloc(1, sizeof(*renderer), MALLOC_CAP_8BIT);
}

void show_renderer_destroy(show_renderer_t *renderer)
{
    heap_caps_free(renderer);
}

static void set_formation(show_renderer_t *renderer, uint16_t formation)
{
    renderer->formation = formation;
    for (size_t led = 0; led < renderer->led_count; ++led)
        renderer->world_points[led] = show_formation_point(renderer->points[led],
            formation, renderer->role_id, renderer->role_spacing_x);
    renderer->world_min_x = renderer->min_x - renderer->role_spacing_x;
    renderer->world_max_x = renderer->max_x + renderer->role_spacing_x;
    renderer->world_min_z = renderer->min_z;
    renderer->world_max_z = renderer->max_z;
    if (renderer->formation == SHOW_FORMATION_CIRCLE) {
        bool first = true;
        for (uint8_t role = 0; role < SHOW_MAX_ROLES; ++role) {
            for (size_t led = 0; led < renderer->led_count; ++led) {
                if (renderer->section_ids[led] == SHOW_SECTION_INVALID) continue;
                led_point_t p = show_formation_point(renderer->points[led],
                    renderer->formation, role, renderer->role_spacing_x);
                if (first) {
                    renderer->world_min_x = renderer->world_max_x = p.x;
                    renderer->world_min_z = renderer->world_max_z = p.z;
                    first = false;
                } else {
                    renderer->world_min_x = fminf(renderer->world_min_x, p.x);
                    renderer->world_max_x = fmaxf(renderer->world_max_x, p.x);
                    renderer->world_min_z = fminf(renderer->world_min_z, p.z);
                    renderer->world_max_z = fmaxf(renderer->world_max_z, p.z);
                }
            }
        }
    }
    renderer->global_center = (led_point_t){
        .x = (renderer->world_min_x + renderer->world_max_x) * 0.5f,
        .y = (renderer->min_y + renderer->max_y) * 0.5f,
        .z = (renderer->world_min_z + renderer->world_max_z) * 0.5f,
    };
    renderer->ring_center = show_formation_point(renderer->local_ring_center,
        formation, SHOW_ROLE_B, renderer->role_spacing_x);
}

esp_err_t show_renderer_configure(show_renderer_t *renderer,
                                  const show_package_program_t *program,
                                  uint8_t role_id)
{
    if (!renderer || !program || role_id >= SHOW_MAX_ROLES ||
        program->info.formation > SHOW_FORMATION_CIRCLE) return ESP_ERR_INVALID_ARG;
    led_layout_config_t *layout = heap_caps_malloc(sizeof(*layout), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!layout) return ESP_ERR_NO_MEM;
    led_layout_snapshot(layout);
    if (layout->total_leds == 0 || layout->total_leds > LED_LAYOUT_MAX_PIXELS) {
        heap_caps_free(layout);
        return ESP_ERR_INVALID_STATE;
    }

    memset(renderer, 0, sizeof(*renderer));
    renderer->program = program;
    renderer->role_id = role_id;
    renderer->led_count = layout->total_leds;
    bool have_point = false;
    led_point_t ring_sum = {0};
    size_t ring_count = 0;

    for (size_t section_index = 0; section_index < layout->section_count; ++section_index) {
        const led_layout_section_t *section = &layout->sections[section_index];
        show_section_id_t section_id = SHOW_SECTION_INVALID;
        (void)show_section_id_from_name(section->name, &section_id);
        for (size_t offset = 0; offset < section->length; ++offset) {
            size_t led = section->logical_start + offset;
            if (led >= renderer->led_count) break;
            led_point_t point = {0};
            if (!led_layout_get_from_config(layout, (int)led, &point)) continue;
            renderer->section_ids[led] = section_id;
            renderer->section_u[led] = section->length > 1
                ? (float)offset / (float)(section->length - 1u) : 0.5f;
            led_point_t flat=show_geometry_noise_plane(section_id,point,renderer->section_u[led]);
            renderer->noise_xy[led][0]=flat.x; renderer->noise_xy[led][1]=flat.y;
            renderer->noise_min_x=have_point ? fminf(renderer->noise_min_x,flat.x) : flat.x;
            renderer->noise_min_y=have_point ? fminf(renderer->noise_min_y,flat.y) : flat.y;
            renderer->noise_max_y=have_point ? fmaxf(renderer->noise_max_y,flat.y) : flat.y;
            point = show_geometry_pose(section_id, point, renderer->section_u[led]);
            renderer->points[led] = point;
            renderer->radial[led] = sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
            if (!have_point) {
                renderer->min_x = renderer->max_x = point.x;
                renderer->min_y = renderer->max_y = point.y;
                renderer->min_z = renderer->max_z = point.z;
            } else {
                renderer->min_x = fminf(renderer->min_x, point.x);
                renderer->max_x = fmaxf(renderer->max_x, point.x);
                renderer->min_y = fminf(renderer->min_y, point.y);
                renderer->max_y = fmaxf(renderer->max_y, point.y);
                renderer->min_z = fminf(renderer->min_z, point.z);
                renderer->max_z = fmaxf(renderer->max_z, point.z);
            }
            have_point = true;
            if (section_id == SHOW_SECTION_FRONT_LEFT_RING) {
                ring_sum.x += point.x;
                ring_sum.y += point.y;
                ring_sum.z += point.z;
                ring_count++;
            }
        }
    }
    heap_caps_free(layout);
    if (!have_point) return ESP_ERR_INVALID_STATE;

    renderer->role_spacing_x = fmaxf(1.0f, (renderer->max_x - renderer->min_x) * 1.5f);
    renderer->local_ring_center = ring_count > 0 ? (led_point_t){
        ring_sum.x / (float)ring_count, ring_sum.y / (float)ring_count, ring_sum.z / (float)ring_count
    } : (led_point_t){0};
    set_formation(renderer, program->info.formation);
    return ESP_OK;
}

esp_err_t show_renderer_render(show_renderer_t *renderer, uint32_t playhead_ms,
                               uint8_t brightness, uint8_t *frame,
                               size_t frame_pixels, size_t *out_pixels)
{
    if (!renderer || !renderer->program || !frame ||
        frame_pixels < renderer->led_count) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t formation = renderer->program->info.formation;
    /* Sorted cue table; also works after backward seeks, never resets clip time. */
    for (uint32_t i = 0; i < renderer->program->formation_cue_count; ++i) {
        const show_formation_cue_v1_3_t *cue = &renderer->program->formation_cues[i];
        if (cue->time_ms > playhead_ms) break;
        formation = cue->formation;
    }
    if (formation != renderer->formation) set_formation(renderer, formation);
    memset(renderer->colors, 0, renderer->led_count * sizeof(renderer->colors[0]));

    uint32_t first_clip = 0;
    uint32_t clip_limit = renderer->program->role.clip_count;
    if (renderer->program->buckets && renderer->program->role.bucket_count > 0 &&
        renderer->program->info.bucket_ms > 0) {
        uint32_t bucket_index = playhead_ms / renderer->program->info.bucket_ms;
        if (bucket_index >= renderer->program->role.bucket_count) {
            bucket_index = renderer->program->role.bucket_count - 1u;
        }
        const show_bucket_v1_t *bucket = &renderer->program->buckets[bucket_index];
        first_clip = bucket->clip_first;
        clip_limit = first_clip + bucket->clip_count;
        if (clip_limit > renderer->program->role.clip_count) {
            clip_limit = renderer->program->role.clip_count;
        }
    }

    uint32_t active[SHOW_RENDER_MAX_ACTIVE];
    size_t active_count = 0;
    for (uint32_t i = first_clip; i < clip_limit; ++i) {
        const show_clip_v1_t *clip = &renderer->program->clips[i];
        if (clip->start_ms > playhead_ms) break;
        if (clip->start_ms <= playhead_ms && playhead_ms < clip->end_ms) {
            if (active_count >= SHOW_RENDER_MAX_ACTIVE) return ESP_ERR_INVALID_SIZE;
            size_t pos = active_count++;
            while (pos > 0) {
                const show_clip_v1_t *prev = &renderer->program->clips[active[pos - 1u]];
                if (prev->layer < clip->layer ||
                    (prev->layer == clip->layer && prev->start_ms <= clip->start_ms)) {
                    break;
                }
                active[pos] = active[pos - 1u];
                pos--;
            }
            active[pos] = i;
        }
    }

    for (size_t i = 0; i < active_count; ++i) {
        const show_clip_v1_t *clip = &renderer->program->clips[active[i]];
        if (clip->effect_kind == SHOW_EFFECT_FILL) {
            const show_param_fill_v1_t *param = show_package_program_param(
                renderer->program, clip, sizeof(*param));
            if (!param) continue;
            float values[SHOW_ANIMATION_KEYS] = {param_u16(param->intensity_1024)};
            values[9]=1;
            size_t base_bytes=sizeof(*param);
            bool local=true;
            for(unsigned a=0;a<3;++a) values[11+a*2]=1;
            if (renderer->program->info.version_minor>=10) {
                show_fill_bounds_t bounds;
                memcpy(&bounds,(const uint8_t *)param+base_bytes,sizeof(bounds));
                base_bytes+=sizeof(bounds); local=bounds.space!=0; values[2]=bounds.softness;
                for(unsigned i=0;i<6;++i) values[10+i]=bounds.limits[i];
            }
            const show_spatial_color_t *gradient=NULL;
            base_bytes=apply_animations(renderer,clip,param,base_bytes,playhead_ms,values,NULL,&gradient);
            bool bounded=false;
            for(unsigned a=0;a<3;++a) bounded |= values[10+a*2]!=0 || values[11+a*2]!=1;
            show_color_grid_t grid;
            show_color_t color = clip_color_at(renderer, clip, param, base_bytes,
                                               param->r, param->g, param->b, param->a,
                                               playhead_ms,&grid);
            float scale = values[0] * values[9] *
                          clip_envelope(clip, playhead_ms);
            if (bounded || (gradient && gradient->enabled)) {
                for (size_t led=0;led<renderer->led_count;++led) {
                    if (!clip_targets_led(renderer,clip,led)) continue;
                    float core=renderer->program->info.version_minor>=9 ? fabsf(renderer->section_u[led]-.5f)*2 : 0;
                    float amount=scale*(bounded ? fill_mask(renderer,led,local,values,&core) : 1);
                    show_color_t shaded=spatial_color_at(renderer,gradient,color,led,core,&grid);
                    blend_pixel(&renderer->colors[led],clip->blend_mode,shaded.r*amount,shaded.g*amount,shaded.b*amount,shaded.a*amount);
                }
            } else render_uniform(renderer, clip,color.r,color.g,color.b,color.a,scale);
        } else {
            const show_param_spatial_v1_t *param = show_package_program_param(
                renderer->program, clip, sizeof(*param));
            if (param) render_spatial(renderer, clip, param, playhead_ms);
        }
    }

    pack_colors(renderer->colors, renderer->led_count, brightness, frame);
    if (out_pixels) *out_pixels = renderer->led_count;
    return ESP_OK;
}
