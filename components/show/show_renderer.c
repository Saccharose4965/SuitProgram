#include "show_renderer.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"

#include "led.h"
#include "led_layout.h"
#include "show_layout_ids.h"

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
    size_t led_count;
    show_section_id_t section_ids[LED_LAYOUT_MAX_PIXELS];
    led_point_t points[LED_LAYOUT_MAX_PIXELS];
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
    led_point_t global_center;
    led_point_t ring_center;
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
    return renderer->points[led].x +
           ((float)renderer->role_id - 1.0f) * renderer->role_spacing_x;
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
    const led_point_t *point = &renderer->points[led];
    switch (axis) {
        case SHOW_AXIS_X: return x;
        case SHOW_AXIS_Z: return point->z;
        case SHOW_AXIS_SECTION_U: return renderer->section_u[led];
        case SHOW_AXIS_RADIAL:
            return sqrtf(x * x + point->y * point->y + point->z * point->z);
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
    float min_x = renderer->min_x - renderer->role_spacing_x;
    float max_x = renderer->max_x + renderer->role_spacing_x;
    switch (axis) {
        case SHOW_AXIS_X: *lo = min_x; *hi = max_x; return;
        case SHOW_AXIS_Y: *lo = renderer->min_y; *hi = renderer->max_y; return;
        case SHOW_AXIS_Z: *lo = renderer->min_z; *hi = renderer->max_z; return;
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
            float dz = fmaxf(fabsf(renderer->min_z), fabsf(renderer->max_z));
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
    if ((clip->flags & SHOW_CLIP_FLAG_TEMPO_SYNC) &&
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
                                  uint32_t playhead_ms)
{
    show_color_t fallback = {
        .r = (float)r / 255.0f,
        .g = (float)g / 255.0f,
        .b = (float)b / 255.0f,
        .a = (float)a / 255.0f,
    };
    if (!base_param || clip->param_bytes < base_bytes + sizeof(show_color_anim_v1_1_t)) {
        return fallback;
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
    for (size_t led = 0; led < renderer->led_count; ++led) {
        if (!clip_targets_led(renderer, clip, led)) continue;
        blend_pixel(&renderer->colors[led], clip->blend_mode,
                    r * scale, g * scale, b * scale, a * scale);
    }
}

static void render_spatial(show_renderer_t *renderer, const show_clip_v1_t *clip,
                           const show_param_spatial_v1_t *param,
                           uint32_t playhead_ms)
{
    float duration = (float)(clip->end_ms - clip->start_ms);
    float clip_t = clamp01((float)(playhead_ms - clip->start_ms) / fmaxf(duration, 1.0f));
    show_color_t color = clip_color_at(renderer, clip, param, sizeof(*param),
                                       param->r, param->g, param->b, param->a,
                                       playhead_ms);
    float r = color.r;
    float g = color.g;
    float b = color.b;
    float a = color.a;
    float intensity = param_u16(param->intensity_1024) * clip_envelope(clip, playhead_ms);
    float width = fmaxf(0.02f, param_u16(param->width_1024));
    float softness = fmaxf(0.001f, param_u16(param->softness_1024));
    float duty = clamp01(param_u16(param->duty_cycle_1024));
    int32_t cycle_index = 0;
    float phase = clip_phase(renderer, clip, param, playhead_ms, &cycle_index);

    if (clip->effect_kind == SHOW_EFFECT_BLINK) {
        float on = phase < duty ? 1.0f : 0.0f;
        if (!on && softness > 0.0f && phase < fminf(1.0f, duty + softness)) {
            on = clamp01(1.0f - (phase - duty) / softness);
        }
        render_uniform(renderer, clip, r, g, b, a, intensity * on);
        return;
    }
    if (clip->effect_kind == SHOW_EFFECT_PULSE) {
        float wave = 0.5f - 0.5f * cosf((float)(2.0 * M_PI) * phase);
        float amount = lerpf(param_u16(param->min_intensity_1024),
                             param_u16(param->max_intensity_1024), wave);
        render_uniform(renderer, clip, r, g, b, a, intensity * amount);
        return;
    }

    float axis_angle = ((float)hash_u32(clip->seed, 0, 0, cycle_index,
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
    float travel = (clip->flags & SHOW_CLIP_FLAG_TEMPO_SYNC) ? phase : clip_t;
    if (clip->effect_kind == SHOW_EFFECT_CHASE ||
        clip->effect_kind == SHOW_EFFECT_SPARKLE ||
        clip->effect_kind == SHOW_EFFECT_FANOUT ||
        clip->effect_kind == SHOW_EFFECT_RADIAL_RAY) {
        travel = phase;
    }
    if (param->reverse) travel = 1.0f - travel;

    float core_half = width * 0.5f;
    float margin = core_half + softness;
    float head = lerpf(-margin, 1.0f + margin, travel);
    uint16_t repeats = param->repeats ? param->repeats : 1u;

    for (size_t led = 0; led < renderer->led_count; ++led) {
        if (!clip_targets_led(renderer, clip, led)) continue;
        float falloff = 1.0f;

        if (clip->effect_kind == SHOW_EFFECT_SWEEP ||
            clip->effect_kind == SHOW_EFFECT_MIRROR_SWEEP) {
            float value = local_axis_value(renderer, led, param->axis_kind, axis_cos, axis_sin);
            float pos = param->axis_kind == SHOW_AXIS_SECTION_U
                ? renderer->section_u[led]
                : (local_inv == 0.0f ? 0.5f : (value - local_lo) * local_inv);
            float distance = fabsf(pos - head);
            if (clip->effect_kind == SHOW_EFFECT_MIRROR_SWEEP) {
                distance = fminf(distance, fabsf((1.0f - pos) - head));
            }
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_FANOUT) {
            float pos = fabsf(renderer->section_u[led] - 0.5f) * 2.0f;
            float distance = fabsf(pos - head);
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_CHASE) {
            float value = local_axis_value(renderer, led, param->axis_kind, axis_cos, axis_sin);
            float pos = param->axis_kind == SHOW_AXIS_SECTION_U
                ? renderer->section_u[led]
                : (local_inv == 0.0f ? 0.5f : (value - local_lo) * local_inv);
            float local = wrap01(pos * repeats + travel * repeats);
            float distance = fminf(local, 1.0f - local);
            falloff = clamp01(1.0f - distance / width);
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
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_RADIAL_RAY) {
            float dx = role_world_x(renderer, led) - renderer->ring_center.x;
            float dy = renderer->points[led].y - renderer->ring_center.y;
            float angle = wrap01(atan2f(dx, -dy) / (float)(2.0 * M_PI));
            float distance = fabsf(wrap01(angle - travel + 0.5f) - 0.5f);
            falloff = distance <= core_half
                ? 1.0f : clamp01(1.0f - (distance - core_half) / softness);
        } else if (clip->effect_kind == SHOW_EFFECT_TRAVELING_ORB) {
            float span = fmaxf(1.0f, global_hi - global_lo);
            float radius = span * width * 0.5f;
            float feather = span * softness * 0.5f;
            float orb_head = lerpf(global_lo - radius - feather * 0.5f,
                                    global_hi + radius + feather * 0.5f, travel);
            float dir_x = 1.0f, dir_y = 0.0f, dir_z = 0.0f;
            if (param->axis_kind == SHOW_AXIS_Y) {
                dir_x = 0.0f; dir_y = 1.0f;
            } else if (param->axis_kind == SHOW_AXIS_Z) {
                dir_x = 0.0f; dir_z = 1.0f;
            } else if (param->axis_kind == SHOW_AXIS_RANDOM_XY) {
                dir_x = axis_cos; dir_y = axis_sin;
            }
            float anchor_x = renderer->global_center.x;
            if (param->axis_mix_1024 & SHOW_SPATIAL_OPTION_RANDOM_CROSS_X) {
                float unit = (float)hash_u32(clip->seed, 0, 0, cycle_index,
                                             UINT32_C(0x54A9D93B)) / 4294967295.0f;
                anchor_x = lerpf(renderer->min_x - renderer->role_spacing_x,
                                 renderer->max_x + renderer->role_spacing_x, unit);
            }
            float center_proj = anchor_x * dir_x + renderer->global_center.y * dir_y +
                                renderer->global_center.z * dir_z;
            float orb_x = anchor_x + dir_x * (orb_head - center_proj);
            float orb_y = renderer->global_center.y + dir_y * (orb_head - center_proj);
            float orb_z = renderer->global_center.z + dir_z * (orb_head - center_proj);
            float dx = role_world_x(renderer, led) - orb_x;
            float dy = renderer->points[led].y - orb_y;
            float dz = renderer->points[led].z - orb_z;
            float distance = sqrtf(dx * dx + dy * dy + dz * dz);
            falloff = distance <= radius ? 1.0f
                : clamp01(1.0f - (distance - radius) / fmaxf(feather, 1e-6f));
        } else if (clip->effect_kind == SHOW_EFFECT_GROUND_ENERGY) {
            float edge = fmaxf(1e-6f, (renderer->max_y - renderer->min_y) * softness);
            float level = lerpf(renderer->max_y + edge, renderer->min_y, travel);
            float y = renderer->points[led].y;
            falloff = y >= level ? 1.0f : clamp01(1.0f - (level - y) / edge);
        }

        float scale = intensity * falloff;
        blend_pixel(&renderer->colors[led], clip->blend_mode,
                    r * scale, g * scale, b * scale, a * scale);
    }
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

esp_err_t show_renderer_configure(show_renderer_t *renderer,
                                  const show_package_program_t *program,
                                  uint8_t role_id)
{
    if (!renderer || !program || role_id >= SHOW_MAX_ROLES) return ESP_ERR_INVALID_ARG;
    led_layout_config_t layout;
    led_layout_snapshot(&layout);
    if (layout.total_leds == 0 || layout.total_leds > LED_LAYOUT_MAX_PIXELS) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(renderer, 0, sizeof(*renderer));
    renderer->program = program;
    renderer->role_id = role_id;
    renderer->led_count = layout.total_leds;
    bool have_point = false;
    led_point_t ring_sum = {0};
    size_t ring_count = 0;

    for (size_t section_index = 0; section_index < layout.section_count; ++section_index) {
        const led_layout_section_t *section = &layout.sections[section_index];
        show_section_id_t section_id = SHOW_SECTION_INVALID;
        (void)show_section_id_from_name(section->name, &section_id);
        for (size_t offset = 0; offset < section->length; ++offset) {
            size_t led = section->logical_start + offset;
            if (led >= renderer->led_count) break;
            led_point_t point = {0};
            if (!led_layout_get_from_config(&layout, (int)led, &point)) continue;
            renderer->section_ids[led] = section_id;
            renderer->points[led] = point;
            renderer->section_u[led] = section->length > 1
                ? (float)offset / (float)(section->length - 1u) : 0.5f;
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
    if (!have_point) return ESP_ERR_INVALID_STATE;

    renderer->role_spacing_x = fmaxf(1.0f, (renderer->max_x - renderer->min_x) * 1.5f);
    renderer->global_center = (led_point_t){
        .x = (renderer->min_x + renderer->max_x) * 0.5f,
        .y = (renderer->min_y + renderer->max_y) * 0.5f,
        .z = (renderer->min_z + renderer->max_z) * 0.5f,
    };
    renderer->ring_center = ring_count > 0 ? (led_point_t){
        .x = ring_sum.x / (float)ring_count,
        .y = ring_sum.y / (float)ring_count,
        .z = ring_sum.z / (float)ring_count,
    } : renderer->global_center;
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
        if (clip->effect_kind == SHOW_EFFECT_SOLID) {
            const show_param_solid_v1_t *param = show_package_program_param(
                renderer->program, clip, sizeof(*param));
            if (!param) continue;
            show_color_t color = clip_color_at(renderer, clip, param, sizeof(*param),
                                               param->r, param->g, param->b, param->a,
                                               playhead_ms);
            float scale = param_u16(param->intensity_1024) *
                          clip_envelope(clip, playhead_ms);
            render_uniform(renderer, clip,
                           color.r, color.g, color.b, color.a,
                           scale);
        } else {
            const show_param_spatial_v1_t *param = show_package_program_param(
                renderer->program, clip, sizeof(*param));
            if (param) render_spatial(renderer, clip, param, playhead_ms);
        }
    }

    float brightness_scale = (float)brightness / 255.0f;
    for (size_t led = 0; led < renderer->led_count; ++led) {
        show_color_t color = renderer->colors[led];
        led_set_pixel_rgb(frame, led,
                          (uint8_t)lroundf(clamp01(color.r) * 255.0f * brightness_scale),
                          (uint8_t)lroundf(clamp01(color.g) * 255.0f * brightness_scale),
                          (uint8_t)lroundf(clamp01(color.b) * 255.0f * brightness_scale));
    }
    if (out_pixels) *out_pixels = renderer->led_count;
    return ESP_OK;
}
