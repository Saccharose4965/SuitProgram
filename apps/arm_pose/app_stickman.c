#include "app_shell.h"
#include "ui_draw.h"
#include "audio_policy.h"

#include "orientation_service.h"
#include "arm_pose.h"
#include "esp_timer.h"
#include "oled.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef arm_vec3_t vec3;

typedef struct {
    vec3 a;
    vec3 b;
} segment3_t;

static bool s_track_left_arm = true;
static float s_view_yaw = 0.0f;
static arm_pose_t s_pose;

static bool pose_sample(imu_quat_t *q)
{
    uint64_t stamp;
    return orientation_service_snapshot(q, &stamp) &&
           (uint64_t)esp_timer_get_time() - stamp <= 250000;
}

const shell_legend_t STICKMAN_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_RIGHT, SHELL_ICON_SELECT, SHELL_ICON_REFRESH },
};

static inline vec3 v3(float x, float y, float z)
{
    return (vec3){ x, y, z };
}

static inline vec3 v3_add(vec3 a, vec3 b)
{
    return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline vec3 v3_sub(vec3 a, vec3 b)
{
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline vec3 v3_scale(vec3 a, float s)
{
    return v3(a.x * s, a.y * s, a.z * s);
}

static inline void fb_pset(uint8_t *fb, int x, int y, int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    if (!fb) return;
    if (x < clip_x0 || y < clip_y0 || x >= clip_x1 || y >= clip_y1) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void draw_line(uint8_t *fb, int x0, int y0, int x1, int y1,
                      int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        fb_pset(fb, x0, y0, clip_x0, clip_y0, clip_x1, clip_y1);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static vec3 rotate_y(vec3 p, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    return v3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z);
}

static vec3 rotate_x(vec3 p, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    return v3(p.x, c * p.y - s * p.z, s * p.y + c * p.z);
}

static vec3 apply_view_yaw(vec3 p)
{
    return rotate_y(p, s_view_yaw);
}

static vec3 camera_transform(vec3 p)
{
    p = rotate_y(p, -0.68f);
    p = rotate_x(p, 0.44f);
    p.z += 140.0f;
    return p;
}

static bool project_point(vec3 p, int cx, int cy, float focal, int *ox, int *oy, float *depth_out)
{
    vec3 cam = camera_transform(apply_view_yaw(p));
    if (cam.z <= 8.0f) return false;
    float invz = 1.0f / cam.z;
    if (ox) *ox = (int)lroundf((float)cx + focal * cam.x * invz);
    if (oy) *oy = (int)lroundf((float)cy - focal * cam.y * invz);
    if (depth_out) *depth_out = cam.z;
    return true;
}

static void draw_head(uint8_t *fb, int cx, int cy, float focal, vec3 center, float radius,
                      int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    const int steps = 12;
    int prev_x = 0;
    int prev_y = 0;
    bool have_prev = false;

    for (int i = 0; i <= steps; ++i) {
        float angle = (2.0f * (float)M_PI * (float)i) / (float)steps;
        vec3 p = v3(center.x + cosf(angle) * radius, center.y + sinf(angle) * radius, center.z);
        int sx = 0;
        int sy = 0;
        if (!project_point(p, cx, cy, focal, &sx, &sy, NULL)) {
            have_prev = false;
            continue;
        }
        if (have_prev) {
            draw_line(fb, prev_x, prev_y, sx, sy, clip_x0, clip_y0, clip_x1, clip_y1);
        }
        prev_x = sx;
        prev_y = sy;
        have_prev = true;
    }
}

static void sort_segments_by_depth(const segment3_t *src, size_t count, size_t *order)
{
    for (size_t i = 0; i < count; ++i) {
        order[i] = i;
    }

    for (size_t i = 1; i < count; ++i) {
        size_t idx = order[i];
        float depth = camera_transform(apply_view_yaw(v3_scale(v3_add(src[idx].a, src[idx].b), 0.5f))).z;
        size_t j = i;
        while (j > 0) {
            size_t prev = order[j - 1];
            float prev_depth = camera_transform(apply_view_yaw(v3_scale(v3_add(src[prev].a, src[prev].b), 0.5f))).z;
            if (prev_depth <= depth) break;
            order[j] = order[j - 1];
            --j;
        }
        order[j] = idx;
    }
}

void stickman_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_pose = (arm_pose_t){0};
    s_view_yaw = 0.0f;
    audio_policy_fft_set_armpose_app(true);
}

void stickman_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    audio_policy_fft_set_armpose_app(false);
}


void stickman_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        s_view_yaw -= 0.20f;
        if (s_view_yaw < -(float)M_PI) s_view_yaw += 2.0f * (float)M_PI;
    } else if (ev->button == INPUT_BTN_B) {
        s_view_yaw += 0.20f;
        if (s_view_yaw > (float)M_PI) s_view_yaw -= 2.0f * (float)M_PI;
    } else if (ev->button == INPUT_BTN_C) {
        s_track_left_arm = !s_track_left_arm;
        s_pose.calibrated = false;
    } else if (ev->button == INPUT_BTN_D) {
        imu_quat_t q;
        if (pose_sample(&q)) arm_pose_calibrate(&s_pose, q);
    }
}

void stickman_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb || w <= 0 || h <= 0) return;

    const int cx = x + w / 2;
    const int cy = y + h / 2 + 8;
    const float focal = 120.0f;
    const int clip_x1 = x + w;
    const int clip_y1 = y + h;

    imu_quat_t q;
    arm_vec3_t aim, span;
    if (!pose_sample(&q)) return;
    if (!s_pose.calibrated) {
        oled_draw_text3x5(fb, x+4, y+4, "Fold arm; press D");
        return;
    }
    if (!arm_pose_direction(&s_pose,q,s_track_left_arm,&aim) ||
        !arm_pose_vector(&s_pose,q,s_track_left_arm,(arm_vec3_t){1,0,0},&span)) return;

    const vec3 neck = v3(0.0f, 18.0f, 0.0f);
    const vec3 pelvis = v3(0.0f, -2.0f, 0.0f);
    const vec3 shoulder_l = v3(-12.0f, 16.0f, 0.0f);
    const vec3 shoulder_r = v3(12.0f, 16.0f, 0.0f);
    const vec3 hip_l = v3(-7.0f, -2.0f, 0.0f);
    const vec3 hip_r = v3(7.0f, -2.0f, 0.0f);
    const vec3 knee_l = v3(-10.0f, -22.0f, 1.0f);
    const vec3 knee_r = v3(10.0f, -22.0f, -1.0f);
    const vec3 foot_l = v3(-12.0f, -40.0f, 5.0f);
    const vec3 foot_r = v3(12.0f, -40.0f, 1.0f);
    const vec3 head_center = v3(0.0f, 31.0f, 0.0f);

    const vec3 active_shoulder = s_track_left_arm ? shoulder_l : shoulder_r;
    const vec3 static_shoulder = s_track_left_arm ? shoulder_r : shoulder_l;
    const float upper_len = 14.0f;
    const float fore_len = 16.0f;
    const float hand_len = 3.5f;

    const vec3 forearm_dir = v3(aim.x, aim.y, aim.z);
    const vec3 upper_dir = arm_pose_estimate_upper_arm(s_track_left_arm, forearm_dir);
    const vec3 elbow = v3_add(active_shoulder, v3_scale(upper_dir, upper_len));
    const vec3 wrist = v3_add(elbow, v3_scale(forearm_dir, fore_len));
    const vec3 hand_center = v3_add(wrist, v3_scale(forearm_dir, hand_len));
    const vec3 hand_span = v3_scale(v3(span.x,span.y,span.z),4.0f);
    const vec3 hand_a = v3_sub(hand_center, hand_span);
    const vec3 hand_b = v3_add(hand_center, hand_span);

    const vec3 static_elbow = v3_add(static_shoulder, v3(0.0f, -14.0f, 0.0f));
    const vec3 static_wrist = v3_add(static_elbow, v3(0.0f, -14.0f, 0.0f));

    const segment3_t segments[] = {
        { shoulder_l, neck },
        { neck, shoulder_r },
        { neck, pelvis },
        { pelvis, hip_l },
        { pelvis, hip_r },
        { hip_l, knee_l },
        { knee_l, foot_l },
        { hip_r, knee_r },
        { knee_r, foot_r },
        { static_shoulder, static_elbow },
        { static_elbow, static_wrist },
        { active_shoulder, elbow },
        { elbow, wrist },
        { hand_a, hand_b },
    };

    size_t order[sizeof(segments) / sizeof(segments[0])];
    sort_segments_by_depth(segments, sizeof(segments) / sizeof(segments[0]), order);

    draw_head(fb, cx, cy, focal, head_center, 6.0f, x, y, clip_x1, clip_y1);

    for (size_t i = 0; i < sizeof(order) / sizeof(order[0]); ++i) {
        const segment3_t *seg = &segments[order[i]];
        int x0 = 0;
        int y0 = 0;
        int x1 = 0;
        int y1 = 0;
        if (!project_point(seg->a, cx, cy, focal, &x0, &y0, NULL)) continue;
        if (!project_point(seg->b, cx, cy, focal, &x1, &y1, NULL)) continue;
        draw_line(fb, x0, y0, x1, y1, x, y, clip_x1, clip_y1);
    }
}
