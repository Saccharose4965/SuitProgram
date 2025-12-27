// threedee.c
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"

#include "threedee.h"
#include "hw.h"
#include "oled.h"
#include "orientation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float w, x, y, z; } quat;
typedef struct { float x, y, z; } vec3;

static const char *TAG = "threedee";
static imu_orientation_t *s_ori = NULL;
// Sensor is rotated 90 deg clockwise around Z relative to the screen.
// Apply a 90 deg CCW correction (about +Z) to align to the screen frame.
static quat s_mount_ds = {0.70710678f, 0.0f, 0.0f, 0.70710678f};

/* =========================
 *  Frame / intent summary
 * =========================
 *
 * World frame W for rendering is chosen coincident with body frame at start.
 * IMU filter yields q_sensor->world; per-MPU mounting correction is applied
 * via s_mount_ds before rendering.
 */

/* =========================
 *  Tiny 3D + quaternion math
 * ========================= */

static inline vec3 v3(float x, float y, float z) { return (vec3){x,y,z}; }

static inline vec3 v3_add(vec3 a, vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline vec3 v3_sub(vec3 a, vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline vec3 v3_scale(vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }

static inline float v3_dot(vec3 a, vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline vec3 v3_cross(vec3 a, vec3 b) {
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline float v3_norm(vec3 a) { return sqrtf(v3_dot(a,a)); }
static inline vec3 v3_normed(vec3 a) {
    float n = v3_norm(a);
    if (n < 1e-6f) return v3(0,0,0);
    return v3_scale(a, 1.0f/n);
}

static inline quat q_ident(void) { return (quat){1,0,0,0}; }

static inline quat q_conj(quat q) { return (quat){q.w, -q.x, -q.y, -q.z}; }

static inline quat q_mul(quat a, quat b) {
    // Hamilton product
    return (quat){
        .w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        .x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        .y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        .z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static inline quat q_normed(quat q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-9f) return q_ident();
    float inv = 1.0f/n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    return q;
}

// Rotate an orientation quaternion by a fixed frame-change q_DS (sensor->device)
static inline quat apply_mount(quat q_sw, quat q_ds){
    quat q_sd = q_conj(q_ds);
    // q_dw = q_ds * q_sw * q_sd
    return q_normed(q_mul(q_mul(q_ds, q_sw), q_sd));
}

static inline void q_to_mat3(quat q, float R[9]) {
    // R maps body->world.
    q = q_normed(q);

    float ww=q.w*q.w, xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    float wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;

    R[0] = ww+xx-yy-zz;
    R[1] = 2.0f*(xy - wz);
    R[2] = 2.0f*(xz + wy);

    R[3] = 2.0f*(xy + wz);
    R[4] = ww-xx+yy-zz;
    R[5] = 2.0f*(yz - wx);

    R[6] = 2.0f*(xz - wy);
    R[7] = 2.0f*(yz + wx);
    R[8] = ww-xx-yy+zz;
}

static inline vec3 mat3_mul_vec(const float m[9], vec3 v) {
    return v3(
        m[0]*v.x + m[1]*v.y + m[2]*v.z,
        m[3]*v.x + m[4]*v.y + m[5]*v.z,
        m[6]*v.x + m[7]*v.y + m[8]*v.z
    );
}

/* =========================
 *  OLED framebuffer helpers
 * ========================= */

static inline void fb_pset_local(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void draw_line(uint8_t *fb, int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1) {
        fb_pset_local(fb, x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static threedee_request_switch_fn s_switch_cb = NULL;
static void *s_switch_user = NULL;
static inline quat imu_quat_to_local(imu_quat_t q)
{
    return (quat){ q.w, q.x, q.y, q.z };
}

/* =========================
 *  Projection
 * ========================= */

static bool project_point(vec3 p_cam, int cx, int cy, float focal, int *ox, int *oy)
{
    const float near_z = 20.0f;
    if (p_cam.z <= near_z) return false;

    float invz = 1.0f / p_cam.z;
    float u = (float)cx + focal * p_cam.x * invz;
    float v = (float)cy - focal * p_cam.y * invz;

    if (ox) *ox = (int)lroundf(u);
    if (oy) *oy = (int)lroundf(v);
    return true;
}

/* =========================
 *  Public app interface
 * ========================= */

void threedee_set_request_switch(threedee_request_switch_fn fn, void *user_data)
{
    s_switch_cb = fn;
    s_switch_user = user_data;
}

void threedee_set_orientation_ctx(imu_orientation_t *ctx)
{
    s_ori = ctx;
}

void threedee_set_mount_quat(imu_quat_t q_ds)
{
    s_mount_ds = (quat){ q_ds.w, q_ds.x, q_ds.y, q_ds.z };
    s_mount_ds = q_normed(s_mount_ds);
}

void threedee_app_init(void)
{
}

void threedee_app_deinit(void)
{
}

void threedee_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;

    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        if (s_switch_cb) s_switch_cb("menu", s_switch_user);
        return;
    }

    // Recenter (define current pose as zero)
    if (ev->type == INPUT_EVENT_PRESS && ev->button == INPUT_BTN_D) {
        if (s_ori) imu_orientation_recenter(s_ori);
    }
}

void threedee_app_tick(float dt_sec)
{
    if (dt_sec < 0) dt_sec = 0;
}

void threedee_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    const int cx = x + w/2;
    const int cy = y + h/2;

    // Focal length tuned for tiny OLED; adjust if you want more/less FOV.
    const float focal = 100.0f;

    // Draw reticle
    fb_pset_local(fb, cx, cy);
    fb_pset_local(fb, cx+1, cy);
    fb_pset_local(fb, cx-1, cy);
    fb_pset_local(fb, cx, cy+1);
    fb_pset_local(fb, cx, cy-1);

    if (!s_ori || !imu_orientation_ready(s_ori)) {
        oled_draw_text3x5(fb, x+2, y+2, "Hold still...");
        return;
    }
    oled_draw_text3x5(fb, x+2, y+2, "D=recenter");

    // Camera rotation: q_sensor->world from filter, then apply mounting to device frame.
    imu_quat_t qf = imu_orientation_get(s_ori); // sensor -> world
    quat q_rel = imu_quat_to_local(qf);
    quat q_dw = apply_mount(q_rel, s_mount_ds); // device -> world
    // World->camera (view) uses the inverse rotation.
    quat q_view = q_conj(q_dw);
    float R_wc[9];
    float R_cw[9];
    q_to_mat3(q_view, R_wc); // world->camera
    // camera->world (transpose)
    R_cw[0]=R_wc[0]; R_cw[1]=R_wc[3]; R_cw[2]=R_wc[6];
    R_cw[3]=R_wc[1]; R_cw[4]=R_wc[4]; R_cw[5]=R_wc[7];
    R_cw[6]=R_wc[2]; R_cw[7]=R_wc[5]; R_cw[8]=R_wc[8];

    // World-anchored cubes
    static const vec3 cube_centers[] = {
        { -50, -30, -40 }, { 60, -40, -30 }, { -30, 70, -20 }, { 80, 50, -10 },
        { -80, 20, 10 }, { 40, 100, 20 }, { -10, -90, 30 }, { 90, -70, 40 },
    };
    const float half = 15.0f; // 30x30x30 cubes
    static const uint8_t edges[][2] = {
        {0,1},{0,2},{0,4},
        {1,3},{1,5},
        {2,3},{2,6},
        {3,7},
        {4,5},{4,6},
        {5,7},
        {6,7},
    };

    for (size_t c = 0; c < sizeof(cube_centers)/sizeof(cube_centers[0]); ++c) {
        vec3 vertsW[8];
        int idx = 0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    float lx = half * (float)sx;
                    float ly = half * (float)sy;
                    float lz = half * (float)sz;
                    vertsW[idx++] = v3_add(cube_centers[c], v3(lx, ly, lz));
                }
            }
        }

        int pxv[8], pyv[8];
        bool okv[8];
        for (int i = 0; i < 8; ++i) {
            vec3 pC = mat3_mul_vec(R_wc, vertsW[i]);
            okv[i] = project_point(pC, cx, cy, focal, &pxv[i], &pyv[i]);
        }
        for (size_t e = 0; e < sizeof(edges)/sizeof(edges[0]); ++e) {
            uint8_t a = edges[e][0], b = edges[e][1];
            if (!okv[a] || !okv[b]) continue;
            draw_line(fb, pxv[a], pyv[a], pxv[b], pyv[b]);
        }
    }

    // World-fixed axes gizmo placed in front of the camera (origin moves with camera, axes stay earth-fixed)
    const float axis_len = 60.0f;
    const float gizmo_dist = 200.0f;

    // Camera forward in world (third column of R_cw)
    vec3 forward_w = v3(R_cw[2], R_cw[5], R_cw[8]);
    vec3 origin_w = v3_scale(forward_w, gizmo_dist);

    vec3 ends_w[3] = {
        v3_add(origin_w, v3(axis_len, 0, 0)),   // world +X
        v3_add(origin_w, v3(0, axis_len, 0)),   // world +Y
        v3_add(origin_w, v3(0, 0, axis_len)),   // world +Z
    };

    vec3 origin_c = mat3_mul_vec(R_wc, origin_w);
    int ox, oy;
    if (!project_point(origin_c, cx, cy, focal, &ox, &oy)) return;

    int px[3], py[3];
    bool ok[3];
    for (int i = 0; i < 3; ++i) {
        vec3 pC = mat3_mul_vec(R_wc, ends_w[i]);
        ok[i] = project_point(pC, cx, cy, focal, &px[i], &py[i]);
    }

    if (ok[0]) draw_line(fb, ox, oy, px[0], py[0]); // X axis
    if (ok[1]) draw_line(fb, ox, oy, px[1], py[1]); // Y axis
    if (ok[2]) draw_line(fb, ox, oy, px[2], py[2]); // Z axis
}
