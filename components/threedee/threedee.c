// threedee.c
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "threedee.h"
#include "hw.h"
#include "oled.h"
#include "orientation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float w, x, y, z; } quat;
typedef struct { float x, y, z; } vec3;

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
static inline vec3 v3_hadamard(vec3 a, vec3 b) { return v3(a.x*b.x, a.y*b.y, a.z*b.z); }

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

static inline vec3 v3_rotate_axis(vec3 p, vec3 axis, float angle)
{
    axis = v3_normed(axis);
    float c = cosf(angle);
    float s = sinf(angle);
    return v3_add(
        v3_add(v3_scale(p, c), v3_scale(v3_cross(axis, p), s)),
        v3_scale(axis, v3_dot(axis, p) * (1.0f - c))
    );
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

static inline void fb_pset_local(uint8_t *fb, int x, int y,
                                 int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    // The shell owns the content viewport; PANEL_W/H are only the backing framebuffer stride.
    if (x < clip_x0 || y < clip_y0 || x >= clip_x1 || y >= clip_y1) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void draw_line(uint8_t *fb, int x0, int y0, int x1, int y1,
                      int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1) {
        fb_pset_local(fb, x0, y0, clip_x0, clip_y0, clip_x1, clip_y1);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static threedee_request_switch_fn s_switch_cb = NULL;
static void *s_switch_user = NULL;
static float s_scene_time = 0.0f;

typedef enum {
    SHAPE_CUBE = 0,
    SHAPE_OCTA,
    SHAPE_PYRAMID,
    SHAPE_PRISM,
} wire_shape_t;

typedef struct {
    wire_shape_t kind;
    vec3 center;
    vec3 scale;
    vec3 spin_axis;
    float spin_rate;
    float spin_phase;
} wire_instance_t;

typedef struct {
    const vec3 *verts;
    size_t vert_count;
    const uint8_t (*edges)[2];
    size_t edge_count;
} wire_shape_def_t;

static const vec3 kCubeVerts[] = {
    {-1,-1,-1}, { 1,-1,-1}, {-1, 1,-1}, { 1, 1,-1},
    {-1,-1, 1}, { 1,-1, 1}, {-1, 1, 1}, { 1, 1, 1},
};

static const uint8_t kCubeEdges[][2] = {
    {0,1},{0,2},{0,4},
    {1,3},{1,5},
    {2,3},{2,6},
    {3,7},
    {4,5},{4,6},
    {5,7},
    {6,7},
};

static const vec3 kOctaVerts[] = {
    { 1, 0, 0}, {-1, 0, 0},
    { 0, 1, 0}, { 0,-1, 0},
    { 0, 0, 1}, { 0, 0,-1},
};

static const uint8_t kOctaEdges[][2] = {
    {0,2},{0,3},{0,4},{0,5},
    {1,2},{1,3},{1,4},{1,5},
    {2,4},{2,5},{3,4},{3,5},
};

static const vec3 kPyramidVerts[] = {
    {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
    { 0, 0, 1},
};

static const uint8_t kPyramidEdges[][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {0,4},{1,4},{2,4},{3,4},
};

static const vec3 kPrismVerts[] = {
    {-1,-1,-1}, { 1,-1,-1}, { 0, 1,-1},
    {-1,-1, 1}, { 1,-1, 1}, { 0, 1, 1},
};

static const uint8_t kPrismEdges[][2] = {
    {0,1},{1,2},{2,0},
    {3,4},{4,5},{5,3},
    {0,3},{1,4},{2,5},
};

static const wire_shape_def_t kWireShapes[] = {
    { kCubeVerts,    sizeof(kCubeVerts) / sizeof(kCubeVerts[0]),       kCubeEdges,    sizeof(kCubeEdges) / sizeof(kCubeEdges[0]) },
    { kOctaVerts,    sizeof(kOctaVerts) / sizeof(kOctaVerts[0]),       kOctaEdges,    sizeof(kOctaEdges) / sizeof(kOctaEdges[0]) },
    { kPyramidVerts, sizeof(kPyramidVerts) / sizeof(kPyramidVerts[0]), kPyramidEdges, sizeof(kPyramidEdges) / sizeof(kPyramidEdges[0]) },
    { kPrismVerts,   sizeof(kPrismVerts) / sizeof(kPrismVerts[0]),     kPrismEdges,   sizeof(kPrismEdges) / sizeof(kPrismEdges[0]) },
};

static const wire_instance_t kSceneInstances[] = {
    { SHAPE_CUBE,    { -72, -38, -56 }, { 18, 18, 18 }, { 0.3f,  1.0f,  0.2f },  0.55f, 0.1f },
    { SHAPE_OCTA,    {  58, -52, -34 }, { 22, 22, 22 }, { 1.0f,  0.1f,  0.4f }, -0.70f, 0.8f },
    { SHAPE_PYRAMID, { -24,  76, -20 }, { 20, 20, 24 }, { 0.2f, -0.7f,  1.0f },  0.48f, 1.4f },
    { SHAPE_PRISM,   {  86,  48,  -6 }, { 24, 18, 20 }, { 0.8f,  0.5f,  0.2f }, -0.38f, 2.0f },
    { SHAPE_CUBE,    { -88,  18,  18 }, { 14, 24, 14 }, { 0.0f,  1.0f,  0.6f },  0.62f, 2.4f },
    { SHAPE_OCTA,    {  28, 106,  26 }, { 18, 26, 18 }, { 0.6f,  0.2f, -0.8f },  0.44f, 3.0f },
    { SHAPE_PRISM,   {  -8, -96,  36 }, { 20, 16, 24 }, { 0.4f, -1.0f,  0.1f }, -0.52f, 3.5f },
    { SHAPE_PYRAMID, {  96, -74,  46 }, { 16, 16, 22 }, { 1.0f,  0.3f,  0.7f },  0.58f, 4.1f },
};
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
    s_scene_time = 0.0f;
}

void threedee_app_deinit(void)
{
}

void threedee_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;

    // Recenter (define current pose as zero)
    if (ev->type == INPUT_EVENT_PRESS && ev->button == INPUT_BTN_D) {
        if (s_ori) imu_orientation_recenter(s_ori);
    }
}

void threedee_app_tick(float dt_sec)
{
    if (dt_sec < 0) dt_sec = 0;
    s_scene_time += dt_sec;
    if (s_scene_time > 1000.0f) {
        s_scene_time = fmodf(s_scene_time, 1000.0f);
    }
}

void threedee_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    const int cx = x + w/2;
    const int cy = y + h/2;
    const int clip_x1 = x + w;
    const int clip_y1 = y + h;

    // Focal length tuned for tiny OLED; adjust if you want more/less FOV.
    const float focal = 100.0f;

    // Camera rotation: q_sensor->world from filter, then apply mounting to device frame.
    quat q_rel = q_ident();
    if (s_ori && imu_orientation_ready(s_ori)) {
        imu_quat_t qf = imu_orientation_get(s_ori); // sensor -> world
        q_rel = imu_quat_to_local(qf);
    }
    quat q_dw = apply_mount(q_rel, s_mount_ds); // device -> world
    // World->camera (view) uses the inverse rotation.
    quat q_view = q_conj(q_dw);
    float R_wc[9];
    q_to_mat3(q_view, R_wc); // world->camera

    for (size_t s = 0; s < sizeof(kSceneInstances)/sizeof(kSceneInstances[0]); ++s) {
        const wire_instance_t *inst = &kSceneInstances[s];
        const wire_shape_def_t *shape = &kWireShapes[inst->kind];
        vec3 vertsW[8];
        int pxv[8], pyv[8];
        bool okv[8];
        float spin = inst->spin_phase + s_scene_time * inst->spin_rate;

        for (size_t i = 0; i < shape->vert_count; ++i) {
            vec3 local = v3_hadamard(shape->verts[i], inst->scale);
            local = v3_rotate_axis(local, inst->spin_axis, spin);
            vertsW[i] = v3_add(inst->center, local);
            vec3 pC = mat3_mul_vec(R_wc, vertsW[i]);
            okv[i] = project_point(pC, cx, cy, focal, &pxv[i], &pyv[i]);
        }

        for (size_t e = 0; e < shape->edge_count; ++e) {
            uint8_t a = shape->edges[e][0];
            uint8_t b = shape->edges[e][1];
            if (!okv[a] || !okv[b]) continue;
            draw_line(fb, pxv[a], pyv[a], pxv[b], pyv[b], x, y, clip_x1, clip_y1);
        }
    }
}
