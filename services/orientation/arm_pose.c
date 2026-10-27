#include "arm_pose.h"
#include <math.h>

static imu_quat_t mul(imu_quat_t a, imu_quat_t b)
{
    return (imu_quat_t){a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
static imu_quat_t conjq(imu_quat_t q) { return (imu_quat_t){q.w,-q.x,-q.y,-q.z}; }
bool arm_pose_calibrate(arm_pose_t *p, imu_quat_t q)
{
    return orientation_reference_calibrate(p,q);
}
bool arm_pose_calibrate_heading(arm_pose_t *p, imu_quat_t q)
{
    return arm_pose_calibrate(p,q);
}
bool arm_pose_vector(const arm_pose_t *p, imu_quat_t sensor, bool left,
                     arm_vec3_t local, arm_vec3_t *d)
{
    if (!p || !d) return false;
    imu_quat_t q;
    if (!orientation_reference_relative(p,sensor,&q)) return false;
    imu_quat_t v=mul(mul(q,(imu_quat_t){0,local.x,local.y,local.z}),conjq(q));
    // Folded reference: target -MPU X, up +MPU Z, lateral +MPU Y.
    // Right-arm selection mirrors lateral placement, not rotation direction.
    *d=(arm_vec3_t){left?v.y:-v.y,v.z,-v.x};
    return true;
}
bool arm_pose_direction(const arm_pose_t *p, imu_quat_t sensor, bool left, arm_vec3_t *d)
{
    return arm_pose_vector(p,sensor,left,(arm_vec3_t){0,1,0},d);
}
bool arm_pose_canvas(arm_vec3_t d, float *x, float *y)
{
    if (!x || !y || !isfinite(d.x) || !isfinite(d.y) || !isfinite(d.z) || d.z<.15f) return false;
    float u=1.5f*d.x/(d.z*.30f), v=-1.5f*d.y/(d.z*.75f);
    if (fabsf(u)>1 || fabsf(v)>1) return false;
    *x=u; *y=v;
    return true;
}
float arm_brush_distance(float x,float y,float ax,float ay,float bx,float by)
{
    float dx=bx-ax,dy=by-ay,n=dx*dx+dy*dy;
    float t=n>1e-9f?fmaxf(0,fminf(1,((x-ax)*dx+(y-ay)*dy)/n)):0;
    dx=x-ax-t*dx; dy=y-ay-t*dy;
    return sqrtf(dx*dx+dy*dy);
}

static inline arm_vec3_t pose_v3(float x, float y, float z)
{
    return (arm_vec3_t){ x, y, z };
}

static inline arm_vec3_t pose_v3_add(arm_vec3_t a, arm_vec3_t b)
{
    return pose_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline arm_vec3_t pose_v3_scale(arm_vec3_t a, float s)
{
    return pose_v3(a.x * s, a.y * s, a.z * s);
}

static inline float pose_v3_dot(arm_vec3_t a, arm_vec3_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float pose_v3_len(arm_vec3_t a)
{
    return sqrtf(pose_v3_dot(a, a));
}

static inline arm_vec3_t pose_v3_norm(arm_vec3_t a)
{
    float n = pose_v3_len(a);
    if (n < 1e-6f) return pose_v3(0.0f, -1.0f, 0.0f);
    return pose_v3_scale(a, 1.0f / n);
}


arm_vec3_t arm_pose_estimate_upper_arm(bool left_arm, arm_vec3_t forearm_dir)
{
    // The forearm IMU does not observe the shoulder joint, so infer a plausible
    // upper-arm link by blending a relaxed hanging pose with the measured
    // forearm direction and a small outward flare.
    arm_vec3_t rest_dir = pose_v3_norm(pose_v3(left_arm ? -0.58f : 0.58f, -0.80f, 0.14f));
    arm_vec3_t flare_dir = pose_v3_norm(pose_v3(left_arm ? -0.30f : 0.30f, -0.12f, 0.52f));
    arm_vec3_t follow_dir = pose_v3_norm(forearm_dir);
    return pose_v3_norm(pose_v3_add(pose_v3_scale(rest_dir, 0.78f),
                          pose_v3_add(pose_v3_scale(follow_dir, 0.34f),
                                 pose_v3_scale(flare_dir, 0.20f))));
}
