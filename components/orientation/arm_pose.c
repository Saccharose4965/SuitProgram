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
static bool normalize(imu_quat_t *q)
{
    float n=sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
    if (!isfinite(n) || n<1e-6f) return false;
    q->w/=n; q->x/=n; q->y/=n; q->z/=n;
    return true;
}
bool arm_pose_calibrate(arm_pose_t *p, imu_quat_t q)
{
    if (!p || !normalize(&q)) return false;
    p->zero=q; p->calibrated=true;
    return true;
}
bool arm_pose_calibrate_heading(arm_pose_t *p, imu_quat_t q)
{
    if (!p || !normalize(&q)) return false;
    // Project the finger axis onto the gravity-level world XY plane. Local
    // Y roll cannot change this axis; neither roll nor pitch tilts the canvas.
    imu_quat_t beam=mul(mul(q,(imu_quat_t){0,0,1,0}),conjq(q));
    if (hypotf(beam.x,beam.y)<.2f) return false; // heading undefined near vertical
    float half=.5f*(atan2f(beam.y,beam.x)-1.5707963268f);
    p->zero=(imu_quat_t){cosf(half),0,0,sinf(half)};
    p->calibrated=true;
    return true;
}
bool arm_pose_vector(const arm_pose_t *p, imu_quat_t sensor, bool left,
                     arm_vec3_t local, arm_vec3_t *d)
{
    if (!p || !p->calibrated || !d || !normalize(&sensor)) return false;
    // Sensor-to-reference rotation, not the inverse used to move a 3D scene.
    imu_quat_t q=mul(conjq(p->zero),sensor);
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
