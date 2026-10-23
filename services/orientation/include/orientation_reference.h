#pragma once
#include "orientation.h"
#include <math.h>

// Shared service policy; each consumer owns its reference. Never absorb tilt.
static inline bool orientation_reference_calibrate(imu_quat_t *zero, imu_quat_t q)
{
    if (!zero) return false;
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (!isfinite(n) || n < 1e-6f) return false;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n;
    // Raw +Y (finger axis), projected onto the gravity-level plane.
    float x = 2*(q.x*q.y - q.w*q.z);
    float y = 1 - 2*(q.x*q.x + q.z*q.z);
    if (hypotf(x,y) < .2f) return false;
    float half = .5f*(atan2f(y,x) - 1.5707963268f);
    *zero = (imu_quat_t){cosf(half),0,0,sinf(half)};
    return true;
}
