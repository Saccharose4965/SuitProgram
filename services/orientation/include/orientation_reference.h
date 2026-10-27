#pragma once
#include "orientation.h"
#include <stdbool.h>
#include <math.h>

typedef struct {
    imu_quat_t heading;
    bool calibrated;
} orientation_reference_t;

static inline void orientation_reference_reset(orientation_reference_t *reference)
{
    if (!reference) return;
    reference->heading = (imu_quat_t){1, 0, 0, 0};
    reference->calibrated = false;
}

// Capture heading from the horizontal finger axis while gravity retains tilt.
static inline bool orientation_reference_calibrate(orientation_reference_t *reference,
                                                   imu_quat_t q)
{
    if (!reference) return false;
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (!isfinite(n) || n < 1e-6f) return false;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n;
    // Raw +Y (finger axis), projected onto the gravity-level plane.
    float x = 2*(q.x*q.y - q.w*q.z);
    float y = 1 - 2*(q.x*q.x + q.z*q.z);
    if (hypotf(x,y) < .2f) return false;
    float half = .5f*(atan2f(y,x) - 1.5707963268f);
    reference->heading = (imu_quat_t){cosf(half),0,0,sinf(half)};
    reference->calibrated = true;
    return true;
}

static inline bool orientation_reference_relative(const orientation_reference_t *reference,
                                                  imu_quat_t sensor,
                                                  imu_quat_t *relative)
{
    if (!reference || !reference->calibrated || !relative) return false;
    float n = sqrtf(sensor.w*sensor.w + sensor.x*sensor.x +
                    sensor.y*sensor.y + sensor.z*sensor.z);
    if (!isfinite(n) || n < 1e-6f) return false;
    sensor.w/=n; sensor.x/=n; sensor.y/=n; sensor.z/=n;
    imu_quat_t z = reference->heading;
    *relative = (imu_quat_t){
        z.w*sensor.w + z.x*sensor.x + z.y*sensor.y + z.z*sensor.z,
        z.w*sensor.x - z.x*sensor.w - z.y*sensor.z + z.z*sensor.y,
        z.w*sensor.y + z.x*sensor.z - z.y*sensor.w - z.z*sensor.x,
        z.w*sensor.z - z.x*sensor.y + z.y*sensor.x - z.z*sensor.w,
    };
    return true;
}
