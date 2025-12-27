// Lightweight IMU orientation filter (accel + gyro → quaternion).
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float w, x, y, z; } imu_quat_t;

typedef struct {
    bool init;
    bool ready;
    bool have_sample;
    uint64_t t_prev_us;

    imu_quat_t q_BW;    // body -> world
    imu_quat_t q_zero;  // recenter reference
    float beta;
} imu_orientation_t;

void imu_orientation_init(imu_orientation_t *st, float beta);
bool imu_orientation_ready(const imu_orientation_t *st);
void imu_orientation_recenter(imu_orientation_t *st);
imu_quat_t imu_orientation_get(const imu_orientation_t *st);

void imu_orientation_update(
    imu_orientation_t *st,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps,
    uint64_t t_us
);

#ifdef __cplusplus
}
#endif
