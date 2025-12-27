#include "orientation.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Hard-coded gyro biases (deg/s) for x and y axes; tune as needed.
static const float GYRO_BIAS_X_DPS = -1.9f;
static const float GYRO_BIAS_Y_DPS = 0.5f;

/* =========================
 *  Math helpers
 * ========================= */

typedef struct { float x, y, z; } vec3;

static inline vec3 v3(float x,float y,float z){ return (vec3){x,y,z}; }
static inline vec3 v3_add(vec3 a, vec3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline vec3 v3_sub(vec3 a, vec3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline vec3 v3_scale(vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline float v3_dot(vec3 a, vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline vec3 v3_cross(vec3 a, vec3 b){
    return v3(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}
static inline float v3_norm(vec3 a){ return sqrtf(v3_dot(a,a)); }
static inline vec3 v3_normed(vec3 a){
    float n = v3_norm(a);
    if (n < 1e-6f) return v3(0,0,0);
    return v3_scale(a, 1.0f/n);
}

/* =========================
 *  Quaternion helpers
 * ========================= */

static inline imu_quat_t q_ident(void){ return (imu_quat_t){1,0,0,0}; }

static inline imu_quat_t q_conj(imu_quat_t q){
    return (imu_quat_t){ q.w, -q.x, -q.y, -q.z };
}

static inline imu_quat_t q_mul(imu_quat_t a, imu_quat_t b){
    return (imu_quat_t){
        .w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        .x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        .y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        .z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static inline imu_quat_t q_normed(imu_quat_t q){
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-9f) return q_ident();
    float inv = 1.0f/n;
    q.w*=inv; q.x*=inv; q.y*=inv; q.z*=inv;
    return q;
}

static inline vec3 q_rotate(imu_quat_t q, vec3 v){
    imu_quat_t p = {0, v.x, v.y, v.z};
    imu_quat_t r = q_mul(q_mul(q,p), q_conj(q));
    return v3(r.x,r.y,r.z);
}

static inline vec3 map_accel(float ax,float ay,float az){
    return v3(ax, ay, az);
}
static inline vec3 map_gyro(float gx,float gy,float gz){
    return v3(gx, gy, gz);
}

/* =========================
 *  Madgwick IMU update
 * ========================= */

static inline float inv_sqrtf_fast(float x){
    return 1.0f / sqrtf(x);
}

static void madgwick_step(float *q0, float *q1, float *q2, float *q3,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt, float beta)
{
    float _q0 = *q0, _q1 = *q1, _q2 = *q2, _q3 = *q3;

    // Rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float qDot1 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
    float qDot2 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
    float qDot3 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    float norm = ax*ax + ay*ay + az*az;
    if (norm > 1e-12f) {
        norm = inv_sqrtf_fast(norm);
        ax *= norm; ay *= norm; az *= norm;

        // Auxiliary variables
        float _2q0 = 2.0f * _q0;
        float _2q1 = 2.0f * _q1;
        float _2q2 = 2.0f * _q2;
        float _2q3 = 2.0f * _q3;

        float _4q0 = 4.0f * _q0;
        float _4q1 = 4.0f * _q1;
        float _4q2 = 4.0f * _q2;

        float _8q1 = 8.0f * _q1;
        float _8q2 = 8.0f * _q2;

        float q0q0 = _q0*_q0;
        float q1q1 = _q1*_q1;
        float q2q2 = _q2*_q2;
        float q3q3 = _q3*_q3;

        // Gradient descent step (IMU)
        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*_q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4.0f*q0q0*_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4.0f*q1q1*_q3 - _2q1*ax + 4.0f*q2q2*_q3 - _2q2*ay;

        // Normalize step magnitude
        norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
        if (norm > 1e-24f) {
            norm = inv_sqrtf_fast(norm);
            s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

            // Apply feedback step
            qDot0 -= beta * s0;
            qDot1 -= beta * s1;
            qDot2 -= beta * s2;
            qDot3 -= beta * s3;
        }
    }

    // Integrate to yield quaternion
    _q0 += qDot0 * dt;
    _q1 += qDot1 * dt;
    _q2 += qDot2 * dt;
    _q3 += qDot3 * dt;

    // Normalize quaternion
    norm = inv_sqrtf_fast(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
    *q0 = _q0 * norm;
    *q1 = _q1 * norm;
    *q2 = _q2 * norm;
    *q3 = _q3 * norm;
}

/* =========================
 *  Public API (multi-instance)
 * ========================= */

void imu_orientation_init(imu_orientation_t *st, float beta)
{
    if (!st) return;
    st->init = false;
    st->ready = false;
    st->have_sample = false;
    st->t_prev_us = 0;
    st->q_BW = q_ident();
    st->q_zero = q_ident();
    st->beta = (beta > 0.0f) ? beta : 0.05f;
}

bool imu_orientation_ready(const imu_orientation_t *st)
{
    return st && st->ready && st->have_sample;
}

void imu_orientation_recenter(imu_orientation_t *st)
{
    if (!st || !st->ready) return;
    st->q_zero = st->q_BW;
}

imu_quat_t imu_orientation_get(const imu_orientation_t *st)
{
    if (!st) return q_ident();
    // return relative orientation (zeroed)
    imu_quat_t q = q_mul(q_conj(st->q_zero), st->q_BW);
    return q_normed(q);
}

void imu_orientation_update(
    imu_orientation_t *st,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps,
    uint64_t t_us
){
    if (!st) return;
    const float d2r = (float)M_PI / 180.0f;

    vec3 aB = map_accel(ax_g, ay_g, az_g);
    vec3 wB = map_gyro((gx_dps - GYRO_BIAS_X_DPS)*d2r,
                       (gy_dps - GYRO_BIAS_Y_DPS)*d2r,
                       gz_dps*d2r);

    if (!st->init) {
        st->init = true;
        st->t_prev_us = t_us;
        st->q_BW = q_ident();
        st->q_zero = st->q_BW;
        st->ready = false;
        st->have_sample = false;
        return; // need next sample for dt
    }

    float dt = (float)(t_us - st->t_prev_us) * 1e-6f;
    if (dt <= 0 || dt > 0.05f) {
        st->t_prev_us = t_us;
        return;
    }
    st->t_prev_us = t_us;
    st->have_sample = true;

    float q0 = st->q_BW.w, q1 = st->q_BW.x, q2 = st->q_BW.y, q3 = st->q_BW.z;
    madgwick_step(&q0, &q1, &q2, &q3,
                  wB.x, wB.y, wB.z,
                  aB.x, aB.y, aB.z,
                  dt, st->beta);
    st->q_BW.w = q0; st->q_BW.x = q1; st->q_BW.y = q2; st->q_BW.z = q3;

    if (!st->ready) {
        st->ready = true;
        st->q_zero = st->q_BW;
    }
}
