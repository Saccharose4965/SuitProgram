#pragma once
#include "orientation.h"

typedef struct { float x, y, z; } arm_vec3_t;
typedef struct { imu_quat_t zero; bool calibrated; } arm_pose_t;

// Recenter with the forearm horizontal across the chest (folded).
bool arm_pose_calibrate(arm_pose_t *pose, imu_quat_t sensor);
// Painting: heading only, using projected +Y; retain gravity as vertical.
bool arm_pose_calibrate_heading(arm_pose_t *pose, imu_quat_t sensor);
// Rotate a raw MPU vector into the calibrated body frame (right, up, forward).
bool arm_pose_vector(const arm_pose_t *pose, imu_quat_t sensor, bool left,
                     arm_vec3_t local, arm_vec3_t *direction);
bool arm_pose_direction(const arm_pose_t *pose, imu_quat_t sensor, bool left,
                        arm_vec3_t *direction);
// Virtual 0.6 x 1.5 m canvas, 1.5 m ahead, centred at the waist.
bool arm_pose_canvas(arm_vec3_t direction, float *x, float *y);
float arm_brush_distance(float x, float y, float ax, float ay, float bx, float by);
