#pragma once
#include "orientation.h"

typedef struct { float x, y, z; } arm_vec3_t;
typedef struct { imu_quat_t zero; bool calibrated; } arm_pose_t;

// Estimated upper-arm direction from forearm pose; shoulder is not measured.
arm_vec3_t arm_pose_estimate_upper_arm(bool left, arm_vec3_t forearm);

// Heading-only reference: folded forearm, preserving gravity/tilt.
bool arm_pose_calibrate(arm_pose_t *pose, imu_quat_t sensor);
// Compatibility spelling; identical shared policy, not a different mode.
bool arm_pose_calibrate_heading(arm_pose_t *pose, imu_quat_t sensor);
// Rotate a raw MPU vector into the calibrated body frame (right, up, forward).
bool arm_pose_vector(const arm_pose_t *pose, imu_quat_t sensor, bool left,
                     arm_vec3_t local, arm_vec3_t *direction);
bool arm_pose_direction(const arm_pose_t *pose, imu_quat_t sensor, bool left,
                        arm_vec3_t *direction);
// Virtual 0.6 x 1.5 m canvas, 1.5 m ahead, centred at the waist.
bool arm_pose_canvas(arm_vec3_t direction, float *x, float *y);
float arm_brush_distance(float x, float y, float ax, float ay, float bx, float by);
