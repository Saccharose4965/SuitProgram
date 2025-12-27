// IMU-driven 3D renderer for the OLED shell.
#pragma once

#include <stdint.h>
#include "input.h"
#include "orientation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*threedee_request_switch_fn)(const char *id, void *user_data);

void threedee_set_request_switch(threedee_request_switch_fn fn, void *user_data);
void threedee_set_orientation_ctx(imu_orientation_t *ctx);
void threedee_set_mount_quat(imu_quat_t q_ds); // sensor->device mounting correction

void threedee_app_init(void);
void threedee_app_deinit(void);
void threedee_app_handle_input(const input_event_t *ev);
void threedee_app_tick(float dt_sec);
void threedee_app_draw(uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
