// Simple accel-driven particle "fluid" demo for the OLED.
#pragma once

#include <stdint.h>
#include "input.h"

#ifdef __cplusplus
extern "C" {
#endif

// Runs until the task is deleted or the user presses button A.
void fluid_run(void);

// Shell-friendly entry points
void fluid_app_init(void);
void fluid_app_deinit(void);
void fluid_app_handle_input(const input_event_t *ev);
void fluid_app_tick(float dt_sec);
void fluid_app_draw(uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
