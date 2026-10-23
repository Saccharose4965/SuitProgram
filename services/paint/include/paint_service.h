#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "link.h"

typedef enum { PAINT_MENU, PAINT_PAINTER, PAINT_CANVAS } paint_mode_t;
typedef enum {
    PAINT_OK, PAINT_NO_MEMORY, PAINT_NO_LAYOUT, PAINT_LEDS_BUSY,
    PAINT_NEED_HORIZONTAL, PAINT_IMU_UNAVAILABLE, PAINT_LINK_UNAVAILABLE
} paint_error_t;
typedef enum {
    PAINT_DISMISS_ERROR, PAINT_LEAVE, PAINT_SELECT_PAINTER, PAINT_SELECT_CANVAS,
    PAINT_TOGGLE_ARM, PAINT_CALIBRATE, PAINT_NEXT_COLOR, PAINT_TOGGLE_BRUSH, PAINT_CLEAR
} paint_command_t;
typedef struct {
    paint_mode_t mode;
    bool left_arm, calibrated, painting, on_canvas, connected;
    uint8_t color;
    paint_error_t error;
} paint_status_t;

// Singleton session. Open/leave/commands/tick/status share one caller task.
// Only receive may run concurrently; it validates/enqueues, never renders.
// Open once on entry; leave before reopening. Buffers persist between sessions.
void paint_service_open(void);
void paint_service_leave(void);
void paint_service_command(paint_command_t command);
void paint_service_tick(float dt);
paint_status_t paint_service_status(void);
void paint_service_receive(link_msg_type_t type, const uint8_t *mac,
                           const uint8_t *data, size_t len);
