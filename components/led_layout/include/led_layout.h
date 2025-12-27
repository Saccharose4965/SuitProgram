#pragma once
#include <stdbool.h>

typedef struct {
    float x; // meters, right
    float y; // meters, up
    float z; // meters, forward
} led_point_t;

// Return true if idx is valid; writes a position in suit/body space.
// The default mapping is a coarse helix placeholder; replace the table in
// led_layout.c with real measured coordinates per LED/segment.
bool led_layout_get(int idx, led_point_t *out);

// Total LEDs supported by the layout (usually LED_STRIP_LENGTH).
int led_layout_count(void);
