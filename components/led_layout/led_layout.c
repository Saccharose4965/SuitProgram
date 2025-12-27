#include "led_layout.h"
#include "led.h" // for LED_STRIP_LENGTH default
#include <math.h>

// Placeholder procedural layout:
// - Wraps LEDs around the torso as a loose helix (z forward, x right, y up).
// - Spreads the strip over ~1.2 m height and 0.35 m radius.
// Replace this with real measured coordinates per LED/segment when available.

bool led_layout_get(int idx, led_point_t *out){
    if (!out) return false;
    const int count = LED_STRIP_LENGTH;
    if (idx < 0 || idx >= count) return false;
    float t = (float)idx / (float)(count > 1 ? (count - 1) : 1);
    float turns = 4.0f;
    float angle = t * turns * 2.0f * (float)M_PI;
    float radius = 0.35f; // meters
    out->x = radius * cosf(angle);
    out->z = radius * sinf(angle);
    out->y = -0.6f + 1.2f * t; // from hips (-0.6 m) to shoulders (+0.6 m)
    return true;
}

int led_layout_count(void){
    return LED_STRIP_LENGTH;
}
