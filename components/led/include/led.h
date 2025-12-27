#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the WS2812B strip on PIN_LED_STRIP_A / GPIO32.
esp_err_t led_init(void);

// Length of the LED strip in pixels (GRB)
#ifndef LED_STRIP_LENGTH
#define LED_STRIP_LENGTH 1400
#endif

// Drive the LED on (true) or off (false).
esp_err_t led_set(bool on);

// Push a full GRB frame (count pixels from the start of the strip).
// Missing pixels will be cleared; excess is ignored.
esp_err_t led_show_pixels(const uint8_t *grb, size_t count);

// Optional helper to toggle the current state.
esp_err_t led_toggle(void);

// Spawn a travelling LED pulse in the given RGB color (0-255 per channel).
void sendpulse(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
