#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the LED modes task (idempotent). Initializes the strip driver.
esp_err_t led_modes_start(void);

// Mode selection helpers
int         led_modes_count(void);
const char *led_modes_name(int idx);
void        led_modes_set(int idx);
int         led_modes_current(void);
void        led_modes_enable(bool enabled);
bool        led_modes_enabled(void);

// Sync control: when disabled, beat-reactive modes fall back to free-run.
void led_modes_set_sync(bool enabled);
bool led_modes_sync_enabled(void);

// Global brightness scale (0–255). Default 96.
void led_modes_set_brightness(uint8_t level);
uint8_t led_modes_get_brightness(void);

// Primary color used by standalone mode rendering.
void led_modes_set_primary_color(uint8_t r, uint8_t g, uint8_t b);
void led_modes_get_primary_color(uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
