#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the WS2812B strip on PIN_LED_STRIP_A / GPIO32.
esp_err_t led_init(void);

// LED strip length used by effects/rendering and transmission.
#ifndef LED_STRIP_LENGTH
#define LED_STRIP_LENGTH 721 // changed from 720, this
#endif

// Strip byte order is fixed to RGB for this hardware.

static inline size_t led_pixel_offset(size_t pixel_index)
{
    return pixel_index * 3u;
}

// Canonical color-order adapter. All LED writers should go through this.
static inline void led_pack_rgb(uint8_t *buf, size_t idx, uint8_t r, uint8_t g, uint8_t b){
    buf[idx + 0] = r;
    buf[idx + 1] = g;
    buf[idx + 2] = b;
}

// Inverse mapping from the strip order back to RGB.
static inline void led_unpack_rgb(const uint8_t *buf, size_t idx,
                                  uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r) *r = buf[idx + 0];
    if (g) *g = buf[idx + 1];
    if (b) *b = buf[idx + 2];
}

static inline void led_set_pixel_rgb(uint8_t *buf, size_t pixel_index,
                                     uint8_t r, uint8_t g, uint8_t b)
{
    led_pack_rgb(buf, led_pixel_offset(pixel_index), r, g, b);
}

static inline void led_get_pixel_rgb(const uint8_t *buf, size_t pixel_index,
                                     uint8_t *r, uint8_t *g, uint8_t *b)
{
    led_unpack_rgb(buf, led_pixel_offset(pixel_index), r, g, b);
}

// Drive the LED on (true) or off (false).
esp_err_t led_set(bool on);

// Push a full frame in strip byte order (fixed RGB).
// Missing pixels will be cleared; excess is ignored.
esp_err_t led_show_pixels(const uint8_t *frame, size_t count);

// Optional helper to toggle the current state.
esp_err_t led_toggle(void);

// Spawn a travelling LED pulse in the given RGB color (0-255 per channel).
void sendpulse(uint8_t r, uint8_t g, uint8_t b);

typedef enum {
    LED_BEAT_ANIM_FLASH = 0,
    LED_BEAT_ANIM_PULSE = 1,
    LED_BEAT_ANIM_WAVE = 2,
    LED_BEAT_ANIM_PULSE_DUAL = 3,
    LED_BEAT_ANIM_SPARK = 4,
    LED_BEAT_ANIM_COMET = 5,
    LED_BEAT_ANIM_SHOCK = 6,
    LED_BEAT_ANIM_SPARK_RAIN = 7,
} led_beat_anim_t;

// Beat animation mode helpers (used by FFT beat trigger path).
int led_beat_anim_count(void);
const char *led_beat_anim_name(int idx);
void led_beat_anim_set(led_beat_anim_t mode);
led_beat_anim_t led_beat_anim_get(void);
void led_beat_enable(bool enabled);
bool led_beat_enabled(void);

// Trigger one beat event using currently selected animation mode.
void led_trigger_beat(uint8_t r, uint8_t g, uint8_t b);
void led_beat_color_set(uint8_t r, uint8_t g, uint8_t b);
void led_beat_color_get(uint8_t *r, uint8_t *g, uint8_t *b);
void led_beat_brightness_set(uint8_t level);
uint8_t led_beat_brightness_get(void);

// Run a blocking LED hardware test pattern (RGBW + chase).
// Useful for validating data line / level-shifter / strip power.
esp_err_t led_run_test_pattern(void);

#ifdef __cplusplus
}
#endif
