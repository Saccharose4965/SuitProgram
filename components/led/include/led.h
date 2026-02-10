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

// Active LEDs used by effects/rendering.
// This is the range you typically tune during bring-up (e.g. 144).
#ifndef LED_STRIP_LENGTH
#define LED_STRIP_LENGTH 1400
#endif

// Total physical LEDs in the chain.
// Driver will always transmit/clear this many to keep tail LEDs latched off.
#ifndef LED_STRIP_PHYSICAL_LENGTH
#define LED_STRIP_PHYSICAL_LENGTH LED_STRIP_LENGTH
#endif

// Strip byte order on the data line.
#define LED_COLOR_ORDER_GRB 0
#define LED_COLOR_ORDER_RGB 1
#ifndef LED_COLOR_ORDER
#define LED_COLOR_ORDER LED_COLOR_ORDER_GRB
#endif

static inline size_t led_pixel_offset(size_t pixel_index)
{
    return pixel_index * 3u;
}

// Canonical color-order adapter. All LED writers should go through this.
static inline void led_pack_rgb(uint8_t *buf, size_t idx, uint8_t r, uint8_t g, uint8_t b){
#if LED_COLOR_ORDER == LED_COLOR_ORDER_GRB
    buf[idx + 0] = g;
    buf[idx + 1] = r;
    buf[idx + 2] = b;
#else
    buf[idx + 0] = r;
    buf[idx + 1] = g;
    buf[idx + 2] = b;
#endif
}

// Inverse mapping from the strip order back to RGB.
static inline void led_unpack_rgb(const uint8_t *buf, size_t idx,
                                  uint8_t *r, uint8_t *g, uint8_t *b)
{
#if LED_COLOR_ORDER == LED_COLOR_ORDER_GRB
    if (g) *g = buf[idx + 0];
    if (r) *r = buf[idx + 1];
    if (b) *b = buf[idx + 2];
#else
    if (r) *r = buf[idx + 0];
    if (g) *g = buf[idx + 1];
    if (b) *b = buf[idx + 2];
#endif
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

// Push a full frame in configured strip byte order (LED_COLOR_ORDER).
// Missing pixels will be cleared; excess is ignored.
esp_err_t led_show_pixels(const uint8_t *frame, size_t count);

// Optional helper to toggle the current state.
esp_err_t led_toggle(void);

// Spawn a travelling LED pulse in the given RGB color (0-255 per channel).
void sendpulse(uint8_t r, uint8_t g, uint8_t b);

typedef enum {
    LED_BEAT_ANIM_FLASH = 0,
    LED_BEAT_ANIM_PULSE = 1,
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

// Run a blocking LED hardware test pattern (RGBW + chase).
// Useful for validating data line / level-shifter / strip power.
esp_err_t led_run_test_pattern(void);

#ifdef __cplusplus
}
#endif
