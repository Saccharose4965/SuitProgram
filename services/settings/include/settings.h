#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SETTINGS_LED_BACKEND_BEAT = 0,
    SETTINGS_LED_BACKEND_AMBIENT = 1,
} settings_led_backend_t;

typedef struct {
    settings_led_backend_t backend;
    int mode;
    uint8_t ambient_speed_percent;
    bool plane_background_enabled;
    bool ring_background_enabled;
} settings_led_animation_t;

typedef struct {
    uint8_t primary_r, primary_g, primary_b;
    uint8_t secondary_r, secondary_g, secondary_b;
    uint8_t color_cycle;
    uint8_t color_style;
    uint8_t highlight_mode;
    bool audio_brightness_enabled;
    uint8_t audio_energy_range;
    uint8_t brightness;
} settings_led_color_t;

typedef struct {
    float speaker_volume; // 0.0 .. 2.0
    float bt_volume;      // 0.0 .. 2.0
    bool  speaker_muted;
    bool  bt_muted;
    bool  use_2048_direction_icons;
    settings_led_animation_t led_animation;
    settings_led_color_t led_color;
} settings_t;

void settings_init(void);
const settings_t *settings_get(void);
void settings_set_audio(float speaker_volume,
                            bool speaker_muted,
                            float bt_volume,
                            bool bt_muted,
                            bool persist);
void settings_set_direction_icons(bool use_2048_direction_icons, bool persist);
void settings_set_led_animation(const settings_led_animation_t *animation,
                                bool persist);
void settings_set_led_color(const settings_led_color_t *color, bool persist);
esp_err_t settings_apply_led_runtime(void);
