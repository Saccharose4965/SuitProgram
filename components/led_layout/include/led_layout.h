#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LED_LAYOUT_MAX_PIXELS
#define LED_LAYOUT_MAX_PIXELS 721
#endif

#define LED_LAYOUT_MAX_STRIPS   2
#define LED_LAYOUT_MAX_SECTIONS 24
#define LED_LAYOUT_NAME_LEN     20

typedef struct {
    float x; // torso-space units, right
    float y; // torso-space units, up
    float z; // torso-space units, forward
} led_point_t;

typedef enum {
    LED_LAYOUT_GEOM_NONE = 0,
    LED_LAYOUT_GEOM_POLYLINE = 1,
    LED_LAYOUT_GEOM_ARC = 2,
} led_layout_geom_t;

#define LED_LAYOUT_MAX_SECTION_POINTS 8

typedef struct {
    char name[LED_LAYOUT_NAME_LEN];
    uint8_t strip;
    uint16_t length;
    bool reversed;
    bool connected_to_prev;
    uint8_t geom_kind;
    uint8_t point_count;
    led_point_t points[LED_LAYOUT_MAX_SECTION_POINTS];
    led_point_t center;
    float radius;
    float start_deg;
    float sweep_deg;
    uint16_t logical_start;
    uint16_t physical_start;
} led_layout_section_t;

typedef struct {
    char profile[LED_LAYOUT_NAME_LEN];
    uint8_t strip_count;
    char strip_names[LED_LAYOUT_MAX_STRIPS][LED_LAYOUT_NAME_LEN];
    uint8_t section_count;
    uint16_t strip_lengths[LED_LAYOUT_MAX_STRIPS];
    uint16_t total_leds;
    led_layout_section_t sections[LED_LAYOUT_MAX_SECTIONS];
} led_layout_config_t;

typedef struct {
    uint8_t strip;
    uint16_t physical_index;
    uint16_t section_index;
    uint16_t section_offset;
} led_layout_mapping_t;

// Load the persisted layout from SD. If the file is missing, a default
// chestplate layout is created in memory and written to SD when possible.
esp_err_t led_layout_init(void);
bool led_layout_ready(void);

// Copy the current layout state. If the layout has not been initialized yet,
// this returns the built-in default configuration.
void led_layout_snapshot(led_layout_config_t *out);

size_t led_layout_count(void);
size_t led_layout_strip_count(void);
size_t led_layout_strip_length(int strip);

bool led_layout_map_logical(size_t logical_index, led_layout_mapping_t *out);
bool led_layout_map_logical_from_config(const led_layout_config_t *cfg,
                                        size_t logical_index,
                                        led_layout_mapping_t *out);

// Placeholder spatial lookup used by future spatial effects.
bool led_layout_get(int idx, led_point_t *out);
bool led_layout_get_from_config(const led_layout_config_t *cfg, int idx, led_point_t *out);

// Live editing helpers used by the shell app.
esp_err_t led_layout_set_section_strip(size_t idx, uint8_t strip);
esp_err_t led_layout_set_section_length(size_t idx, uint16_t length);
esp_err_t led_layout_set_section_reversed(size_t idx, bool reversed);
esp_err_t led_layout_set_section_start_deg(size_t idx, float start_deg);

esp_err_t led_layout_save(void);
esp_err_t led_layout_reload(void);
esp_err_t led_layout_reset_default(bool persist);

#ifdef __cplusplus
}
#endif
