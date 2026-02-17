#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BAD_APPLE_FRAME_WIDTH  128
#define BAD_APPLE_FRAME_HEIGHT 64
#define BAD_APPLE_FRAME_BYTES  ((BAD_APPLE_FRAME_WIDTH * BAD_APPLE_FRAME_HEIGHT) / 8)

typedef enum {
    BAD_APPLE_SOURCE_AUTO = 0,
    BAD_APPLE_SOURCE_HEATSHRINK_RLE,
    BAD_APPLE_SOURCE_RLE,
    BAD_APPLE_SOURCE_RAW,
} bad_apple_source_type_t;

typedef struct {
    const char *path;
    bad_apple_source_type_t type;
    uint8_t target_fps;
    bool loop;
} bad_apple_config_t;

typedef struct {
    bool playing;
    bool paused;
    bool eof;
    bool frame_valid;
    bool loop;
    uint8_t target_fps;
    uint32_t frame_index;
    size_t file_size_bytes;
    bad_apple_source_type_t source_type;
    esp_err_t last_err;
    char path[128];
} bad_apple_status_t;

const char *bad_apple_source_type_name(bad_apple_source_type_t type);

esp_err_t bad_apple_start(const bad_apple_config_t *cfg);
void bad_apple_stop(void);
esp_err_t bad_apple_restart(void);

void bad_apple_tick(float dt_sec);
esp_err_t bad_apple_copy_frame(uint8_t *dst, size_t dst_size);

void bad_apple_set_paused(bool paused);
void bad_apple_toggle_pause(void);
bool bad_apple_is_paused(void);

void bad_apple_set_target_fps(uint8_t fps);
uint8_t bad_apple_get_target_fps(void);

void bad_apple_get_status(bad_apple_status_t *out);

#ifdef __cplusplus
}
#endif
