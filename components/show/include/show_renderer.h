#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "show_package.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct show_renderer show_renderer_t;

show_renderer_t *show_renderer_create(void);
void show_renderer_destroy(show_renderer_t *renderer);
esp_err_t show_renderer_configure(show_renderer_t *renderer,
                                  const show_package_program_t *program,
                                  uint8_t role_id);
esp_err_t show_renderer_render(show_renderer_t *renderer, uint32_t playhead_ms,
                               uint8_t brightness, uint8_t *frame,
                               size_t frame_pixels, size_t *out_pixels);

#ifdef __cplusplus
}
#endif
