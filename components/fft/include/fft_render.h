#pragma once
#include "fft_display.h"
#include "esp_err.h"

typedef fft_display_packet_t fft_render_packet_t;
// Static mutex only: OLED rendering uses the shell task, no worker stack/queue.
esp_err_t fft_render_init(void);
void fft_render_reset(void);
void fft_render_submit(const fft_render_packet_t *pkt);
void fft_render_update_detector(const detector_result_t *result);
void fft_render_set_view(fft_view_t view);
void fft_render_set_display_enabled(bool enabled);
bool fft_render_is_display_enabled(void);
void fft_render_copy_frame(uint8_t *dst_fb, size_t dst_len);
