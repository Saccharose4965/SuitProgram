#pragma once

#include "app_shell.h"

void leds_animations_app_init(shell_app_context_t *ctx);
void leds_ambient_app_init(shell_app_context_t *ctx);
void leds_app_deinit(shell_app_context_t *ctx);
void leds_app_tick(shell_app_context_t *ctx, float dt_sec);
void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t LEDS_LEGEND;

void led_source_app_init(shell_app_context_t *ctx);
void led_source_app_tick(shell_app_context_t *ctx, float dt_sec);
void led_source_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void led_source_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t LED_SOURCE_LEGEND;

void led_color_app_init(shell_app_context_t *ctx);
void led_color_app_deinit(shell_app_context_t *ctx);
void led_color_app_tick(shell_app_context_t *ctx, float dt_sec);
void led_color_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void led_color_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t LED_COLOR_LEGEND;

void manual_bpm_app_init(shell_app_context_t *ctx);
void manual_bpm_app_deinit(shell_app_context_t *ctx);
void manual_bpm_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void manual_bpm_app_tick(shell_app_context_t *ctx, float dt_sec);
void manual_bpm_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t MANUAL_BPM_LEGEND;

void fft_sync_app_init(shell_app_context_t *ctx);
void fft_sync_app_deinit(shell_app_context_t *ctx);
void fft_sync_app_tick(shell_app_context_t *ctx, float dt_sec);
void fft_sync_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void fft_sync_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t FFT_SYNC_LEGEND;
