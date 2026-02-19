#pragma once

#include "app_shell.h"
#include "app_keyboard.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *menu_hud_label(void);
void menu_init(shell_app_context_t *ctx);
void menu_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void menu_tick(shell_app_context_t *ctx, float dt_sec);
void menu_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t MENU_LEGEND;

void status_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void status_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t STATUS_LEGEND;

void volume_init(shell_app_context_t *ctx);
void volume_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void volume_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t VOLUME_LEGEND;

void service_restart_app_init(shell_app_context_t *ctx);
void service_restart_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void service_restart_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t SERVICE_RESTART_LEGEND;

void adc_debug_app_init(shell_app_context_t *ctx);
void adc_debug_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void adc_debug_app_tick(shell_app_context_t *ctx, float dt_sec);
void adc_debug_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t ADC_DEBUG_LEGEND;

void leds_audio_app_init(shell_app_context_t *ctx);
void leds_custom_app_init(shell_app_context_t *ctx);
void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t LEDS_LEGEND;

void fft_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void fft_app_init(shell_app_context_t *ctx);
void fft_app_deinit(shell_app_context_t *ctx);
void fft_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t FFT_LEGEND;

void music_app_init(shell_app_context_t *ctx);
void music_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void music_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
void music_stop_playback(void);
extern shell_legend_t MUSIC_LEGEND;

void bt_app_init_wrapper(shell_app_context_t *ctx);
void bt_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void bt_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t BT_AUDIO_LEGEND;

void file_rx_app_init(shell_app_context_t *ctx);
void file_rx_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void file_rx_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t FILE_RX_LEGEND;

void flappy_app_init(shell_app_context_t *ctx);
void flappy_app_deinit(shell_app_context_t *ctx);
void flappy_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void flappy_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t FLAPPY_LEGEND;

void t2048_app_init(shell_app_context_t *ctx);
void t2048_app_deinit(shell_app_context_t *ctx);
void t2048_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void t2048_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t T2048_LEGEND;

void tetris_app_init(shell_app_context_t *ctx);
void tetris_app_deinit(shell_app_context_t *ctx);
void tetris_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t TETRIS_LEGEND;

void snake_app_init_wrapper(shell_app_context_t *ctx);
void snake_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void snake_tick_wrapper(shell_app_context_t *ctx, float dt_sec);
void snake_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t SNAKE_LEGEND;

void pong_app_init_wrapper(shell_app_context_t *ctx);
void pong_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void pong_tick_wrapper(shell_app_context_t *ctx, float dt_sec);
void pong_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t PONG_LEGEND;

void threedee_app_init_wrapper(shell_app_context_t *ctx);
void threedee_app_deinit_wrapper(shell_app_context_t *ctx);
void threedee_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void threedee_tick_wrapper(shell_app_context_t *ctx, float dt_sec);
void threedee_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t THREEDEE_LEGEND;

void fluid_app_init_wrapper(shell_app_context_t *ctx);
void fluid_app_deinit_wrapper(shell_app_context_t *ctx);
void fluid_app_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void fluid_app_tick_wrapper(shell_app_context_t *ctx, float dt_sec);
void fluid_app_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t FLUID_LEGEND;

void bad_apple_app_init(shell_app_context_t *ctx);
void bad_apple_app_deinit(shell_app_context_t *ctx);
void bad_apple_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void bad_apple_app_tick(shell_app_context_t *ctx, float dt_sec);
void bad_apple_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t BAD_APPLE_LEGEND;

void calculator_app_init(shell_app_context_t *ctx);
void calculator_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void calculator_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t CALCULATOR_LEGEND;

#ifdef __cplusplus
}
#endif
