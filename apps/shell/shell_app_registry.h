#pragma once

#include "app_shell.h"
#include "keyboard.h"
#include "link.h"
#include "remote_control_app.h"

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

void led_layout_app_init(shell_app_context_t *ctx);
void led_layout_app_deinit(shell_app_context_t *ctx);
void led_layout_app_tick(shell_app_context_t *ctx, float dt_sec);
void led_layout_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void led_layout_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t LED_LAYOUT_LEGEND;

void fft_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void fft_app_init(shell_app_context_t *ctx);
void fft_app_deinit(shell_app_context_t *ctx);
void fft_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);

void show_player_app_init(shell_app_context_t *ctx);
void show_player_app_deinit(shell_app_context_t *ctx);
void show_player_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void show_player_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t SHOW_PLAYER_LEGEND;

void bt_app_init_wrapper(shell_app_context_t *ctx);
void bt_app_deinit_wrapper(shell_app_context_t *ctx);
void bt_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void bt_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t BT_AUDIO_LEGEND;

void file_rx_app_init(shell_app_context_t *ctx);
void file_rx_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void file_rx_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t FILE_RX_LEGEND;

void flappy_app_init(shell_app_context_t *ctx);
void flappy_app_deinit(shell_app_context_t *ctx);
void flappy_app_tick(shell_app_context_t *ctx, float dt_sec);
void flappy_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void flappy_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);

void t2048_app_init(shell_app_context_t *ctx);
void t2048_app_deinit(shell_app_context_t *ctx);
void t2048_app_tick(shell_app_context_t *ctx, float dt_sec);
void t2048_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void t2048_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t T2048_LEGEND;

void tetris_app_init(shell_app_context_t *ctx);
void tetris_app_deinit(shell_app_context_t *ctx);
void tetris_app_tick(shell_app_context_t *ctx, float dt_sec);
void tetris_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void tetris_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t TETRIS_LEGEND;

void snake_app_init(shell_app_context_t *ctx);
void snake_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void snake_app_tick(shell_app_context_t *ctx, float dt_sec);
void snake_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t SNAKE_LEGEND;

void pong_app_init_wrapper(shell_app_context_t *ctx);
void pong_app_deinit_wrapper(shell_app_context_t *ctx);
void pong_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev);
void pong_tick_wrapper(shell_app_context_t *ctx, float dt_sec);
void pong_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern shell_legend_t PONG_LEGEND;

void threedee_app_init(shell_app_context_t *ctx);
void threedee_app_deinit(shell_app_context_t *ctx);
void threedee_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void threedee_app_tick(shell_app_context_t *ctx, float dt_sec);
void threedee_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t THREEDEE_LEGEND;

void stickman_app_init(shell_app_context_t *ctx);
void gyro_app_init(shell_app_context_t *ctx);
void gyro_app_deinit(shell_app_context_t *ctx);
void gyro_app_tick(shell_app_context_t *ctx,float dt);
void gyro_app_input(shell_app_context_t *ctx,const input_event_t *ev);
void gyro_app_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h);
void stickman_app_deinit(shell_app_context_t *ctx);
void light_paint_init(shell_app_context_t *ctx);
void light_paint_deinit(shell_app_context_t *ctx);
void light_paint_input(shell_app_context_t *ctx,const input_event_t *ev);
void light_paint_tick(shell_app_context_t *ctx,float dt);
void light_paint_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h);
extern shell_legend_t LIGHT_PAINT_LEGEND;
void stickman_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void stickman_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
extern const shell_legend_t STICKMAN_LEGEND;

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
