#pragma once

#include "app_shell.h"

void volume_init(shell_app_context_t *ctx);
void volume_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void volume_draw(shell_app_context_t *ctx, uint8_t *fb,
                 int x, int y, int w, int h);
extern const shell_legend_t VOLUME_LEGEND;

void preferences_app_init(shell_app_context_t *ctx);
void preferences_app_handle_input(shell_app_context_t *ctx,
                                  const input_event_t *ev);
void preferences_app_tick(shell_app_context_t *ctx,float dt_sec);
void preferences_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                          int x, int y, int w, int h);
extern const shell_legend_t PREFERENCES_LEGEND;

void service_restart_app_init(shell_app_context_t *ctx);
void service_restart_app_handle_input(shell_app_context_t *ctx,
                                      const input_event_t *ev);
void service_restart_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                              int x, int y, int w, int h);
extern const shell_legend_t SERVICE_RESTART_LEGEND;

void adc_debug_app_init(shell_app_context_t *ctx);
void adc_debug_app_handle_input(shell_app_context_t *ctx,
                                const input_event_t *ev);
void adc_debug_app_tick(shell_app_context_t *ctx, float dt_sec);
void adc_debug_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                        int x, int y, int w, int h);
