#pragma once

#include <stdbool.h>

typedef struct {
  bool active;
  float bpm;
  float cycle_phase;
  float phase_offset;
} manual_bpm_sync_state_t;

typedef enum {
  LED_SOURCE_MANUAL = 0,
  LED_SOURCE_FFT,
  LED_SOURCE_FOLLOW_A,
  LED_SOURCE_FOLLOW_B,
  LED_SOURCE_FOLLOW_C,
} led_source_mode_t;

typedef enum {
  LED_TIMING_ON_BEAT = 0,
  LED_TIMING_OFFBEAT,
  LED_TIMING_DOUBLE,
} led_timing_mode_t;

const char *led_source_mode_name(led_source_mode_t mode);
void led_source_mode_set(led_source_mode_t mode);
led_source_mode_t led_source_mode_get(void);
bool led_source_is_following(void);
int led_source_follow_role(void);
void led_source_service_tick(float dt_sec);
bool led_source_manual_state_get(manual_bpm_sync_state_t *out);
void led_source_manual_bpm_adjust(float delta);
void led_source_manual_phase_adjust(float delta);
void led_source_timing_set(led_timing_mode_t mode);
led_timing_mode_t led_source_timing_get(void);
const char *led_source_timing_name(led_timing_mode_t mode);
