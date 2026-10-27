#include "led_source.h"

#include <math.h>
#include <stdint.h>

#include "audio_policy.h"
#include "fft.h"
#include "led.h"
#include "led_modes.h"

static led_source_mode_t s_source_mode = LED_SOURCE_FFT;
static manual_bpm_sync_state_t s_manual = {
    .active = false,
    .bpm = 120.0f,
    .cycle_phase = 0.0f,
    .phase_offset = 0.0f,
};
static uint32_t s_manual_beat_count = 0;
static bool s_fft_phase_valid = false;
static float s_fft_prev_aligned_phase = 0.0f;
static uint32_t s_fft_beat_count = 0;
static led_timing_mode_t s_timing_mode = LED_TIMING_ON_BEAT;

static float clamp_bpm(float bpm) {
  if (bpm < 32.0f)
    bpm = 32.0f;
  if (bpm > 255.0f)
    bpm = 255.0f;
  return bpm;
}

static float wrap01(float v) {
  while (v < 0.0f)
    v += 1.0f;
  while (v >= 1.0f)
    v -= 1.0f;
  return v;
}

bool led_source_is_following(void) {
  return s_source_mode >= LED_SOURCE_FOLLOW_A &&
         s_source_mode <= LED_SOURCE_FOLLOW_C;
}

int led_source_follow_role(void) {
  return led_source_is_following()
             ? (int)s_source_mode - (int)LED_SOURCE_FOLLOW_A
             : -1;
}

static int beat_count_between(float start_phase, float end_phase,
                              float trigger_phase) {
  float start_rel = start_phase - trigger_phase;
  float end_rel = end_phase - trigger_phase;
  int start_wrap = (int)floorf(start_rel);
  int end_wrap = (int)floorf(end_rel);
  return end_wrap - start_wrap;
}

const char *led_source_mode_name(led_source_mode_t mode) {
  switch (mode) {
  case LED_SOURCE_MANUAL:
    return "manual";
  case LED_SOURCE_FOLLOW_A:
    return "follow A";
  case LED_SOURCE_FOLLOW_B:
    return "follow B";
  case LED_SOURCE_FOLLOW_C:
    return "follow C";
  default:
    return "fft";
  }
}

void led_source_mode_set(led_source_mode_t mode) {
  if (mode < LED_SOURCE_MANUAL || mode > LED_SOURCE_FOLLOW_C) {
    mode = LED_SOURCE_FFT;
  }
  if (mode != s_source_mode) {
    s_source_mode = mode;
    s_fft_phase_valid = false;
  }
  audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_LED_SOURCE,
                           s_source_mode == LED_SOURCE_FFT);
  fft_led_export_enable(s_source_mode == LED_SOURCE_FFT);
}

led_source_mode_t led_source_mode_get(void) { return s_source_mode; }

bool led_source_manual_state_get(manual_bpm_sync_state_t *out) {
  if (!out)
    return false;
  *out = s_manual;
  out->active = s_source_mode == LED_SOURCE_MANUAL;
  out->bpm = clamp_bpm(out->bpm);
  out->cycle_phase = wrap01(out->cycle_phase);
  out->phase_offset = wrap01(out->phase_offset);
  return true;
}

void led_source_manual_bpm_adjust(float delta) {
  s_manual.bpm = clamp_bpm(s_manual.bpm + delta);
}

void led_source_manual_phase_adjust(float delta) {
  s_manual.phase_offset = wrap01(s_manual.phase_offset + delta);
}

const char *led_source_timing_name(led_timing_mode_t mode) {
  switch (mode) {
  case LED_TIMING_OFFBEAT:
    return "offbeat";
  case LED_TIMING_DOUBLE:
    return "2x";
  default:
    return "on beat";
  }
}

void led_source_timing_set(led_timing_mode_t mode) {
  if (mode < LED_TIMING_ON_BEAT || mode > LED_TIMING_DOUBLE) {
    mode = LED_TIMING_ON_BEAT;
  }
  s_timing_mode = mode;
  s_manual.phase_offset = (mode == LED_TIMING_OFFBEAT) ? 0.5f : 0.0f;
  fft_beat_offbeat_set(mode == LED_TIMING_OFFBEAT);
  fft_beat_ratio_set(mode == LED_TIMING_DOUBLE ? 2u : 1u);
}

led_timing_mode_t led_source_timing_get(void) { return s_timing_mode; }

void led_source_service_tick(float dt_sec) {
  bool following = led_source_is_following();
  bool want_fft = s_source_mode == LED_SOURCE_FFT;
  audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_LED_SOURCE, want_fft);
  fft_led_export_enable(want_fft);

  if (following) {
    led_modes_set_sync_clock(false, 0.0f);
    return;
  }

  if (s_source_mode == LED_SOURCE_MANUAL) {
    float bpm = clamp_bpm(s_manual.bpm) *
                (s_timing_mode == LED_TIMING_DOUBLE ? 2.0f : 1.0f);
    if (dt_sec > 0.0f) {
      float start_phase = s_manual.cycle_phase;
      float end_phase = start_phase + dt_sec * (bpm / 60.0f);
      int beats =
          beat_count_between(start_phase, end_phase, s_manual.phase_offset);
      while (beats-- > 0) {
        led_trigger_beat(255, 96, 0);
        s_manual_beat_count++;
      }
      s_manual.cycle_phase = wrap01(end_phase);
    }

    float aligned_phase = wrap01(s_manual.cycle_phase - s_manual.phase_offset);
    float hz = bpm / 60.0f;
    float timeline_sec =
        (hz > 0.05f) ? (((float)s_manual_beat_count + aligned_phase) / hz)
                     : 0.0f;
    led_modes_set_sync_clock(true, timeline_sec);
    return;
  }

  fft_sync_state_t st = {0};
  fft_get_sync_state(&st);
  if (!(st.running && st.beat_enabled && st.bpm > 0.5f)) {
    s_fft_phase_valid = false;
    led_modes_set_sync_clock(false, 0.0f);
    return;
  }

  float aligned_phase = wrap01(st.beat_phase - st.trigger_phase);
  if (!s_fft_phase_valid) {
    s_fft_beat_count = 0;
    s_fft_phase_valid = true;
  } else if (aligned_phase + 0.05f < s_fft_prev_aligned_phase) {
    s_fft_beat_count++;
  }
  s_fft_prev_aligned_phase = aligned_phase;

  float hz = st.bpm / 60.0f;
  float timeline_sec =
      (hz > 0.05f) ? (((float)s_fft_beat_count + aligned_phase) / hz) : 0.0f;
  led_modes_set_sync_clock(true, timeline_sec);
}
