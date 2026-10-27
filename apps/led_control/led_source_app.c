#include "led_control_apps.h"

#include <stdio.h>

#include "led_source.h"
#include "live_sync.h"

const shell_legend_t LED_SOURCE_LEGEND = {
    .slots = {SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE,
              SHELL_ICON_SELECT},
};

enum {
  LED_SOURCE_ENTRY_BACK = 0,
  LED_SOURCE_ENTRY_FFT,
  LED_SOURCE_ENTRY_MANUAL,
  LED_SOURCE_ENTRY_FOLLOW_A,
  LED_SOURCE_ENTRY_FOLLOW_B,
  LED_SOURCE_ENTRY_FOLLOW_C,
  LED_SOURCE_ENTRY_COUNT,
};

typedef struct {
  size_t selected;
  shell_menu_motion_t motion;
  char fft_label[18];
  char manual_label[18];
  char follow_labels[3][18];
  char title[18];
} led_source_app_state_t;

static led_source_app_state_t s_source_app = {
    .selected = LED_SOURCE_ENTRY_FFT,
};

static void led_source_refresh_labels(void) {
  led_source_mode_t mode = led_source_mode_get();
  live_sync_status_t sync = {0};
  live_sync_get_status(&sync);
  snprintf(s_source_app.title, sizeof(s_source_app.title), "LED / SOURCE %c",
           sync.role_ready && sync.local_role < 3 ? 'A' + sync.local_role : '?');
  snprintf(s_source_app.fft_label, sizeof(s_source_app.fft_label), "FFT%s",
           mode == LED_SOURCE_FFT ? " *" : "");
  snprintf(s_source_app.manual_label, sizeof(s_source_app.manual_label),
           "Manual%s", mode == LED_SOURCE_MANUAL ? " *" : "");
  for (int role = 0; role < 3; ++role) {
    bool selected = mode == LED_SOURCE_FOLLOW_A + role;
    snprintf(s_source_app.follow_labels[role],
             sizeof(s_source_app.follow_labels[role]), "Follow %c%s",
             'A' + role, selected ? (sync.linked ? " *" : " lost") : "");
  }
}

void led_source_app_init(shell_app_context_t *ctx) {
  (void)ctx;
  led_source_refresh_labels();
  led_source_mode_t mode = led_source_mode_get();
  s_source_app.selected = mode == LED_SOURCE_MANUAL
                              ? LED_SOURCE_ENTRY_MANUAL
                          : mode >= LED_SOURCE_FOLLOW_A && mode <= LED_SOURCE_FOLLOW_C
                              ? LED_SOURCE_ENTRY_FOLLOW_A + (mode - LED_SOURCE_FOLLOW_A)
                              : LED_SOURCE_ENTRY_FFT;
  shell_ui_menu_reset(&s_source_app.motion, s_source_app.selected);
}

void led_source_app_tick(shell_app_context_t *ctx, float dt_sec) {
  (void)ctx;
  shell_ui_menu_tick(&s_source_app.motion, dt_sec, s_source_app.selected);
}

void led_source_app_handle_input(shell_app_context_t *ctx,
                                 const input_event_t *ev) {
  if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS)
    return;

  if (ev->button == INPUT_BTN_A) {
    if (s_source_app.selected > 0)
      s_source_app.selected--;
    return;
  }
  if (ev->button == INPUT_BTN_B) {
    if (s_source_app.selected + 1 < LED_SOURCE_ENTRY_COUNT)
      s_source_app.selected++;
    return;
  }
  if (ev->button != INPUT_BTN_D) {
    return;
  }

  switch (s_source_app.selected) {
  case LED_SOURCE_ENTRY_BACK:
    if (ctx->request_switch) {
      ctx->request_switch("menu", ctx->request_user_data);
    }
    break;
  case LED_SOURCE_ENTRY_FFT:
    led_source_mode_set(LED_SOURCE_FFT);
    break;
  case LED_SOURCE_ENTRY_MANUAL:
    led_source_mode_set(LED_SOURCE_MANUAL);
    if (ctx->request_switch)
      ctx->request_switch("manual_bpm", ctx->request_user_data);
    break;
  case LED_SOURCE_ENTRY_FOLLOW_A:
  case LED_SOURCE_ENTRY_FOLLOW_B:
  case LED_SOURCE_ENTRY_FOLLOW_C:
    led_source_mode_set(
        (led_source_mode_t)(LED_SOURCE_FOLLOW_A + (s_source_app.selected -
                                                   LED_SOURCE_ENTRY_FOLLOW_A)));
    break;
  default:
    break;
  }
}

void led_source_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y,
                         int w, int h) {
  (void)ctx;
  if (!fb)
    return;

  led_source_refresh_labels();

  const shell_menu_entry_t entries[LED_SOURCE_ENTRY_COUNT] = {
      {.id = "back", .label = "Back"},
      {.id = "fft", .label = s_source_app.fft_label},
      {.id = "manual", .label = s_source_app.manual_label},
      {.id = "follow_a", .label = s_source_app.follow_labels[0]},
      {.id = "follow_b", .label = s_source_app.follow_labels[1]},
      {.id = "follow_c", .label = s_source_app.follow_labels[2]},
  };

  shell_menu_view_t view = {
      .entries = entries,
      .count = LED_SOURCE_ENTRY_COUNT,
      .selected = s_source_app.selected,
      .title = s_source_app.title,
      .motion = &s_source_app.motion,
  };
  shell_ui_draw_menu(fb, x, y, w, h, &view);
}
