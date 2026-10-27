#include "shell_ui.h"
#include "ui_draw.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Local framebuffer helpers (1bpp linear buffer)
static inline void fb_pset(uint8_t *fb, int x, int y) {
  if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H)
    return;
  int idx = y * PANEL_W + x;
  fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static inline void fb_pset_clip(uint8_t *fb, int x, int y, int x0, int y0,
                                int x1, int y1) {
  if ((unsigned)x < (unsigned)x0 || (unsigned)x > (unsigned)x1)
    return;
  if ((unsigned)y < (unsigned)y0 || (unsigned)y > (unsigned)y1)
    return;
  fb_pset(fb, x, y);
}

static void fb_rect_clear(uint8_t *fb, int x0, int y0, int w, int h) {
  int x1 = x0 + w - 1;
  int y1 = y0 + h - 1;
  if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H)
    return;
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 >= PANEL_W)
    x1 = PANEL_W - 1;
  if (y1 >= PANEL_H)
    y1 = PANEL_H - 1;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      int idx = y * PANEL_W + x;
      fb[idx >> 3] &= (uint8_t)~(1u << (7 - (idx & 7)));
    }
  }
}

// ----------------------------------------------------------------------
// 5x5 legend icons + helpers
// ----------------------------------------------------------------------
static const uint8_t ICONS[][5] = {
    {0, 0, 0, 0, 0},                               // NONE
    {0b00100, 0b01110, 0b11111, 0b00100, 0b00100}, // UP
    {0b00100, 0b00100, 0b11111, 0b01110, 0b00100}, // DOWN
    {0b00110, 0b01100, 0b11111, 0b01100, 0b00110}, // LEFT
    {0b01100, 0b00110, 0b11111, 0b00110, 0b01100}, // RIGHT
    {0b00100, 0b01110, 0b11011, 0b01110, 0b00100}, // SELECT
    {0b00110, 0b01001, 0b00010, 0b10010, 0b01100}, // REFRESH
    {0b10001, 0b01010, 0b00100, 0b01010, 0b10001}, // MUTE (X)
    {0b01000, 0b01100, 0b01110, 0b01100, 0b01000}, // PLAY (triangle)
    {0b11011, 0b11011, 0b11011, 0b11011, 0b11011}, // PAUSE (bars)
    {0b00110, 0b00101, 0b00100, 0b01100, 0b01100}, // music note
    {0b01110, 0b01010, 0b11111, 0b11111, 0b11111}, // lock
    {0b01110, 0b11111, 0b11111, 0b11111, 0b01110}, // record
    {0b11111, 0b11111, 0b11111, 0b11111, 0b11111}, // stop
};

static const uint8_t DIR_ICONS_2048[][5] = {
    {0b00100, 0b01010, 0b10001, 0b10001, 0b11111}, // UP
    {0b11111, 0b10001, 0b10001, 0b01010, 0b00100}, // DOWN
    {0b00111, 0b01001, 0b10001, 0b01001, 0b00111}, // LEFT
    {0b11100, 0b10010, 0b10001, 0b10010, 0b11100}, // RIGHT
};

static shell_direction_icon_style_t s_direction_icon_style =
    SHELL_DIRECTION_ICON_STYLE_SHELL;
static shell_ui_capture_t s_capture;

static void copy_capture_text(char *dst, size_t dst_len, const char *src) {
  if (!dst || dst_len == 0)
    return;
  snprintf(dst, dst_len, "%s", src ? src : "");
}

void shell_ui_capture_begin(void) { memset(&s_capture, 0, sizeof(s_capture)); }

void shell_ui_capture_get(shell_ui_capture_t *out) {
  if (out)
    *out = s_capture;
}

static void capture_entries(shell_ui_capture_kind_t kind, const char *title,
                            const shell_menu_entry_t *entries, size_t count,
                            size_t selected, size_t first) {
  if (!entries || count == 0)
    return;
  if (count > UINT8_MAX)
    count = UINT8_MAX;
  if (selected >= count)
    selected = count - 1;
  if (first > selected)
    first = selected;
  if (first >= count)
    first = count - 1;
  size_t available = count - first;
  size_t captured = available < SHELL_UI_CAPTURE_MAX_ENTRIES
                        ? available
                        : SHELL_UI_CAPTURE_MAX_ENTRIES;
  s_capture.kind = kind;
  s_capture.total_count = (uint8_t)count;
  s_capture.selected = (uint8_t)selected;
  s_capture.first = (uint8_t)first;
  s_capture.entry_count = (uint8_t)captured;
  copy_capture_text(s_capture.title, sizeof(s_capture.title), title);
  for (size_t i = 0; i < captured; ++i) {
    copy_capture_text(s_capture.labels[i], sizeof(s_capture.labels[i]),
                      entries[first + i].label);
  }
}

static const uint8_t *legend_icon_rows(shell_icon_id_t icon) {
  if (icon >= SHELL_ICON_UP && icon <= SHELL_ICON_RIGHT &&
      s_direction_icon_style == SHELL_DIRECTION_ICON_STYLE_2048) {
    return DIR_ICONS_2048[icon - SHELL_ICON_UP];
  }
  if (icon <= SHELL_ICON_NONE || icon > SHELL_ICON_STOP)
    return NULL;
  return ICONS[icon];
}

static void draw_icon5(uint8_t *fb, int x, int y, shell_icon_id_t icon) {
  const uint8_t *rows = legend_icon_rows(icon);
  if (!rows)
    return;
  for (int r = 0; r < 5; ++r) {
    uint8_t row = rows[r];
    for (int c = 0; c < 5; ++c) {
      if (row & (1u << (4 - c))) {
        fb_pset(fb, x + c, y + r);
      }
    }
  }
}

void shell_ui_set_direction_icon_style(shell_direction_icon_style_t style) {
  if (style != SHELL_DIRECTION_ICON_STYLE_2048) {
    style = SHELL_DIRECTION_ICON_STYLE_SHELL;
  }
  s_direction_icon_style = style;
}

shell_direction_icon_style_t shell_ui_get_direction_icon_style(void) {
  return s_direction_icon_style;
}

static const char *live_sync_hud_name(system_live_sync_t state) {
  switch (state) {
  case SYS_LIVE_SYNC_FOLLOW_A:
    return "A";
  case SYS_LIVE_SYNC_FOLLOW_B:
    return "B";
  case SYS_LIVE_SYNC_FOLLOW_C:
    return "C";
  default:
    return "loc";
  }
}

void shell_ui_draw_hud(uint8_t *fb, const system_state_t *state,
                       const char *left_label) {
  if (!fb || !state)
    return;

  fb_rect_clear(fb, 0, 0, PANEL_W, SHELL_UI_HUD_HEIGHT);

  const int margin = 2;
  char status_buf[32];
  if (state->fft_running) {
    unsigned bpm = (unsigned)((state->fft_bpm_centi + 50u) / 100u);
    if (bpm > 0u) {
      snprintf(status_buf, sizeof(status_buf), "fft:%u sync:%s%s", bpm,
               live_sync_hud_name(state->live_sync),
               state->sync_link_active ? "*" : "");
    } else {
      snprintf(status_buf, sizeof(status_buf), "fft:on sync:%s%s",
               live_sync_hud_name(state->live_sync),
               state->sync_link_active ? "*" : "");
    }
  } else {
    snprintf(status_buf, sizeof(status_buf), "fft:off sync:%s%s",
             live_sync_hud_name(state->live_sync),
             state->sync_link_active ? "*" : "");
  }

  int status_w = (int)strlen(status_buf) * 4 - 1;
  if (status_w < 0)
    status_w = 0;
  int status_x = PANEL_W - margin - status_w;
  if (status_x < margin)
    status_x = margin;
  oled_draw_text3x5(fb, status_x, 1, status_buf);

  const char *label = left_label ? left_label : state->led_mode_name;
  if (label && *label) {
    int max_chars = (status_x - margin - 2) / 4;
    if (max_chars < 0)
      max_chars = 0;
    if ((int)strlen(label) > max_chars) {
      static char scratch[32];
      int n = max_chars;
      if (n > (int)sizeof(scratch) - 1)
        n = (int)sizeof(scratch) - 1;
      strncpy(scratch, label, n);
      scratch[n] = '\0';
      oled_draw_text3x5(fb, margin, 1, scratch);
    } else {
      oled_draw_text3x5(fb, margin, 1, label);
    }
  }

  for (int x = 0; x < PANEL_W; ++x) {
    fb_pset(fb, x, SHELL_UI_HUD_HEIGHT - 1);
  }
}

void shell_ui_draw_legend(uint8_t *fb, const shell_legend_t *legend) {
  if (!fb || !legend)
    return;
  s_capture.legend = *legend;

  // Clear legend band before drawing icons
  fb_rect_clear(fb, 0, PANEL_H - SHELL_UI_LEGEND_HEIGHT, PANEL_W,
                SHELL_UI_LEGEND_HEIGHT);

  int y = PANEL_H - SHELL_UI_LEGEND_HEIGHT +
          1; // leave 1px gap above, 1px gap below
  int spacing = PANEL_W / 4;
  for (int i = 0; i < 4; ++i) {
    if (legend->slots[i] == SHELL_ICON_NONE)
      continue;
    int cx = spacing * i + spacing / 2 - 2;
    draw_icon5(fb, cx, y, legend->slots[i]);
  }
}

static void draw_centered_text(uint8_t *fb, int x, int w, int y,
                               const char *text) {
  if (!text || !*text)
    return;
  int text_w = (int)strlen(text) * 4 - 1;
  oled_draw_text3x5(fb, x + (w - text_w) / 2, y, text);
}

static void draw_clipped_text(uint8_t *fb, int x, int y, int w,
                              const char *text) {
  if (!fb || !text || !*text || w < 3)
    return;
  size_t max_chars = (size_t)((w + 1) / 4);
  if (max_chars > 47u)
    max_chars = 47u;
  char clipped[48];
  snprintf(clipped, sizeof(clipped), "%.*s", (int)max_chars, text);
  oled_draw_text3x5(fb, x, y, clipped);
}

void shell_ui_draw_text_page(uint8_t *fb, int x, int y, int w, int h,
                             const shell_text_page_t *page) {
  if (!fb || !page || w <= 4 || h <= 4)
    return;
  const int left = x + 2;
  const int text_w = w - 4;
  int line_y = y + 2;
  if (page->title && page->title[0]) {
    draw_clipped_text(fb, left, line_y, text_w, page->title);
    line_y += 8;
  }
  for (size_t i = 0; i < page->line_count && line_y + 5 <= y + h; ++i) {
    draw_clipped_text(fb, left, line_y, text_w,
                      page->lines ? page->lines[i] : NULL);
    line_y += 8;
  }
}

void shell_ui_draw_overlay(uint8_t *fb, int x, int y, int w, int h,
                           const shell_overlay_view_t *view) {
  if (!fb || !view || w < 3 || h < 3)
    return;
  fb_rect_clear(fb, x, y, w, h);
  for (int px = x; px < x + w; ++px) {
    fb_pset(fb, px, y);
    fb_pset(fb, px, y + h - 1);
  }
  for (int py = y; py < y + h; ++py) {
    fb_pset(fb, x, py);
    fb_pset(fb, x + w - 1, py);
  }

  int line_y = y + 4;
  draw_centered_text(fb, x, w, line_y, view->title);
  draw_centered_text(fb, x, w, line_y + 8, view->line1);
  draw_centered_text(fb, x, w, line_y + 14, view->line2);
  draw_centered_text(fb, x, w, y + h - 8, view->prompt);
}

// ----------------------------------------------------------------------
// Menu list rendering with caller-owned smooth scrolling
// ----------------------------------------------------------------------
void shell_ui_menu_reset(shell_menu_motion_t *motion, size_t selected) {
  if (motion)
    motion->scroll = (float)selected;
}

void shell_ui_menu_tick(shell_menu_motion_t *motion, float dt_sec,
                        size_t selected) {
  if (!motion)
    return;
  float target = (float)selected;
  float diff = target - motion->scroll;
  if (fabsf(diff) < 0.01f) {
    motion->scroll = target;
    return;
  }

  float speed = 10.0f; // items per second toward target
  float step = diff * fminf(1.0f, dt_sec * speed);
  // Clamp step to avoid huge jumps on long frames
  if (step > 0.5f)
    step = 0.5f;
  if (step < -0.5f)
    step = -0.5f;
  motion->scroll += step;
}

static void draw_menu_icon(uint8_t *fb, int x, int y, const uint16_t *sprite,
                           int clip_x0, int clip_y0, int clip_x1, int clip_y1) {
  static const uint16_t placeholder[9] = {
      0b001111100, 0b011111110, 0b111111111, 0b111001111, 0b111001111,
      0b111111111, 0b111111111, 0b011111110, 0b001111100,
  };
  for (int r = 0; r < 9; ++r) {
    uint16_t row = sprite ? sprite[r] : placeholder[r];
    for (int c = 0; c < 9; ++c) {
      if (row & (1u << (8 - c))) {
        fb_pset_clip(fb, x + c, y + r, clip_x0, clip_y0, clip_x1, clip_y1);
      }
    }
  }
}

static void draw_choice(uint8_t *fb, int x, int y, int w, const char *label,
                        const uint16_t *sprite, bool selected, int clip_x0,
                        int clip_y0, int clip_x1, int clip_y1) {
  const int frame_h = 15, icon_w = 9, outer_pad = 3, icon_gap = 4;
  int fy0 = y, fy1 = y + frame_h - 1;
  if (fy1 < clip_y0 || fy0 > clip_y1)
    return;

  draw_menu_icon(fb, x + outer_pad, y + (frame_h - icon_w) / 2, sprite, clip_x0,
                 clip_y0, clip_x1, clip_y1);

  int fx0 = x + outer_pad + icon_w + icon_gap;
  int fx1 = x + w - outer_pad - 1;
  if (fx1 <= fx0)
    return;
  for (int dx = fx0 + 1; dx < fx1; ++dx) {
    fb_pset_clip(fb, dx, fy0, clip_x0, clip_y0, clip_x1, clip_y1);
    fb_pset_clip(fb, dx, fy1, clip_x0, clip_y0, clip_x1, clip_y1);
  }
  for (int dy = fy0 + 1; dy < fy1; ++dy) {
    fb_pset_clip(fb, fx0, dy, clip_x0, clip_y0, clip_x1, clip_y1);
    fb_pset_clip(fb, fx1, dy, clip_x0, clip_y0, clip_x1, clip_y1);
  }
  fb_pset_clip(fb, fx0 + 1, fy0 + 1, clip_x0, clip_y0, clip_x1, clip_y1);
  fb_pset_clip(fb, fx1 - 1, fy0 + 1, clip_x0, clip_y0, clip_x1, clip_y1);
  fb_pset_clip(fb, fx0 + 1, fy1 - 1, clip_x0, clip_y0, clip_x1, clip_y1);
  fb_pset_clip(fb, fx1 - 1, fy1 - 1, clip_x0, clip_y0, clip_x1, clip_y1);
  if (selected) {
    for (int dy = fy0 + 2; dy <= fy1 - 2; ++dy) {
      fb_pset_clip(fb, fx0 + 1, dy, clip_x0, clip_y0, clip_x1, clip_y1);
      fb_pset_clip(fb, fx0 + 2, dy, clip_x0, clip_y0, clip_x1, clip_y1);
    }
  }
  int text_y = y + (frame_h - 5) / 2;
  if (text_y <= clip_y1 && text_y + 4 >= clip_y0)
    oled_draw_text3x5(fb, fx0 + 4, text_y, label);
}

void shell_ui_draw_menu(uint8_t *fb, int x, int y, int w, int h,
                        const shell_menu_view_t *view) {
  if (!fb || !view || !view->entries || view->count == 0)
    return;

  size_t selected = view->selected;
  if (selected >= view->count)
    selected = view->count - 1;
  size_t capture_first = selected > 4 ? selected - 4 : 0;
  if (capture_first + SHELL_UI_CAPTURE_MAX_ENTRIES > view->count &&
      view->count > SHELL_UI_CAPTURE_MAX_ENTRIES) {
    capture_first = view->count - SHELL_UI_CAPTURE_MAX_ENTRIES;
  }
  capture_entries(SHELL_UI_CAPTURE_MENU, view->title, view->entries,
                  view->count, selected, capture_first);

  const int frame_h = 15;
  const int gap = 1;
  const int visible = (view->count < 3) ? (int)view->count : 3;
  const int item_h = frame_h + gap;
  const int stack_h = frame_h * visible + gap * (visible - 1);
  int y0 = y + (h - stack_h) / 2;
  if (y0 < y + 1)
    y0 = y + 1;

  // Window start (in items, fractional) centers selection when possible
  float scroll = view->motion ? view->motion->scroll : (float)selected;
  float window_start = scroll - 1.0f;
  float min_start = 0.0f;
  float max_start = (float)view->count - (float)visible;
  if (max_start < 0.0f)
    max_start = 0.0f;
  if (window_start < min_start)
    window_start = min_start;
  if (window_start > max_start)
    window_start = max_start;

  int start_idx = (int)floorf(window_start);
  float frac = window_start - (float)start_idx;
  float base_y = (float)y0 - frac * item_h;

  int items_to_draw = visible + 1; // extra for fractional scroll
  int clip_x0 = x;
  int clip_x1 = x + w - 1;
  int clip_y0 = y;
  int clip_y1 = y + h - 1;
  for (int i = 0; i < items_to_draw; ++i) {
    int idx = start_idx + i;
    if (idx >= (int)view->count)
      break;
    float fy_f = base_y + i * item_h;
    int fy = (int)lroundf(fy_f);
    const uint16_t *sprite = view->sprites ? view->sprites[idx] : NULL;
    draw_choice(fb, x, fy, w, view->entries[idx].label, sprite,
                (size_t)idx == selected, clip_x0, clip_y0, clip_x1, clip_y1);
  }
}

void shell_ui_grid_reset(shell_grid_motion_t *motion, size_t selected) {
  if (motion)
    motion->column = (float)(selected / 3u);
}

void shell_ui_grid_tick(shell_grid_motion_t *motion, float dt_sec,
                        size_t selected) {
  if (!motion)
    return;
  float target = (float)(selected / 3u);
  float diff = target - motion->column;
  if (fabsf(diff) < 0.01f) {
    motion->column = target;
    return;
  }
  float step = diff * fminf(1.0f, dt_sec * 10.0f);
  if (step > 0.5f)
    step = 0.5f;
  if (step < -0.5f)
    step = -0.5f;
  motion->column += step;
}

void shell_ui_draw_grid(uint8_t *fb, int x, int y, int w, int h,
                        const shell_grid_view_t *view) {
  if (!fb || !view || !view->entries || !view->count)
    return;
  size_t selected =
      view->selected < view->count ? view->selected : view->count - 1;
  size_t selected_column = selected / 3u;
  size_t capture_first = selected_column > 0 ? (selected_column - 1u) * 3u : 0u;
  if (capture_first + SHELL_UI_CAPTURE_MAX_ENTRIES > view->count &&
      view->count > SHELL_UI_CAPTURE_MAX_ENTRIES) {
    capture_first = ((view->count - 1u) / 3u > 1u)
                        ? (((view->count - 1u) / 3u) - 2u) * 3u
                        : 0u;
  }
  capture_entries(SHELL_UI_CAPTURE_GRID, NULL, view->entries, view->count,
                  selected, capture_first);
  float column = view->motion ? view->motion->column : (float)(selected / 3u);
  int first = (int)floorf(column);
  const int frame_h = 15, gap = 1, item_h = frame_h + gap;
  int card_w = (w * 3) / 4;
  if (card_w < 32)
    card_w = w;
  int column_gap = w / 32;
  if (column_gap < 2)
    column_gap = 2;
  int stride = card_w + column_gap;
  int center_x = x + (w - card_w) / 2;
  const int stack_h = frame_h * 3 + gap * 2;
  int y0 = y + (h - stack_h) / 2;
  if (y0 < y + 1)
    y0 = y + 1;
  int clip_x0 = x, clip_x1 = x + w - 1, clip_y0 = y, clip_y1 = y + h - 1;

  // Include both neighbours and one extra column during fractional motion.
  for (int col = first - 1; col <= first + 2; ++col) {
    if (col < 0)
      continue;
    size_t base = (size_t)col * 3u;
    if (base >= view->count)
      continue;
    int column_x =
        center_x + (int)lroundf(((float)col - column) * (float)stride);
    for (size_t row = 0; row < 3u && base + row < view->count; ++row) {
      size_t idx = base + row;
      const uint16_t *sprite = view->sprites ? view->sprites[idx] : NULL;
      draw_choice(fb, column_x, y0 + (int)row * item_h, card_w,
                  view->entries[idx].label, sprite, idx == selected, clip_x0,
                  clip_y0, clip_x1, clip_y1);
    }
  }
}

void shell_ui_draw_remote_corners(uint8_t *fb, bool controller) {
  if (!fb)
    return;
  const int arm = controller ? 5 : 3;
  for (int i = 0; i < arm; ++i) {
    fb_pset(fb, i, 0);
    fb_pset(fb, 0, i);
    fb_pset(fb, PANEL_W - 1 - i, 0);
    fb_pset(fb, PANEL_W - 1, i);
    fb_pset(fb, i, PANEL_H - 1);
    fb_pset(fb, 0, PANEL_H - 1 - i);
    fb_pset(fb, PANEL_W - 1 - i, PANEL_H - 1);
    fb_pset(fb, PANEL_W - 1, PANEL_H - 1 - i);
  }
}
