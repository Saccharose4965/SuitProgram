#include "live_sync.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"
#include "fft.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "led.h"
#include "led_modes.h"
#include "led_source.h"
#include "show_runtime.h"

enum {
  PROTOCOL = 0x4c,
  PKT_PING = 1,
  PKT_PONG,
  PKT_STATE,
  PKT_RELEASE,
  MAX_PEERS = 8,
  PING_INTERVAL_US = 700000,
  STATE_INTERVAL_US = 250000,
  PEER_STALE_US = 3500000,
  LINK_TIMEOUT_US = 2500000,
  MIN_LEAD_US = 120000,
  MAX_LEAD_US = 450000,
  FLAG_BEAT = 1u << 0,
  FLAG_AMBIENT = 1u << 1,
};

typedef struct __attribute__((packed)) {
  uint8_t protocol, kind, role, reserved;
  uint32_t session, token;
  int64_t t1;
} ping_packet_t;
typedef struct __attribute__((packed)) {
  uint8_t protocol, kind, role, reserved;
  uint32_t session, token;
  int64_t t1, t2, t3;
} pong_packet_t;
typedef struct __attribute__((packed)) {
  uint8_t protocol, kind, role, flags;
  uint32_t session, sequence;
  uint16_t bpm_centi, phase_milli;
  uint8_t mode, timing, brightness, speed;
  uint8_t pr, pg, pb, sr, sg, sb, cycle, style, highlight, plane, ring,
      audio_brightness, energy;
  int64_t beat_anchor_us, animation_sample_us;
  uint32_t animation_time_ms;
} state_packet_t;
typedef struct __attribute__((packed)) {
  uint8_t protocol, kind, role, reserved;
  uint32_t session;
} release_packet_t;

typedef struct {
  bool used, valid;
  uint8_t mac[6];
  int64_t offset, one_way, seen;
} peer_t;
typedef struct {
  bool active;
  uint8_t role;
  uint32_t session, token, sequence;
  int64_t last_ping, last_state;
  peer_t peers[MAX_PEERS];
} publisher_t;
typedef struct {
  bool active, beat_active, ambient;
  uint8_t role, mac[6], mode;
  uint32_t session, sequence;
  int64_t updated, period, target_period, next_beat;
  int64_t animation_sample;
  uint32_t animation_ms;
  float bpm;
  led_beat_settings_t beat;
  led_modes_settings_t ambient_cfg;
} follower_t;
typedef struct {
  bool valid, beat_enabled, ambient_enabled;
  led_beat_anim_t beat_mode;
  int ambient_mode;
  led_beat_settings_t beat;
  led_modes_settings_t ambient;
} restore_t;
typedef struct {
  bool beat_active, ambient;
  float bpm, phase, animation_time;
  int64_t next_beat;
  uint8_t mode;
  led_beat_settings_t beat;
  led_modes_settings_t ambient_cfg;
} source_t;

static publisher_t s_pub;
static follower_t s_follow;
static restore_t s_restore;
static bool s_was_following;
static const uint8_t s_broadcast[6] = {255, 255, 255, 255, 255, 255};
typedef struct {
  uint8_t mac[6];
  size_t len;
  uint8_t data[sizeof(state_packet_t)];
} rx_event_t;
static StaticQueue_t s_rx_queue_storage;
static uint8_t s_rx_queue_buffer[8 * sizeof(rx_event_t)];
static QueueHandle_t s_rx_queue;

static int64_t clamp64(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static float clamp_bpm(float v) {
  return v < 32.0f ? 32.0f : (v > 255.0f ? 255.0f : v);
}
static int beat_mode(int v) {
  return v >= 0 && v < led_beat_anim_count() ? v : LED_BEAT_ANIM_FLASH;
}
static int ambient_mode(int v) {
  return v >= 0 && v < led_modes_count() ? v : 0;
}
static int64_t bpm_period(float bpm) {
  return (int64_t)llround(60000000.0 / clamp_bpm(bpm));
}
static bool same_mac(const uint8_t *a, const uint8_t *b) {
  return a && b && memcmp(a, b, 6) == 0;
}
static float phase_remaining(float phase, float trigger) {
  float r = trigger - phase;
  while (r <= 0.000001f)
    r += 1.0f;
  while (r > 1.0f)
    r -= 1.0f;
  return r;
}
static uint16_t pack_phase(float p) {
  while (p < 0)
    p += 1;
  while (p >= 1)
    p -= 1;
  return (uint16_t)lroundf(p * 1000);
}

static void save_local(void) {
  if (s_restore.valid)
    return;
  s_restore.valid = true;
  s_restore.beat_enabled = led_beat_enabled();
  s_restore.ambient_enabled = led_modes_enabled();
  s_restore.beat_mode = led_beat_anim_get();
  s_restore.ambient_mode = led_modes_current();
  led_beat_settings_get(&s_restore.beat);
  led_modes_settings_get(&s_restore.ambient);
}
static void restore_local(void) {
  if (!s_restore.valid)
    return;
  led_beat_settings_apply(&s_restore.beat);
  led_modes_settings_apply(&s_restore.ambient);
  led_beat_anim_set(s_restore.beat_mode);
  led_modes_set(s_restore.ambient_mode);
  led_beat_enable(s_restore.beat_enabled);
  led_modes_enable(s_restore.ambient_enabled);
  led_modes_set_sync_clock(false, 0);
  s_restore.valid = false;
}

static void capture_source(int64_t now, source_t *out) {
  memset(out, 0, sizeof(*out));
  led_beat_settings_get(&out->beat);
  led_modes_settings_get(&out->ambient_cfg);
  out->ambient = led_modes_enabled();
  out->mode = (uint8_t)(out->ambient ? ambient_mode(led_modes_current())
                                     : beat_mode(led_beat_anim_get()));
  if (out->ambient &&
      (!led_modes_get_sync_clock(&out->animation_time) ||
       !isfinite(out->animation_time) || out->animation_time < 0))
    out->animation_time = 0;
  manual_bpm_sync_state_t manual = {0};
  if (led_source_manual_state_get(&manual) && manual.active &&
      manual.bpm > 0.5f) {
    out->bpm = clamp_bpm(manual.bpm);
    out->phase = manual.phase_offset;
    out->beat_active = true;
    int64_t period = bpm_period(out->bpm);
    out->next_beat =
        now +
        (int64_t)llround((double)period * phase_remaining(manual.cycle_phase,
                                                          manual.phase_offset));
    return;
  }
  fft_sync_state_t fft = {0};
  fft_get_sync_state(&fft);
  if (fft.running && fft.beat_enabled && fft.bpm > 0.5f) {
    out->bpm = clamp_bpm(fft.bpm);
    out->phase = fft.phase_offset;
    out->beat_active = true;
    int64_t period = bpm_period(out->bpm);
    out->next_beat = now + (int64_t)llround((double)period *
                                            phase_remaining(fft.beat_phase,
                                                            fft.trigger_phase));
  }
}

static peer_t *find_peer(const uint8_t mac[6], bool create) {
  peer_t *free_slot = NULL, *oldest = NULL;
  for (size_t i = 0; i < MAX_PEERS; i++) {
    peer_t *p = &s_pub.peers[i];
    if (p->used && same_mac(p->mac, mac))
      return p;
    if (!p->used && !free_slot)
      free_slot = p;
    if (p->used && (!oldest || p->seen < oldest->seen))
      oldest = p;
  }
  if (!create)
    return NULL;
  peer_t *p = free_slot ? free_slot : oldest;
  if (!p)
    return NULL;
  memset(p, 0, sizeof(*p));
  p->used = true;
  memcpy(p->mac, mac, 6);
  return p;
}
static void send_release(void) {
  if (!s_pub.active || link_active_path() == LINK_PATH_NONE)
    return;
  release_packet_t p = {PROTOCOL, PKT_RELEASE, s_pub.role, 0, s_pub.session};
  (void)link_send_frame_to(s_broadcast, LINK_MSG_CONTROL, (const uint8_t *)&p,
                           sizeof(p), false);
}
static void stop_publisher(void) {
  if (s_pub.active)
    send_release();
  memset(&s_pub, 0, sizeof(s_pub));
}
static void start_publisher(uint8_t role) {
  stop_publisher();
  s_pub.active = true;
  s_pub.role = role;
  s_pub.session = esp_random();
  if (!s_pub.session)
    s_pub.session = 1;
}
static void send_ping(int64_t now) {
  ping_packet_t p = {PROTOCOL,      PKT_PING,      s_pub.role, 0,
                     s_pub.session, ++s_pub.token, now};
  (void)link_send_frame_to(s_broadcast, LINK_MSG_CONTROL, (const uint8_t *)&p,
                           sizeof(p), false);
}
static void fill_settings(state_packet_t *p, const source_t *s) {
  if (s->ambient) {
    const led_modes_settings_t *c = &s->ambient_cfg;
    p->brightness = c->brightness;
    p->speed = c->speed_percent;
    p->pr = c->primary_r;
    p->pg = c->primary_g;
    p->pb = c->primary_b;
    p->sr = c->secondary_r;
    p->sg = c->secondary_g;
    p->sb = c->secondary_b;
    p->cycle = c->color_cycle;
    p->style = c->color_style;
    p->highlight = c->highlight_mode;
    p->plane = c->plane_background_enabled;
    p->ring = c->ring_background_enabled;
  } else {
    const led_beat_settings_t *c = &s->beat;
    p->brightness = c->brightness;
    p->speed = 100;
    p->pr = c->primary_r;
    p->pg = c->primary_g;
    p->pb = c->primary_b;
    p->sr = c->secondary_r;
    p->sg = c->secondary_g;
    p->sb = c->secondary_b;
    p->cycle = c->color_cycle;
    p->style = c->color_style;
    p->highlight = c->highlight_mode;
    p->plane = c->plane_background_enabled;
    p->ring = c->ring_background_enabled;
    p->audio_brightness = c->audio_brightness_enabled;
    p->energy = c->audio_energy_range;
  }
}
static void send_state(int64_t now) {
  source_t s;
  capture_source(now, &s);
  int64_t period = s.beat_active ? bpm_period(s.bpm) : 0;
  for (size_t i = 0; i < MAX_PEERS; i++) {
    peer_t *peer = &s_pub.peers[i];
    if (!peer->used || !peer->valid)
      continue;
    if (now - peer->seen > PEER_STALE_US) {
      memset(peer, 0, sizeof(*peer));
      continue;
    }
    int64_t lead = clamp64(peer->one_way * 2 + 80000, MIN_LEAD_US, MAX_LEAD_US),
            anchor = s.next_beat;
    while (period > 0 && anchor - now < lead)
      anchor += period;
    state_packet_t p = {
        .protocol = PROTOCOL,
        .kind = PKT_STATE,
        .role = s_pub.role,
        .flags =
            (s.beat_active ? FLAG_BEAT : 0) | (s.ambient ? FLAG_AMBIENT : 0),
        .session = s_pub.session,
        .sequence = ++s_pub.sequence,
        .bpm_centi = s.beat_active ? (uint16_t)lroundf(s.bpm * 100) : 0,
        .phase_milli = pack_phase(s.phase),
        .mode = s.mode,
        .timing = (uint8_t)led_source_timing_get(),
        .beat_anchor_us = period > 0 ? anchor + peer->offset : 0,
        .animation_sample_us = now + peer->offset,
        .animation_time_ms = (uint32_t)lroundf(s.animation_time * 1000)};
    fill_settings(&p, &s);
    (void)link_send_frame_to(peer->mac, LINK_MSG_CONTROL, (const uint8_t *)&p,
                             sizeof(p), false);
  }
}
static void publisher_tick(int64_t now, uint8_t role) {
  if (!s_pub.active || s_pub.role != role)
    start_publisher(role);
  if (link_active_path() == LINK_PATH_NONE)
    return;
  if (now - s_pub.last_ping >= PING_INTERVAL_US) {
    send_ping(now);
    s_pub.last_ping = now;
  }
  if (now - s_pub.last_state >= STATE_INTERVAL_US) {
    send_state(now);
    s_pub.last_state = now;
  }
}

static void apply_follower(void) {
  if (s_follow.ambient) {
    led_beat_enable(false);
    led_modes_settings_apply(&s_follow.ambient_cfg);
    led_modes_set(ambient_mode(s_follow.mode));
    led_modes_enable(true);
  } else {
    led_modes_enable(false);
    led_beat_settings_apply(&s_follow.beat);
    led_beat_anim_set((led_beat_anim_t)beat_mode(s_follow.mode));
    led_beat_enable(true);
  }
}
static void follower_tick(int64_t now) {
  if (!s_follow.active)
    return;
  if (s_follow.ambient && s_follow.animation_sample > 0) {
    float t = (float)s_follow.animation_ms / 1000 +
              (float)(now - s_follow.animation_sample) / 1000000;
    led_modes_set_sync_clock(true, t);
  }
  if (!s_follow.beat_active || s_follow.period <= 0 || s_follow.next_beat <= 0)
    return;
  while (now >= s_follow.next_beat) {
    led_trigger_beat(255, 96, 0);
    s_follow.period +=
        clamp64((s_follow.target_period - s_follow.period) / 8, -4000, 4000);
    s_follow.next_beat += s_follow.period;
  }
}

bool live_sync_blocks_app_tick(const char *app_id) {
  return led_source_is_following() && app_id &&
         strcmp(app_id, "manual_bpm") == 0;
}
void live_sync_get_status(live_sync_status_t *out) {
  if (!out)
    return;
  memset(out, 0, sizeof(*out));
  show_runtime_status_t role = show_runtime_get_status();
  out->role_ready = role.role_ready;
  out->local_role = role.role_ready ? role.role_id : UINT8_MAX;
  out->following = led_source_is_following();
  out->source_role =
      out->following ? (uint8_t)led_source_follow_role() : UINT8_MAX;
  if (!out->following || !s_follow.active) {
    out->age_ms = -1;
    return;
  }
  int64_t age = esp_timer_get_time() - s_follow.updated;
  out->age_ms = s_follow.updated ? (int)(age / 1000) : -1;
  out->linked = s_follow.updated && age <= LINK_TIMEOUT_US;
  out->bpm = s_follow.bpm;
}

static void receive_state(const uint8_t mac[6], const state_packet_t *p,
                          int64_t now) {
  if (!mac || p->role != (uint8_t)led_source_follow_role())
    return;
  bool fresh = !s_follow.active || !same_mac(mac, s_follow.mac) ||
               p->session != s_follow.session;
  if (!fresh && (int32_t)(p->sequence - s_follow.sequence) <= 0)
    return;
  if (fresh) {
    memset(&s_follow, 0, sizeof(s_follow));
    memcpy(s_follow.mac, mac, 6);
    s_follow.role = p->role;
    s_follow.session = p->session;
  }
  s_follow.sequence = p->sequence;
  s_follow.active = true;
  s_follow.updated = now;
  s_follow.beat_active = (p->flags & FLAG_BEAT) != 0;
  s_follow.ambient = (p->flags & FLAG_AMBIENT) != 0;
  s_follow.mode = p->mode;
  s_follow.bpm = p->bpm_centi ? (float)p->bpm_centi / 100 : 0;
  int64_t period = s_follow.beat_active && s_follow.bpm > 0.5f
                       ? bpm_period(s_follow.bpm)
                       : 0,
          anchor = p->beat_anchor_us;
  while (period > 0 && anchor <= now)
    anchor += period;
  if (fresh || s_follow.next_beat <= 0 || !s_follow.beat_active) {
    s_follow.period = period;
    s_follow.next_beat = anchor;
  } else if (period > 0 && anchor > 0) {
    int64_t d = anchor - s_follow.next_beat;
    if (llabs(d) > period)
      s_follow.next_beat = anchor;
    else
      s_follow.next_beat += clamp64(d / 8, -6000, 6000);
  }
  s_follow.target_period = period;
  s_follow.animation_sample = p->animation_sample_us;
  s_follow.animation_ms = p->animation_time_ms;
  s_follow.beat =
      (led_beat_settings_t){.primary_r = p->pr,
                            .primary_g = p->pg,
                            .primary_b = p->pb,
                            .secondary_r = p->sr,
                            .secondary_g = p->sg,
                            .secondary_b = p->sb,
                            .color_cycle = p->cycle,
                            .color_style = p->style,
                            .highlight_mode = p->highlight,
                            .plane_background_enabled = p->plane,
                            .ring_background_enabled = p->ring,
                            .audio_brightness_enabled = p->audio_brightness,
                            .audio_energy_range = p->energy,
                            .brightness = p->brightness};
  s_follow.ambient_cfg =
      (led_modes_settings_t){.brightness = p->brightness,
                             .speed_percent = p->speed,
                             .primary_r = p->pr,
                             .primary_g = p->pg,
                             .primary_b = p->pb,
                             .secondary_r = p->sr,
                             .secondary_g = p->sg,
                             .secondary_b = p->sb,
                             .plane_background_enabled = p->plane,
                             .ring_background_enabled = p->ring,
                             .color_cycle = p->cycle,
                             .color_style = p->style,
                             .highlight_mode = p->highlight};
  apply_follower();
}

static void process_packet(const uint8_t *mac, const uint8_t *data,
                           size_t len) {
  int64_t now = esp_timer_get_time();
  switch (data[1]) {
  case PKT_PING: {
    if (!led_source_is_following() || !mac || len < sizeof(ping_packet_t))
      return;
    const ping_packet_t *q = (const void *)data;
    if (q->role != (uint8_t)led_source_follow_role())
      return;
    pong_packet_t p = {PROTOCOL, PKT_PONG,   q->role,
                       0,        q->session, q->token,
                       q->t1,    now,        esp_timer_get_time()};
    (void)link_send_frame_to(mac, LINK_MSG_CONTROL, (const uint8_t *)&p,
                             sizeof(p), false);
    break;
  }
  case PKT_PONG: {
    if (!s_pub.active || !mac || len < sizeof(pong_packet_t))
      return;
    const pong_packet_t *p = (const void *)data;
    if (p->role != s_pub.role || p->session != s_pub.session)
      return;
    peer_t *peer = find_peer(mac, true);
    if (!peer)
      return;
    int64_t rtt = (now - p->t1) - (p->t3 - p->t2);
    if (rtt < 0)
      rtt = 0;
    int64_t offset = ((p->t2 - p->t1) + (p->t3 - now)) / 2;
    if (peer->valid) {
      peer->offset = (peer->offset * 3 + offset) / 4;
      peer->one_way = (peer->one_way * 3 + rtt / 2) / 4;
    } else {
      peer->offset = offset;
      peer->one_way = rtt / 2;
      peer->valid = true;
    }
    peer->seen = now;
    s_pub.last_state = 0;
    break;
  }
  case PKT_STATE:
    if (led_source_is_following() && len >= sizeof(state_packet_t))
      receive_state(mac, (const void *)data, now);
    break;
  case PKT_RELEASE: {
    if (!led_source_is_following() || len < sizeof(release_packet_t))
      return;
    const release_packet_t *p = (const void *)data;
    if (s_follow.active && same_mac(mac, s_follow.mac) &&
        p->role == s_follow.role && p->session == s_follow.session)
      s_follow.updated = 0;
    break;
  }
  default:
    break;
  }
}

void live_sync_service_tick(float dt_sec) {
  (void)dt_sec;
  if (!s_rx_queue) {
    s_rx_queue = xQueueCreateStatic(8, sizeof(rx_event_t), s_rx_queue_buffer,
                                    &s_rx_queue_storage);
  }
  rx_event_t event;
  while (s_rx_queue && xQueueReceive(s_rx_queue, &event, 0) == pdTRUE) {
    process_packet(event.mac, event.data, event.len);
  }
  show_runtime_set_role_presence(true);
  int64_t now = esp_timer_get_time();
  bool following = led_source_is_following();
  if (following && !s_was_following) {
    save_local();
    stop_publisher();
    memset(&s_follow, 0, sizeof(s_follow));
  } else if (!following && s_was_following) {
    memset(&s_follow, 0, sizeof(s_follow));
    restore_local();
  }
  s_was_following = following;
  if (following) {
    follower_tick(now);
    return;
  }
  show_runtime_status_t role = show_runtime_get_status();
  if (role.role_ready && role.role_id < 3)
    publisher_tick(now, role.role_id);
  else
    stop_publisher();
}

void live_sync_handle_link_frame(link_msg_type_t type, const uint8_t *mac,
                                 const uint8_t *data, size_t len) {
  if (type != LINK_MSG_CONTROL || !mac || !data || len < 2 ||
      data[0] != PROTOCOL || len > sizeof(((rx_event_t *)0)->data) ||
      !s_rx_queue)
    return;
  rx_event_t event = {.len = len};
  memcpy(event.mac, mac, 6);
  memcpy(event.data, data, len);
  (void)xQueueSend(s_rx_queue, &event, 0);
}
