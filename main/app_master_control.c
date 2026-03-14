#include "app_shell.h"
#include "shell_apps.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"

#include "fft.h"
#include "led.h"
#include "led_modes.h"
#include "link.h"
#include "oled.h"

const shell_legend_t MASTER_CONTROL_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

enum {
    MASTER_CTRL_PROTO_VER = 4,
    MASTER_CTRL_PKT_PING_REQ = 1,
    MASTER_CTRL_PKT_PING_RESP = 2,
    MASTER_CTRL_PKT_STATE = 3,
    MASTER_CTRL_PKT_RELEASE = 4,
    MASTER_CTRL_MAX_PEERS = 8,
    MASTER_CTRL_STATE_INTERVAL_US = 900000,
    MASTER_CTRL_PING_INTERVAL_US = 700000,
    MASTER_CTRL_SLAVE_TIMEOUT_US = 2500000,
    MASTER_CTRL_PEER_STALE_US = 3500000,
    MASTER_CTRL_MIN_LEAD_US = 120000,
    MASTER_CTRL_MAX_LEAD_US = 450000,
    MASTER_CTRL_PERIOD_NUDGE_MIN_US = 200,
    MASTER_CTRL_PERIOD_NUDGE_MAX_US = 4000,
    MASTER_CTRL_SYNC_TRIM_MIN_US = 250,
    MASTER_CTRL_SYNC_TRIM_MAX_US = 6000,
    MASTER_CTRL_RESYNC_MIN_US = 120000,
    MASTER_CTRL_RESYNC_MAX_US = 900000,
    MASTER_CTRL_STATE_FLAG_BEAT_ACTIVE = 1u << 0,
    MASTER_CTRL_STATE_FLAG_CUSTOM_ACTIVE = 1u << 1,
    MASTER_CTRL_STATE_FLAG_PLANE_BG = 1u << 2,
    MASTER_CTRL_STATE_FLAG_RING_BG = 1u << 3,
    MASTER_CTRL_SOURCE_FLAG_FFT_LOCKED = 1u << 0,
    MASTER_CTRL_FLASH_TICKS = 3,
};

typedef enum {
    MASTER_CTRL_RATIO_MATCH = 0,
    MASTER_CTRL_RATIO_HALF,
    MASTER_CTRL_RATIO_DOUBLE,
    MASTER_CTRL_RATIO_COUNT,
} master_ctrl_ratio_t;

typedef enum {
    MASTER_CTRL_PHASE_ZERO = 0,
    MASTER_CTRL_PHASE_OFFBEAT,
    MASTER_CTRL_PHASE_COUNT,
} master_ctrl_phase_t;

typedef enum {
    MASTER_CTRL_COLOR_TRON = 0,
    MASTER_CTRL_COLOR_MATCH,
    MASTER_CTRL_COLOR_FREE,
    MASTER_CTRL_COLOR_COUNT,
} master_ctrl_color_mode_t;

typedef enum {
    MASTER_CTRL_ROLE_OFF = 0,
    MASTER_CTRL_ROLE_SLAVE,
    MASTER_CTRL_ROLE_MASTER,
    MASTER_CTRL_ROLE_COUNT,
} master_ctrl_role_t;

typedef enum {
    MASTER_CTRL_ROW_RATIO = 0,
    MASTER_CTRL_ROW_PHASE,
    MASTER_CTRL_ROW_COLOR,
    MASTER_CTRL_ROW_CONTROL,
    MASTER_CTRL_ROW_COUNT,
} master_ctrl_row_t;

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t kind;
    uint16_t reserved;
    uint32_t session_id;
    uint32_t token;
    int64_t t1_master_us;
} master_ctrl_ping_req_t;

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t kind;
    uint16_t reserved;
    uint32_t session_id;
    uint32_t token;
    int64_t t1_master_us;
    int64_t t2_slave_rx_us;
    int64_t t3_slave_tx_us;
} master_ctrl_ping_rsp_t;

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t kind;
    uint8_t mode;
    uint8_t flags;
    uint32_t session_id;
    uint16_t bpm_centi;
    uint8_t source_kind;
    uint8_t source_flags;
    uint8_t custom_speed_percent;
    uint8_t color_mode;
    uint16_t phase_offset_milli;
    uint8_t primary_r;
    uint8_t primary_g;
    uint8_t primary_b;
    uint8_t secondary_r;
    uint8_t secondary_g;
    uint8_t secondary_b;
    int64_t anchor_beat_us;
    uint32_t seq;
} master_ctrl_state_pkt_t;

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t kind;
    uint16_t reserved;
    uint32_t session_id;
} master_ctrl_release_pkt_t;

typedef struct {
    bool used;
    bool offset_valid;
    uint8_t mac[6];
    int64_t offset_us;   // slave clock ~= master clock + offset_us
    int64_t rtt_us;
    int64_t one_way_us;
    int64_t last_seen_us;
} master_ctrl_peer_t;

typedef struct {
    bool active;
    uint32_t session_id;
    float bpm;
    int mode;
    bool custom_active;
    uint8_t custom_speed_percent;
    uint8_t state_flags;
    master_ctrl_ratio_t bpm_ratio;
    master_ctrl_phase_t phase_mode;
    master_ctrl_color_mode_t color_mode;
    uint8_t primary_r;
    uint8_t primary_g;
    uint8_t primary_b;
    uint8_t secondary_r;
    uint8_t secondary_g;
    uint8_t secondary_b;
    int beat_flash_ticks;
    int64_t last_ping_us;
    int64_t last_state_us;
    int64_t last_source_next_beat_us;
    uint32_t token_seq;
    uint32_t state_seq;
    master_ctrl_peer_t peers[MASTER_CTRL_MAX_PEERS];
} master_ctrl_master_t;

typedef struct {
    bool beat_active;
    bool active;
    bool custom_active;
    uint8_t master_mac[6];
    uint32_t session_id;
    float bpm;
    int64_t period_us;
    int64_t target_period_us;
    int64_t sync_trim_us;
    int64_t queued_resync_beat_us;
    int64_t next_beat_us;
    int64_t last_update_us;
    uint8_t source_kind;
    uint8_t source_flags;
    float phase_offset;
    int mode;
    uint8_t custom_speed_percent;
    bool plane_bg;
    bool ring_bg;
    master_ctrl_color_mode_t color_mode;
    uint8_t primary_r;
    uint8_t primary_g;
    uint8_t primary_b;
    uint8_t secondary_r;
    uint8_t secondary_g;
    uint8_t secondary_b;
    int beat_flash_ticks;
} master_ctrl_slave_t;

typedef enum {
    MASTER_CTRL_SOURCE_NONE = 0,
    MASTER_CTRL_SOURCE_MANUAL_BPM,
    MASTER_CTRL_SOURCE_FFT_SYNC,
    MASTER_CTRL_SOURCE_CUSTOM,
} master_ctrl_source_kind_t;

typedef struct {
    master_ctrl_source_kind_t source;
    uint8_t source_flags;
    bool beat_active;
    bool custom_active;
    float bpm;
    float phase_offset;
    int64_t next_beat_us;
    int mode;
    uint8_t custom_speed_percent;
    bool plane_bg;
    bool ring_bg;
    uint8_t primary_r;
    uint8_t primary_g;
    uint8_t primary_b;
    uint8_t secondary_r;
    uint8_t secondary_g;
    uint8_t secondary_b;
} master_ctrl_source_state_t;

static master_ctrl_master_t s_master = {
    .active = false,
    .session_id = 0,
    .bpm = 0.0f,
    .mode = LED_BEAT_ANIM_FLASH,
    .custom_active = false,
    .custom_speed_percent = 100,
    .state_flags = 0,
    .bpm_ratio = MASTER_CTRL_RATIO_MATCH,
    .phase_mode = MASTER_CTRL_PHASE_ZERO,
    .color_mode = MASTER_CTRL_COLOR_FREE,
    .beat_flash_ticks = 0,
    .last_ping_us = 0,
    .last_state_us = 0,
    .token_seq = 0,
    .state_seq = 0,
};

static master_ctrl_slave_t s_slave = {0};
static master_ctrl_row_t s_selected_row = MASTER_CTRL_ROW_RATIO;
static master_ctrl_role_t s_role = MASTER_CTRL_ROLE_OFF;

static const uint8_t kBroadcastMac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

enum {
    MASTER_CTRL_TRON_MASTER_R = 0,
    MASTER_CTRL_TRON_MASTER_G = 196,
    MASTER_CTRL_TRON_MASTER_B = 255,
    MASTER_CTRL_TRON_MASTER_R2 = 0,
    MASTER_CTRL_TRON_MASTER_G2 = 92,
    MASTER_CTRL_TRON_MASTER_B2 = 255,
    MASTER_CTRL_TRON_SLAVE_R = 255,
    MASTER_CTRL_TRON_SLAVE_G = 112,
    MASTER_CTRL_TRON_SLAVE_B = 0,
    MASTER_CTRL_TRON_SLAVE_R2 = 255,
    MASTER_CTRL_TRON_SLAVE_G2 = 48,
    MASTER_CTRL_TRON_SLAVE_B2 = 0,
};

static inline void master_ctrl_fb_pset(uint8_t *fb, int x, int y)
{
    if (!fb) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void master_ctrl_fb_fill_rect(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            master_ctrl_fb_pset(fb, x, y);
        }
    }
}

static float master_ctrl_clamp_bpm(float bpm)
{
    if (bpm < 32.0f) bpm = 32.0f;
    if (bpm > 255.0f) bpm = 255.0f;
    return bpm;
}

static int64_t master_ctrl_clamp64(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t master_ctrl_custom_speed_clamp(uint8_t percent)
{
    if (percent < 10u) return 100u;
    if (percent > 250u) return 250u;
    return percent;
}

static const char *master_ctrl_source_name(master_ctrl_source_kind_t source)
{
    switch (source) {
        case MASTER_CTRL_SOURCE_MANUAL_BPM: return "manual";
        case MASTER_CTRL_SOURCE_FFT_SYNC:   return "fft";
        case MASTER_CTRL_SOURCE_CUSTOM:     return "custom";
        default:                            return "idle";
    }
}

static const char *master_ctrl_ratio_name(master_ctrl_ratio_t ratio)
{
    switch (ratio) {
        case MASTER_CTRL_RATIO_HALF:   return "1/2";
        case MASTER_CTRL_RATIO_DOUBLE: return "2/1";
        default:                       return "match";
    }
}

static const char *master_ctrl_phase_name(master_ctrl_phase_t phase)
{
    return (phase == MASTER_CTRL_PHASE_OFFBEAT) ? "offbeat" : "0";
}

static const char *master_ctrl_color_name(master_ctrl_color_mode_t mode)
{
    switch (mode) {
        case MASTER_CTRL_COLOR_TRON:  return "tron";
        case MASTER_CTRL_COLOR_MATCH: return "match";
        default:                      return "free";
    }
}

static const char *master_ctrl_role_name(master_ctrl_role_t role)
{
    switch (role) {
        case MASTER_CTRL_ROLE_MASTER: return "master";
        case MASTER_CTRL_ROLE_SLAVE:  return "slave";
        default:                      return "off";
    }
}

static bool master_ctrl_mac_equal(const uint8_t *a, const uint8_t *b)
{
    return a && b && memcmp(a, b, 6) == 0;
}

static bool master_ctrl_mode_supported(int mode)
{
    return mode >= 0 && mode < led_beat_anim_count();
}

static int master_ctrl_mode_clamp(int mode)
{
    return master_ctrl_mode_supported(mode) ? mode : (int)LED_BEAT_ANIM_FLASH;
}

static bool master_ctrl_custom_mode_supported(int mode)
{
    return mode >= 0 && mode < led_modes_count();
}

static int master_ctrl_custom_mode_clamp(int mode)
{
    return master_ctrl_custom_mode_supported(mode) ? mode : 0;
}

static const char *master_ctrl_led_mode_name(bool custom_active, int mode)
{
    return custom_active ? led_modes_name(master_ctrl_custom_mode_clamp(mode))
                         : led_beat_anim_name(master_ctrl_mode_clamp(mode));
}

static master_ctrl_ratio_t master_ctrl_ratio_cycle(master_ctrl_ratio_t ratio, int dir)
{
    int idx = ((int)ratio + (dir >= 0 ? 1 : -1) + MASTER_CTRL_RATIO_COUNT) % MASTER_CTRL_RATIO_COUNT;
    return (master_ctrl_ratio_t)idx;
}

static master_ctrl_phase_t master_ctrl_phase_cycle(master_ctrl_phase_t phase, int dir)
{
    int idx = ((int)phase + (dir >= 0 ? 1 : -1) + MASTER_CTRL_PHASE_COUNT) % MASTER_CTRL_PHASE_COUNT;
    return (master_ctrl_phase_t)idx;
}

static master_ctrl_color_mode_t master_ctrl_color_cycle(master_ctrl_color_mode_t mode, int dir)
{
    int idx = ((int)mode + (dir >= 0 ? 1 : -1) + MASTER_CTRL_COLOR_COUNT) % MASTER_CTRL_COLOR_COUNT;
    return (master_ctrl_color_mode_t)idx;
}

static master_ctrl_role_t master_ctrl_role_cycle(master_ctrl_role_t role, int dir)
{
    int idx = ((int)role + (dir >= 0 ? 1 : -1) + MASTER_CTRL_ROLE_COUNT) % MASTER_CTRL_ROLE_COUNT;
    return (master_ctrl_role_t)idx;
}

static float master_ctrl_ratio_apply(master_ctrl_ratio_t ratio, float bpm)
{
    switch (ratio) {
        case MASTER_CTRL_RATIO_HALF:   bpm *= 0.5f; break;
        case MASTER_CTRL_RATIO_DOUBLE: bpm *= 2.0f; break;
        default: break;
    }
    return master_ctrl_clamp_bpm(bpm);
}

static void master_ctrl_get_tron_colors(bool slave_side,
                                        uint8_t *pr, uint8_t *pg, uint8_t *pb,
                                        uint8_t *sr, uint8_t *sg, uint8_t *sb)
{
    if (slave_side) {
        if (pr) *pr = MASTER_CTRL_TRON_SLAVE_R;
        if (pg) *pg = MASTER_CTRL_TRON_SLAVE_G;
        if (pb) *pb = MASTER_CTRL_TRON_SLAVE_B;
        if (sr) *sr = MASTER_CTRL_TRON_SLAVE_R2;
        if (sg) *sg = MASTER_CTRL_TRON_SLAVE_G2;
        if (sb) *sb = MASTER_CTRL_TRON_SLAVE_B2;
    } else {
        if (pr) *pr = MASTER_CTRL_TRON_MASTER_R;
        if (pg) *pg = MASTER_CTRL_TRON_MASTER_G;
        if (pb) *pb = MASTER_CTRL_TRON_MASTER_B;
        if (sr) *sr = MASTER_CTRL_TRON_MASTER_R2;
        if (sg) *sg = MASTER_CTRL_TRON_MASTER_G2;
        if (sb) *sb = MASTER_CTRL_TRON_MASTER_B2;
    }
}

static void master_ctrl_set_local_colors(uint8_t pr, uint8_t pg, uint8_t pb,
                                         uint8_t sr, uint8_t sg, uint8_t sb)
{
    led_beat_color_set(pr, pg, pb);
    led_beat_secondary_color_set(sr, sg, sb);
    led_modes_set_primary_color(pr, pg, pb);
    led_modes_set_secondary_color(sr, sg, sb);
}

static bool master_ctrl_colors_match(uint8_t pr, uint8_t pg, uint8_t pb,
                                     uint8_t sr, uint8_t sg, uint8_t sb)
{
    uint8_t cur_pr = 0, cur_pg = 0, cur_pb = 0;
    uint8_t cur_sr = 0, cur_sg = 0, cur_sb = 0;
    led_beat_color_get(&cur_pr, &cur_pg, &cur_pb);
    led_beat_secondary_color_get(&cur_sr, &cur_sg, &cur_sb);
    return cur_pr == pr && cur_pg == pg && cur_pb == pb &&
           cur_sr == sr && cur_sg == sg && cur_sb == sb;
}

static void master_ctrl_apply_master_color_policy(void)
{
    if (!s_master.active || s_master.color_mode != MASTER_CTRL_COLOR_TRON) return;

    uint8_t pr = 0, pg = 0, pb = 0, sr = 0, sg = 0, sb = 0;
    master_ctrl_get_tron_colors(false, &pr, &pg, &pb, &sr, &sg, &sb);
    if (!master_ctrl_colors_match(pr, pg, pb, sr, sg, sb)) {
        master_ctrl_set_local_colors(pr, pg, pb, sr, sg, sb);
    }
}

static void master_ctrl_apply_slave_colors(master_ctrl_color_mode_t color_mode,
                                           uint8_t pr, uint8_t pg, uint8_t pb,
                                           uint8_t sr, uint8_t sg, uint8_t sb)
{
    if (color_mode == MASTER_CTRL_COLOR_FREE) {
        return;
    }
    if (color_mode == MASTER_CTRL_COLOR_TRON) {
        master_ctrl_get_tron_colors(true, &pr, &pg, &pb, &sr, &sg, &sb);
    }
    if (!master_ctrl_colors_match(pr, pg, pb, sr, sg, sb)) {
        master_ctrl_set_local_colors(pr, pg, pb, sr, sg, sb);
    }
}

static int master_ctrl_peer_count(void)
{
    int count = 0;
    for (size_t i = 0; i < MASTER_CTRL_MAX_PEERS; ++i) {
        if (s_master.peers[i].used && s_master.peers[i].offset_valid) {
            count++;
        }
    }
    return count;
}

static int master_ctrl_avg_rtt_ms(void)
{
    int64_t total_us = 0;
    int count = 0;
    for (size_t i = 0; i < MASTER_CTRL_MAX_PEERS; ++i) {
        if (s_master.peers[i].used && s_master.peers[i].offset_valid) {
            total_us += s_master.peers[i].rtt_us;
            count++;
        }
    }
    if (count <= 0) return 0;
    return (int)lround((double)total_us / (1000.0 * (double)count));
}

static master_ctrl_peer_t *master_ctrl_find_peer(const uint8_t *mac, bool create)
{
    if (!mac) return NULL;

    master_ctrl_peer_t *free_slot = NULL;
    master_ctrl_peer_t *oldest = NULL;
    for (size_t i = 0; i < MASTER_CTRL_MAX_PEERS; ++i) {
        master_ctrl_peer_t *peer = &s_master.peers[i];
        if (peer->used) {
            if (master_ctrl_mac_equal(peer->mac, mac)) {
                return peer;
            }
            if (!oldest || peer->last_seen_us < oldest->last_seen_us) {
                oldest = peer;
            }
        } else if (!free_slot) {
            free_slot = peer;
        }
    }

    if (!create) return NULL;
    master_ctrl_peer_t *slot = free_slot ? free_slot : oldest;
    if (!slot) return NULL;
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    memcpy(slot->mac, mac, 6);
    return slot;
}

static void master_ctrl_refresh_system_label(bool custom_active, int mode)
{
    const char *label = master_ctrl_led_mode_name(custom_active, mode);
    int mode_id = custom_active ? master_ctrl_custom_mode_clamp(mode)
                                : (100 + master_ctrl_mode_clamp(mode));
    system_state_set_led_mode(mode_id, label ? label : "master");
}

static void master_ctrl_force_remote_led_state(bool custom_active, int mode,
                                               uint8_t custom_speed_percent,
                                               bool plane_bg, bool ring_bg)
{
    fft_set_display_enabled(false);
    if (fft_visualizer_running()) {
        fft_visualizer_stop();
    }

    if (custom_active) {
        led_beat_enable(false);
        led_modes_plane_background_enable(plane_bg);
        led_modes_ring_background_enable(ring_bg);
        led_modes_set_speed_percent(master_ctrl_custom_speed_clamp(custom_speed_percent));
        led_modes_set(master_ctrl_custom_mode_clamp(mode));
        led_modes_enable(true);
    } else {
        led_modes_enable(false);
        led_beat_plane_background_enable(plane_bg);
        led_beat_ring_background_enable(ring_bg);
        led_beat_anim_set((led_beat_anim_t)master_ctrl_mode_clamp(mode));
        led_beat_enable(true);
    }

    master_ctrl_refresh_system_label(custom_active, mode);
}

static void master_ctrl_enforce_slave_override(void)
{
    fft_set_display_enabled(false);
    if (fft_visualizer_running()) {
        fft_visualizer_stop();
    }

    if (s_slave.custom_active) {
        led_beat_enable(false);
        led_modes_plane_background_enable(s_slave.plane_bg);
        led_modes_ring_background_enable(s_slave.ring_bg);
        led_modes_set_speed_percent(master_ctrl_custom_speed_clamp(s_slave.custom_speed_percent));
        led_modes_set(master_ctrl_custom_mode_clamp(s_slave.mode));
        led_modes_enable(true);
    } else {
        led_modes_enable(false);
        led_beat_plane_background_enable(s_slave.plane_bg);
        led_beat_ring_background_enable(s_slave.ring_bg);
        led_beat_anim_set((led_beat_anim_t)master_ctrl_mode_clamp(s_slave.mode));
        led_beat_enable(true);
    }

    master_ctrl_refresh_system_label(s_slave.custom_active, s_slave.mode);
}

static int64_t master_ctrl_period_us_from_bpm(float bpm)
{
    bpm = master_ctrl_clamp_bpm(bpm);
    return (int64_t)llround(60000000.0 / (double)bpm);
}

static int64_t master_ctrl_period_nudge_limit_us(int64_t period_us)
{
    int64_t limit = period_us / 96;
    return master_ctrl_clamp64(limit,
                               MASTER_CTRL_PERIOD_NUDGE_MIN_US,
                               MASTER_CTRL_PERIOD_NUDGE_MAX_US);
}

static int64_t master_ctrl_sync_trim_limit_us(int64_t period_us)
{
    int64_t limit = period_us / 32;
    return master_ctrl_clamp64(limit,
                               MASTER_CTRL_SYNC_TRIM_MIN_US,
                               MASTER_CTRL_SYNC_TRIM_MAX_US);
}

static int64_t master_ctrl_resync_limit_us(int64_t period_us)
{
    int64_t limit = (period_us * 3) / 2;
    return master_ctrl_clamp64(limit,
                               MASTER_CTRL_RESYNC_MIN_US,
                               MASTER_CTRL_RESYNC_MAX_US);
}

static int64_t master_ctrl_nudge_toward(int64_t current, int64_t target, int64_t max_step)
{
    int64_t delta = target - current;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static uint16_t master_ctrl_phase_offset_pack(float phase_offset)
{
    while (phase_offset < 0.0f) phase_offset += 1.0f;
    while (phase_offset >= 1.0f) phase_offset -= 1.0f;
    return (uint16_t)lroundf(phase_offset * 1000.0f);
}

static float master_ctrl_phase_offset_unpack(uint16_t phase_offset_milli)
{
    return ((float)phase_offset_milli) / 1000.0f;
}

static float master_ctrl_phase_to_next(float phase, float trigger_phase)
{
    float remaining = trigger_phase - phase;
    while (remaining <= 0.000001f) remaining += 1.0f;
    while (remaining > 1.0f) remaining -= 1.0f;
    return remaining;
}

static void master_ctrl_capture_source_state(int64_t now_us, master_ctrl_source_state_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    led_beat_color_get(&out->primary_r, &out->primary_g, &out->primary_b);
    led_beat_secondary_color_get(&out->secondary_r, &out->secondary_g, &out->secondary_b);

    if (led_modes_enabled()) {
        out->source = MASTER_CTRL_SOURCE_CUSTOM;
        out->custom_active = true;
        out->mode = master_ctrl_custom_mode_clamp(led_modes_current());
        out->custom_speed_percent = master_ctrl_custom_speed_clamp(led_modes_get_speed_percent());
        out->plane_bg = led_modes_plane_background_enabled();
        out->ring_bg = led_modes_ring_background_enabled();
        return;
    }

    out->mode = master_ctrl_mode_clamp((int)led_beat_anim_get());
    out->plane_bg = led_beat_plane_background_enabled();
    out->ring_bg = led_beat_ring_background_enabled();

    if (!led_beat_enabled()) {
        return;
    }

    manual_bpm_sync_state_t manual = {0};
    if (manual_bpm_get_sync_state(&manual) && manual.active && manual.bpm > 0.5f) {
        int64_t period_us = master_ctrl_period_us_from_bpm(manual.bpm);
        float remaining = master_ctrl_phase_to_next(manual.cycle_phase, manual.phase_offset);
        out->source = MASTER_CTRL_SOURCE_MANUAL_BPM;
        out->source_flags = 0;
        out->beat_active = true;
        out->bpm = master_ctrl_clamp_bpm(manual.bpm);
        out->phase_offset = manual.phase_offset;
        out->next_beat_us = now_us + (int64_t)llround((double)period_us * (double)remaining);
        return;
    }

    fft_sync_state_t fft_state = {0};
    fft_get_sync_state(&fft_state);
    if (fft_state.running && fft_state.bpm > 0.5f) {
        out->source = MASTER_CTRL_SOURCE_FFT_SYNC;
        out->source_flags = fft_state.bpm_locked ? MASTER_CTRL_SOURCE_FLAG_FFT_LOCKED : 0u;
        out->beat_active = fft_state.beat_enabled;
        out->bpm = master_ctrl_clamp_bpm(fft_state.bpm);
        out->phase_offset = fft_state.phase_offset;
        if (out->beat_active) {
            int64_t period_us = master_ctrl_period_us_from_bpm(out->bpm);
            float remaining = master_ctrl_phase_to_next(fft_state.beat_phase, fft_state.trigger_phase);
            out->next_beat_us = now_us + (int64_t)llround((double)period_us * (double)remaining);
        }
    }
}

static void master_ctrl_send_release(void)
{
    if (!s_master.active || link_active_path() == LINK_PATH_NONE) return;

    master_ctrl_release_pkt_t pkt = {
        .ver = MASTER_CTRL_PROTO_VER,
        .kind = MASTER_CTRL_PKT_RELEASE,
        .reserved = 0,
        .session_id = s_master.session_id,
    };
    (void)link_send_frame_to(kBroadcastMac, LINK_MSG_CONTROL,
                             (const uint8_t *)&pkt, sizeof(pkt), false);
}

static void master_ctrl_send_ping_req(int64_t now_us)
{
    if (!s_master.active || link_active_path() == LINK_PATH_NONE) return;

    master_ctrl_ping_req_t pkt = {
        .ver = MASTER_CTRL_PROTO_VER,
        .kind = MASTER_CTRL_PKT_PING_REQ,
        .reserved = 0,
        .session_id = s_master.session_id,
        .token = ++s_master.token_seq,
        .t1_master_us = now_us,
    };
    (void)link_send_frame_to(kBroadcastMac, LINK_MSG_CONTROL,
                             (const uint8_t *)&pkt, sizeof(pkt), false);
}

static void master_ctrl_send_state_updates(int64_t now_us, const master_ctrl_source_state_t *state)
{
    if (!s_master.active || link_active_path() == LINK_PATH_NONE || !state) return;

    uint8_t flags = 0;
    if (state->beat_active) flags |= MASTER_CTRL_STATE_FLAG_BEAT_ACTIVE;
    if (state->custom_active) flags |= MASTER_CTRL_STATE_FLAG_CUSTOM_ACTIVE;
    if (state->plane_bg) flags |= MASTER_CTRL_STATE_FLAG_PLANE_BG;
    if (state->ring_bg) flags |= MASTER_CTRL_STATE_FLAG_RING_BG;
    float target_bpm = 0.0f;
    int64_t period_us = 0;
    if (state->beat_active && state->bpm > 0.5f) {
        target_bpm = master_ctrl_ratio_apply(s_master.bpm_ratio, state->bpm);
        period_us = master_ctrl_period_us_from_bpm(target_bpm);
    }
    bool beat_clock_valid = state->beat_active && period_us > 0 && state->next_beat_us > 0;

    for (size_t i = 0; i < MASTER_CTRL_MAX_PEERS; ++i) {
        master_ctrl_peer_t *peer = &s_master.peers[i];
        if (!peer->used || !peer->offset_valid) continue;
        if ((now_us - peer->last_seen_us) > MASTER_CTRL_PEER_STALE_US) {
            memset(peer, 0, sizeof(*peer));
            continue;
        }

        int64_t lead_us = peer->one_way_us * 2 + 80000;
        if (lead_us < MASTER_CTRL_MIN_LEAD_US) lead_us = MASTER_CTRL_MIN_LEAD_US;
        if (lead_us > MASTER_CTRL_MAX_LEAD_US) lead_us = MASTER_CTRL_MAX_LEAD_US;

        int64_t anchor_master_us = state->next_beat_us;
        if (beat_clock_valid) {
            if (s_master.phase_mode == MASTER_CTRL_PHASE_OFFBEAT) {
                anchor_master_us += period_us / 2;
            }
            while ((anchor_master_us - now_us) < lead_us) {
                anchor_master_us += period_us;
            }
        }

        master_ctrl_state_pkt_t pkt = {
            .ver = MASTER_CTRL_PROTO_VER,
            .kind = MASTER_CTRL_PKT_STATE,
            .mode = (uint8_t)state->mode,
            .flags = flags,
            .session_id = s_master.session_id,
            .bpm_centi = (target_bpm > 0.5f) ? (uint16_t)lroundf(target_bpm * 100.0f) : 0u,
            .source_kind = (uint8_t)state->source,
            .source_flags = state->source_flags,
            .custom_speed_percent = state->custom_speed_percent,
            .color_mode = (uint8_t)s_master.color_mode,
            .phase_offset_milli = master_ctrl_phase_offset_pack(state->phase_offset),
            .primary_r = state->primary_r,
            .primary_g = state->primary_g,
            .primary_b = state->primary_b,
            .secondary_r = state->secondary_r,
            .secondary_g = state->secondary_g,
            .secondary_b = state->secondary_b,
            .anchor_beat_us = beat_clock_valid ? (anchor_master_us + peer->offset_us) : 0,
            .seq = ++s_master.state_seq,
        };

        (void)link_send_frame_to(peer->mac, LINK_MSG_CONTROL,
                                 (const uint8_t *)&pkt, sizeof(pkt), false);
    }
}

static void master_ctrl_clear_slave(void)
{
    memset(&s_slave, 0, sizeof(s_slave));
}

static void master_ctrl_trigger_local_beat(int *flash_ticks)
{
    led_trigger_beat(255, 96, 0);
    if (flash_ticks) {
        *flash_ticks = MASTER_CTRL_FLASH_TICKS;
    }
}

static void master_ctrl_begin_master_session(void)
{
    memset(s_master.peers, 0, sizeof(s_master.peers));
    s_master.session_id = esp_random();
    if (s_master.session_id == 0) {
        s_master.session_id = 1;
    }
    s_master.active = true;
    s_master.bpm = 0.0f;
    if (led_modes_enabled()) {
        s_master.mode = master_ctrl_custom_mode_clamp(led_modes_current());
        s_master.custom_active = true;
    } else {
        s_master.mode = master_ctrl_mode_clamp((int)led_beat_anim_get());
        s_master.custom_active = false;
    }
    s_master.custom_speed_percent = master_ctrl_custom_speed_clamp(led_modes_get_speed_percent());
    s_master.state_flags = 0;
    s_master.beat_flash_ticks = 0;
    s_master.last_ping_us = 0;
    s_master.last_state_us = 0;
    s_master.last_source_next_beat_us = 0;
    s_master.token_seq = 0;
    s_master.state_seq = 0;
    master_ctrl_clear_slave();
    master_ctrl_apply_master_color_policy();
}

static void master_ctrl_stop_master_session(void)
{
    if (!s_master.active) return;
    master_ctrl_send_release();
    s_master.active = false;
    memset(s_master.peers, 0, sizeof(s_master.peers));
    s_master.last_ping_us = 0;
    s_master.last_state_us = 0;
    s_master.last_source_next_beat_us = 0;
    s_master.beat_flash_ticks = 0;
    s_master.state_flags = 0;
}

static void master_ctrl_set_role(master_ctrl_role_t role)
{
    if (role == s_role) return;

    if (s_role == MASTER_CTRL_ROLE_MASTER) {
        master_ctrl_stop_master_session();
    }

    if (role == MASTER_CTRL_ROLE_MASTER) {
        s_role = role;
        master_ctrl_begin_master_session();
        return;
    }

    master_ctrl_clear_slave();
    s_role = role;
}

static void master_ctrl_master_tick(int64_t now_us)
{
    master_ctrl_apply_master_color_policy();
    master_ctrl_source_state_t state = {0};
    master_ctrl_capture_source_state(now_us, &state);
    uint8_t state_flags = 0;
    if (state.beat_active) state_flags |= MASTER_CTRL_STATE_FLAG_BEAT_ACTIVE;
    if (state.custom_active) state_flags |= MASTER_CTRL_STATE_FLAG_CUSTOM_ACTIVE;
    if (state.plane_bg) state_flags |= MASTER_CTRL_STATE_FLAG_PLANE_BG;
    if (state.ring_bg) state_flags |= MASTER_CTRL_STATE_FLAG_RING_BG;
    float bpm = (state.bpm > 0.5f) ? state.bpm : 0.0f;
    bool force_state_send = false;

    bool color_changed = false;
    if (s_master.color_mode == MASTER_CTRL_COLOR_MATCH) {
        color_changed = s_master.primary_r != state.primary_r ||
                        s_master.primary_g != state.primary_g ||
                        s_master.primary_b != state.primary_b ||
                        s_master.secondary_r != state.secondary_r ||
                        s_master.secondary_g != state.secondary_g ||
                        s_master.secondary_b != state.secondary_b;
    }

    if (s_master.custom_active != state.custom_active ||
        s_master.mode != state.mode ||
        s_master.custom_speed_percent != state.custom_speed_percent ||
        s_master.state_flags != state_flags ||
        fabsf(s_master.bpm - bpm) > 0.005f ||
        color_changed) {
        s_master.custom_active = state.custom_active;
        s_master.mode = state.mode;
        s_master.custom_speed_percent = state.custom_speed_percent;
        s_master.bpm = bpm;
        s_master.state_flags = state_flags;
        s_master.primary_r = state.primary_r;
        s_master.primary_g = state.primary_g;
        s_master.primary_b = state.primary_b;
        s_master.secondary_r = state.secondary_r;
        s_master.secondary_g = state.secondary_g;
        s_master.secondary_b = state.secondary_b;
        s_master.last_state_us = 0;
        force_state_send = true;
    }

    if (state.beat_active && state.next_beat_us > 0 && bpm > 0.5f) {
        int64_t source_period_us = master_ctrl_period_us_from_bpm(bpm);
        if (s_master.last_source_next_beat_us <= 0 ||
            state.next_beat_us > (s_master.last_source_next_beat_us + (source_period_us / 2))) {
            force_state_send = true;
        }
        s_master.last_source_next_beat_us = state.next_beat_us;
    } else {
        s_master.last_source_next_beat_us = 0;
    }

    if ((now_us - s_master.last_ping_us) >= MASTER_CTRL_PING_INTERVAL_US) {
        master_ctrl_send_ping_req(now_us);
        s_master.last_ping_us = now_us;
    }
    if (force_state_send || (now_us - s_master.last_state_us) >= MASTER_CTRL_STATE_INTERVAL_US) {
        master_ctrl_send_state_updates(now_us, &state);
        s_master.last_state_us = now_us;
    }
}

static void master_ctrl_slave_tick(int64_t now_us)
{
    if ((now_us - s_slave.last_update_us) > MASTER_CTRL_SLAVE_TIMEOUT_US) {
        master_ctrl_clear_slave();
        return;
    }

    bool need_force = fft_visualizer_running();
    if (s_slave.custom_active) {
        need_force = need_force ||
                     led_beat_enabled() ||
                     !led_modes_enabled() ||
                     master_ctrl_custom_mode_clamp(led_modes_current()) !=
                         master_ctrl_custom_mode_clamp(s_slave.mode) ||
                     master_ctrl_custom_speed_clamp(led_modes_get_speed_percent()) !=
                         master_ctrl_custom_speed_clamp(s_slave.custom_speed_percent) ||
                     led_modes_plane_background_enabled() != s_slave.plane_bg ||
                     led_modes_ring_background_enabled() != s_slave.ring_bg;
    } else {
        need_force = need_force ||
                     led_modes_enabled() ||
                     !led_beat_enabled() ||
                     master_ctrl_mode_clamp((int)led_beat_anim_get()) !=
                         master_ctrl_mode_clamp(s_slave.mode) ||
                     led_beat_plane_background_enabled() != s_slave.plane_bg ||
                     led_beat_ring_background_enabled() != s_slave.ring_bg;
    }

    if (need_force) {
        master_ctrl_force_remote_led_state(s_slave.custom_active, s_slave.mode,
                                           s_slave.custom_speed_percent,
                                           s_slave.plane_bg, s_slave.ring_bg);
    } else {
        master_ctrl_enforce_slave_override();
    }
    master_ctrl_apply_slave_colors(s_slave.color_mode,
                                   s_slave.primary_r, s_slave.primary_g, s_slave.primary_b,
                                   s_slave.secondary_r, s_slave.secondary_g, s_slave.secondary_b);

    if (s_slave.beat_flash_ticks > 0) {
        s_slave.beat_flash_ticks--;
    }

    if (!s_slave.beat_active || s_slave.period_us <= 0 || s_slave.next_beat_us <= 0) return;
    while (now_us >= s_slave.next_beat_us) {
        master_ctrl_trigger_local_beat(&s_slave.beat_flash_ticks);
        int64_t desired_period_us = s_slave.target_period_us;
        if (desired_period_us <= 0) {
            desired_period_us = s_slave.period_us;
        }
        desired_period_us += s_slave.sync_trim_us;

        if (s_slave.period_us != desired_period_us) {
            int64_t period_step = master_ctrl_period_nudge_limit_us(s_slave.period_us);
            s_slave.period_us = master_ctrl_nudge_toward(s_slave.period_us,
                                                         desired_period_us,
                                                         period_step);
        }
        if (s_slave.queued_resync_beat_us > 0) {
            int64_t resync_beat_us = s_slave.queued_resync_beat_us;
            int64_t resync_period_us = (s_slave.target_period_us > 0)
                ? s_slave.target_period_us
                : s_slave.period_us;
            if (resync_period_us <= 0) {
                resync_period_us = s_slave.period_us;
            }
            while (resync_beat_us <= now_us) {
                resync_beat_us += resync_period_us;
            }
            s_slave.next_beat_us = resync_beat_us;
            s_slave.queued_resync_beat_us = 0;
        } else {
            s_slave.next_beat_us += s_slave.period_us;
        }
    }
}

void master_control_service_tick(float dt_sec)
{
    (void)dt_sec;
    int64_t now_us = esp_timer_get_time();
    if (s_role == MASTER_CTRL_ROLE_MASTER && s_master.active) {
        master_ctrl_master_tick(now_us);
    } else if (s_role == MASTER_CTRL_ROLE_SLAVE && s_slave.active) {
        master_ctrl_slave_tick(now_us);
    }
}

bool master_control_service_blocks_app_tick(const char *app_id)
{
    if (!app_id) return false;
    if (s_role == MASTER_CTRL_ROLE_SLAVE && s_slave.active) {
        return strcmp(app_id, "manual_bpm") == 0;
    }
    return false;
}

system_master_ctrl_t master_control_hud_state(void)
{
    switch (s_role) {
        case MASTER_CTRL_ROLE_MASTER: return SYS_MASTER_CTRL_MASTER;
        case MASTER_CTRL_ROLE_SLAVE:  return SYS_MASTER_CTRL_SLAVE;
        default:                      return SYS_MASTER_CTRL_OFF;
    }
}

bool master_control_hud_link_active(void)
{
    int64_t now_us = esp_timer_get_time();
    if (s_role == MASTER_CTRL_ROLE_MASTER) {
        return master_ctrl_peer_count() > 0;
    }
    if (s_role == MASTER_CTRL_ROLE_SLAVE) {
        return s_slave.active &&
               ((now_us - s_slave.last_update_us) <= MASTER_CTRL_SLAVE_TIMEOUT_US);
    }
    return false;
}

static void master_ctrl_apply_slave_state(const uint8_t *src_mac,
                                          const master_ctrl_state_pkt_t *pkt,
                                          int64_t now_us)
{
    if (!src_mac || !pkt) return;
    bool beat_active = (pkt->flags & MASTER_CTRL_STATE_FLAG_BEAT_ACTIVE) != 0;
    bool custom_active = (pkt->flags & MASTER_CTRL_STATE_FLAG_CUSTOM_ACTIVE) != 0;
    bool plane_bg = (pkt->flags & MASTER_CTRL_STATE_FLAG_PLANE_BG) != 0;
    bool ring_bg = (pkt->flags & MASTER_CTRL_STATE_FLAG_RING_BG) != 0;
    int mode = custom_active
        ? master_ctrl_custom_mode_clamp((int)pkt->mode)
        : master_ctrl_mode_clamp((int)pkt->mode);
    uint8_t custom_speed_percent = master_ctrl_custom_speed_clamp(pkt->custom_speed_percent);
    float bpm = 0.0f;
    int64_t period_us = 0;
    if (pkt->bpm_centi > 0) {
        bpm = master_ctrl_clamp_bpm((float)pkt->bpm_centi / 100.0f);
        if (beat_active) {
            period_us = master_ctrl_period_us_from_bpm(bpm);
        }
    }
    int64_t anchor_us = pkt->anchor_beat_us;

    if (s_slave.active &&
        (now_us - s_slave.last_update_us) < MASTER_CTRL_SLAVE_TIMEOUT_US &&
        (!master_ctrl_mac_equal(s_slave.master_mac, src_mac) ||
         s_slave.session_id != pkt->session_id)) {
        return;
    }

    bool fresh_session = !s_slave.active ||
                         !master_ctrl_mac_equal(s_slave.master_mac, src_mac) ||
                         s_slave.session_id != pkt->session_id;
    bool state_changed = fresh_session ||
                         s_slave.custom_active != custom_active ||
                         s_slave.mode != mode ||
                         s_slave.beat_active != beat_active ||
                         s_slave.custom_speed_percent != custom_speed_percent ||
                         s_slave.plane_bg != plane_bg ||
                         s_slave.ring_bg != ring_bg;

    if (fresh_session) {
        memset(&s_slave, 0, sizeof(s_slave));
        memcpy(s_slave.master_mac, src_mac, 6);
        s_slave.session_id = pkt->session_id;
    }

    if (beat_active && period_us > 0 && anchor_us > 0) {
        while (anchor_us <= now_us) {
            anchor_us += period_us;
        }
        if (fresh_session || s_slave.next_beat_us <= 0 || !s_slave.beat_active) {
            s_slave.next_beat_us = anchor_us;
            s_slave.sync_trim_us = 0;
            s_slave.queued_resync_beat_us = 0;
        } else {
            int64_t delta_us = anchor_us - s_slave.next_beat_us;
            int64_t resync_limit_us = master_ctrl_resync_limit_us(period_us);
            if (delta_us > resync_limit_us || delta_us < -resync_limit_us) {
                s_slave.queued_resync_beat_us = anchor_us;
                s_slave.sync_trim_us = 0;
            } else if (delta_us != 0) {
                int64_t trim_limit_us = master_ctrl_sync_trim_limit_us(period_us);
                int64_t desired_trim_us =
                    master_ctrl_clamp64(delta_us / 16, -trim_limit_us, trim_limit_us);
                int64_t trim_step_us =
                    master_ctrl_clamp64(trim_limit_us / 4, 100, 1500);
                s_slave.sync_trim_us =
                    master_ctrl_nudge_toward(s_slave.sync_trim_us,
                                             desired_trim_us,
                                             trim_step_us);
            }
        }
    } else {
        s_slave.next_beat_us = 0;
        s_slave.sync_trim_us = 0;
        s_slave.queued_resync_beat_us = 0;
    }

    s_slave.active = true;
    s_slave.bpm = bpm;
    s_slave.source_kind = (pkt->source_kind <= MASTER_CTRL_SOURCE_CUSTOM)
        ? pkt->source_kind
        : (uint8_t)MASTER_CTRL_SOURCE_NONE;
    s_slave.source_flags = pkt->source_flags;
    s_slave.phase_offset = master_ctrl_phase_offset_unpack(pkt->phase_offset_milli);
    if (beat_active && period_us > 0) {
        s_slave.target_period_us = period_us;
        if (fresh_session || s_slave.period_us <= 0) {
            s_slave.period_us = period_us;
        }
    } else {
        s_slave.period_us = 0;
        s_slave.target_period_us = 0;
    }
    s_slave.mode = mode;
    s_slave.custom_active = custom_active;
    s_slave.beat_active = beat_active;
    s_slave.custom_speed_percent = custom_speed_percent;
    s_slave.plane_bg = plane_bg;
    s_slave.ring_bg = ring_bg;
    s_slave.color_mode = (pkt->color_mode < MASTER_CTRL_COLOR_COUNT)
        ? (master_ctrl_color_mode_t)pkt->color_mode
        : MASTER_CTRL_COLOR_FREE;
    if (s_slave.color_mode == MASTER_CTRL_COLOR_TRON) {
        master_ctrl_get_tron_colors(true,
                                    &s_slave.primary_r, &s_slave.primary_g, &s_slave.primary_b,
                                    &s_slave.secondary_r, &s_slave.secondary_g, &s_slave.secondary_b);
    } else {
        s_slave.primary_r = pkt->primary_r;
        s_slave.primary_g = pkt->primary_g;
        s_slave.primary_b = pkt->primary_b;
        s_slave.secondary_r = pkt->secondary_r;
        s_slave.secondary_g = pkt->secondary_g;
        s_slave.secondary_b = pkt->secondary_b;
    }

    if (state_changed) {
        master_ctrl_force_remote_led_state(s_slave.custom_active, s_slave.mode,
                                           s_slave.custom_speed_percent,
                                           s_slave.plane_bg, s_slave.ring_bg);
    } else {
        master_ctrl_enforce_slave_override();
    }
    master_ctrl_apply_slave_colors(s_slave.color_mode,
                                   s_slave.primary_r, s_slave.primary_g, s_slave.primary_b,
                                   s_slave.secondary_r, s_slave.secondary_g, s_slave.secondary_b);
    s_slave.last_update_us = now_us;
}

void master_control_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac,
                                      const uint8_t *payload, size_t len)
{
    if (type != LINK_MSG_CONTROL || !payload || len < 2) return;

    uint8_t ver = payload[0];
    uint8_t kind = payload[1];
    if (ver != MASTER_CTRL_PROTO_VER) return;
    if (s_role == MASTER_CTRL_ROLE_OFF) return;

    int64_t now_us = esp_timer_get_time();

    if (kind == MASTER_CTRL_PKT_PING_REQ) {
        if (s_role != MASTER_CTRL_ROLE_SLAVE || !src_mac || len < sizeof(master_ctrl_ping_req_t)) return;
        const master_ctrl_ping_req_t *req = (const master_ctrl_ping_req_t *)payload;
        master_ctrl_ping_rsp_t rsp = {
            .ver = MASTER_CTRL_PROTO_VER,
            .kind = MASTER_CTRL_PKT_PING_RESP,
            .reserved = 0,
            .session_id = req->session_id,
            .token = req->token,
            .t1_master_us = req->t1_master_us,
            .t2_slave_rx_us = now_us,
            .t3_slave_tx_us = esp_timer_get_time(),
        };
        (void)link_send_frame_to(src_mac, LINK_MSG_CONTROL,
                                 (const uint8_t *)&rsp, sizeof(rsp), false);
        return;
    }

    if (kind == MASTER_CTRL_PKT_PING_RESP) {
        if (s_role != MASTER_CTRL_ROLE_MASTER || !s_master.active ||
            !src_mac || len < sizeof(master_ctrl_ping_rsp_t)) return;
        const master_ctrl_ping_rsp_t *rsp = (const master_ctrl_ping_rsp_t *)payload;
        if (rsp->session_id != s_master.session_id) return;

        master_ctrl_peer_t *peer = master_ctrl_find_peer(src_mac, true);
        if (!peer) return;

        int64_t t1 = rsp->t1_master_us;
        int64_t t2 = rsp->t2_slave_rx_us;
        int64_t t3 = rsp->t3_slave_tx_us;
        int64_t t4 = now_us;
        int64_t rtt_us = (t4 - t1) - (t3 - t2);
        if (rtt_us < 0) rtt_us = 0;
        int64_t offset_us = ((t2 - t1) + (t3 - t4)) / 2;

        if (peer->offset_valid) {
            peer->offset_us = (peer->offset_us * 3 + offset_us) / 4;
            peer->rtt_us = (peer->rtt_us * 3 + rtt_us) / 4;
            peer->one_way_us = (peer->one_way_us * 3 + (rtt_us / 2)) / 4;
        } else {
            peer->offset_us = offset_us;
            peer->rtt_us = rtt_us;
            peer->one_way_us = rtt_us / 2;
            peer->offset_valid = true;
        }
        peer->last_seen_us = now_us;
        s_master.last_state_us = 0;
        return;
    }

    if (kind == MASTER_CTRL_PKT_STATE) {
        if (s_role != MASTER_CTRL_ROLE_SLAVE || len < sizeof(master_ctrl_state_pkt_t)) return;
        const master_ctrl_state_pkt_t *pkt = (const master_ctrl_state_pkt_t *)payload;
        master_ctrl_apply_slave_state(src_mac, pkt, now_us);
        return;
    }

    if (kind == MASTER_CTRL_PKT_RELEASE) {
        if (s_role != MASTER_CTRL_ROLE_SLAVE || len < sizeof(master_ctrl_release_pkt_t)) return;
        const master_ctrl_release_pkt_t *pkt = (const master_ctrl_release_pkt_t *)payload;
        if (s_slave.active &&
            master_ctrl_mac_equal(s_slave.master_mac, src_mac) &&
            s_slave.session_id == pkt->session_id) {
            master_ctrl_clear_slave();
        }
    }
}

void master_control_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
}

void master_control_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
}

void master_control_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A && s_selected_row > MASTER_CTRL_ROW_RATIO) {
        s_selected_row = (master_ctrl_row_t)(s_selected_row - 1);
        return;
    }
    if (ev->button == INPUT_BTN_B && s_selected_row + 1 < MASTER_CTRL_ROW_COUNT) {
        s_selected_row = (master_ctrl_row_t)(s_selected_row + 1);
        return;
    }
    if (ev->button != INPUT_BTN_C && ev->button != INPUT_BTN_D) {
        return;
    }

    int dir = (ev->button == INPUT_BTN_D) ? +1 : -1;
    switch (s_selected_row) {
        case MASTER_CTRL_ROW_RATIO:
            s_master.bpm_ratio = master_ctrl_ratio_cycle(s_master.bpm_ratio, dir);
            if (s_master.active) s_master.last_state_us = 0;
            break;
        case MASTER_CTRL_ROW_PHASE:
            s_master.phase_mode = master_ctrl_phase_cycle(s_master.phase_mode, dir);
            if (s_master.active) s_master.last_state_us = 0;
            break;
        case MASTER_CTRL_ROW_COLOR:
            s_master.color_mode = master_ctrl_color_cycle(s_master.color_mode, dir);
            if (s_master.active) {
                master_ctrl_apply_master_color_policy();
                s_master.last_state_us = 0;
            }
            break;
        case MASTER_CTRL_ROW_CONTROL:
            master_ctrl_set_role(master_ctrl_role_cycle(s_role, dir));
            break;
        default:
            break;
    }
}

void master_control_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)h;
    if (!fb) return;

    char line[32];
    master_ctrl_source_state_t state = {0};
    if (s_role == MASTER_CTRL_ROLE_MASTER && s_master.active) {
        master_ctrl_capture_source_state(esp_timer_get_time(), &state);
    }
    snprintf(line, sizeof(line), "MASTER %s", master_ctrl_role_name(s_role));
    oled_draw_text3x5(fb, x + 2, y + 2, line);

    if (s_role == MASTER_CTRL_ROLE_MASTER) {
        snprintf(line, sizeof(line), "%s %up %dms",
                 master_ctrl_source_name(state.source),
                 (unsigned)master_ctrl_peer_count(),
                 master_ctrl_avg_rtt_ms());
    } else if (s_role == MASTER_CTRL_ROLE_SLAVE) {
        int age_ms = s_slave.active ? (int)((esp_timer_get_time() - s_slave.last_update_us) / 1000LL) : -1;
        if (s_slave.active && s_slave.bpm > 0.5f) {
            snprintf(line, sizeof(line), "sync %3u %dms",
                     (unsigned)lroundf(s_slave.bpm),
                     (age_ms >= 0) ? age_ms : 0);
        } else {
            snprintf(line, sizeof(line), "sync waiting");
        }
    } else {
        snprintf(line, sizeof(line), "sync disabled");
    }
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "%cratio:%s",
             s_selected_row == MASTER_CTRL_ROW_RATIO ? '>' : ' ',
             master_ctrl_ratio_name(s_master.bpm_ratio));
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "%cphase:%s",
             s_selected_row == MASTER_CTRL_ROW_PHASE ? '>' : ' ',
             master_ctrl_phase_name(s_master.phase_mode));
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    snprintf(line, sizeof(line), "%ccolor:%s",
             s_selected_row == MASTER_CTRL_ROW_COLOR ? '>' : ' ',
             master_ctrl_color_name(s_master.color_mode));
    oled_draw_text3x5(fb, x + 2, y + 34, line);

    snprintf(line, sizeof(line), "%ccontrol:%s",
             s_selected_row == MASTER_CTRL_ROW_CONTROL ? '>' : ' ',
             master_ctrl_role_name(s_role));
    oled_draw_text3x5(fb, x + 2, y + 42, line);

    if (s_master.beat_flash_ticks > 0) {
        master_ctrl_fb_fill_rect(fb, x + w - 8, y + 2, 5, 5);
    } else if (s_role == MASTER_CTRL_ROLE_SLAVE && s_slave.beat_flash_ticks > 0) {
        master_ctrl_fb_fill_rect(fb, x + w - 8, y + 2, 5, 5);
    }
}
