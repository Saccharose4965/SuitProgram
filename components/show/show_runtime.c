#include "show_runtime.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "led.h"
#include "led_layout.h"
#include "led_modes.h"
#include "show_package.h"
#include "show_renderer.h"
#include "audio_player.h"
#include "audio.h"

#define SHOW_SYNC_PROTOCOL       0x52u
#define SHOW_SYNC_QUEUE_DEPTH    12u
#define SHOW_SYNC_MAX_PACKET     112u
#define SHOW_SYNC_MAX_PEERS      8u
#define SHOW_SYNC_TRANSPORT_US   250000LL
#define SHOW_SYNC_PING_US        1000000LL
#define SHOW_SYNC_START_LEAD_US  500000LL
#define SHOW_SYNC_STALE_US       2000000LL
#define SHOW_SYNC_MASTER_HOLD_US 5000000LL
#define SHOW_RENDER_FPS          60u

typedef enum {
    SHOW_WIRE_PING_REQUEST = 1,
    SHOW_WIRE_PING_RESPONSE,
    SHOW_WIRE_TRANSPORT,
    SHOW_WIRE_RELEASE,
    SHOW_WIRE_HELLO,
    SHOW_WIRE_QUERY,
    SHOW_WIRE_ROLES,
} show_wire_kind_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol, kind;
    uint16_t bytes;
    uint8_t role, authority, active;
    uint8_t ready, wanted;
    uint32_t boot, request;
} show_wire_hello_t;

typedef struct __attribute__((packed)) {
    uint8_t mac[6], role;
    uint32_t boot, request;
} show_role_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol, kind;
    uint16_t bytes;
    uint32_t boot, revision;
    uint8_t count;
    show_role_entry_t entries[SHOW_MAX_ROLES];
} show_wire_roles_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol;
    uint8_t kind;
    uint16_t bytes;
    uint32_t session_id;
    uint32_t sequence;
    int64_t master_send_us;
} show_wire_ping_request_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol;
    uint8_t kind;
    uint16_t bytes;
    uint32_t session_id;
    uint32_t sequence;
    int64_t master_send_us;
    int64_t follower_receive_us;
    int64_t follower_send_us;
} show_wire_ping_response_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol;
    uint8_t kind;
    uint16_t bytes;
    uint32_t session_id;
    uint32_t generation;
    uint64_t show_uid;
    uint32_t duration_ms;
    uint32_t playhead_ms;
    int64_t target_anchor_us;
    int32_t anchor_delay_us;
    uint8_t state;
    uint8_t role_count;
    uint16_t fps_hint;
    char slug[32];
} show_wire_transport_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol;
    uint8_t kind;
    uint16_t bytes;
    uint32_t session_id;
    uint32_t generation;
    uint64_t show_uid;
} show_wire_release_t;

typedef struct {
    uint8_t src_mac[6];
    size_t len;
    uint8_t data[SHOW_SYNC_MAX_PACKET];
    int64_t received_us;
} show_rx_event_t;

typedef struct {
    bool used;
    uint8_t mac[6];
    int64_t offset_us; /* follower clock - master clock */
    uint32_t rtt_us;
    int64_t updated_us;
    int64_t seen_us;
    uint8_t role;
    bool authority;
    bool role_ready;
    uint8_t wanted;
    uint32_t boot, request;
} show_peer_t;

typedef struct {
    bool initialized;
    bool menu_open;
    bool role_ready;
    uint8_t wanted_role;
    uint32_t boot, role_request;
    int64_t discover_until_us, last_roles_us, role_reload_after_us;
    uint8_t coordinator[6];
    show_wire_roles_t roles;
    bool audio_owned;
    bool audio_started;
    uint32_t audio_position_ms;
    int64_t audio_request_us;
    uint16_t bt_delay_ms;
    uint8_t own_mac[6];
    int64_t last_hello_us;
    bool program_loaded;
    show_runtime_state_t state;
    bool authority;
    bool synced;
    bool output_owned;
    bool restore_modes;
    bool restore_beat;
    bool restore_audio_brightness;
    uint8_t role_id;
    char slug[32];
    char error[32];
    show_package_program_t program;
    show_renderer_t *renderer;
    QueueHandle_t rx_queue;
    uint8_t frame[LED_STRIP_LENGTH * 3u];
    size_t frame_pixels;

    uint32_t session_id;
    uint32_t generation;
    uint32_t sequence;
    uint32_t anchor_playhead_ms;
    int64_t anchor_local_us;
    uint32_t paused_playhead_ms;
    int64_t last_render_us;
    int64_t render_not_before_us;
    uint32_t render_budget_us;
    bool frame_valid;
    uint32_t frame_playhead_ms;
    uint8_t frame_brightness;
    int64_t last_transport_tx_us;
    int64_t last_ping_tx_us;
    int64_t last_transport_rx_us;
    uint8_t master_mac[6];
    bool have_master;
    show_peer_t peers[SHOW_SYNC_MAX_PEERS];
} show_runtime_t;

static show_runtime_t s_runtime;
static const uint8_t broadcast_mac[6] = {255,255,255,255,255,255};
static void service_roles(int64_t now_us);
static esp_err_t load_program(const char *show_slug, uint8_t role_id);

static void send_hello(void)
{
    show_wire_hello_t p = {SHOW_SYNC_PROTOCOL, SHOW_WIRE_HELLO, sizeof(p),
        s_runtime.role_id, s_runtime.authority,
        s_runtime.menu_open || s_runtime.output_owned,
        s_runtime.role_ready, s_runtime.wanted_role,
        s_runtime.boot, s_runtime.role_request};
    (void)link_send_frame_to(broadcast_mac, LINK_MSG_CONTROL, (const uint8_t *)&p, sizeof(p), false);
}

void show_runtime_set_menu_open(bool open)
{
    if (show_runtime_init() != ESP_OK) return;
    if (open && !s_runtime.menu_open && !s_runtime.output_owned) {
        s_runtime.role_ready = false;
        s_runtime.wanted_role = UINT8_MAX;
        s_runtime.discover_until_us = esp_timer_get_time() + 700000;
        s_runtime.roles.count = 0;
        uint8_t query[] = {SHOW_SYNC_PROTOCOL, SHOW_WIRE_QUERY, 4, 0};
        (void)link_send_frame_to(broadcast_mac, LINK_MSG_CONTROL, query, sizeof(query), false);
    }
    s_runtime.menu_open = open;
    send_hello();
}

void show_runtime_set_bt_delay_ms(uint16_t delay_ms)
{ s_runtime.bt_delay_ms = delay_ms > 1000 ? 1000 : delay_ms; }
uint16_t show_runtime_get_bt_delay_ms(void) { return s_runtime.bt_delay_ms; }

static void set_error(const char *message)
{
    snprintf(s_runtime.error, sizeof(s_runtime.error), "%s", message ? message : "");
}

static bool same_mac(const uint8_t a[6], const uint8_t b[6])
{
    return a && b && memcmp(a, b, 6) == 0;
}

static uint32_t current_playhead_ms(int64_t now_us)
{
    if (s_runtime.state == SHOW_RUNTIME_PAUSED) return s_runtime.paused_playhead_ms;
    if (s_runtime.state != SHOW_RUNTIME_PLAYING) return 0;
    if (now_us <= s_runtime.anchor_local_us) return s_runtime.anchor_playhead_ms;

    uint64_t elapsed_ms = (uint64_t)(now_us - s_runtime.anchor_local_us) / 1000u;
    uint64_t playhead = (uint64_t)s_runtime.anchor_playhead_ms + elapsed_ms;
    if (playhead > s_runtime.program.info.duration_ms) {
        playhead = s_runtime.program.info.duration_ms;
    }
    return (uint32_t)playhead;
}

// Diagnostic A/B switch, read once when a show claims output, never per frame.
static bool s_diagnostic_no_leds;

static esp_err_t submit_show_frame(size_t pixels)
{
    if (s_diagnostic_no_leds) return ESP_OK;
    return led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame, pixels);
}

static esp_err_t acquire_output(void)
{
    if (s_runtime.output_owned) return ESP_OK;
    esp_err_t err = led_init();
    if (err != ESP_OK) return err;

    s_runtime.restore_modes = led_modes_enabled();
    s_runtime.restore_beat = led_beat_enabled();
    s_runtime.restore_audio_brightness = led_audio_brightness_enabled();
    err = led_output_claim(LED_OUTPUT_OWNER_SHOW);
    if (err != ESP_OK) return err;

    s_runtime.output_owned = true;
    s_runtime.frame_valid = false;
    s_runtime.last_render_us = 0;
    s_runtime.render_not_before_us = 0;
    s_runtime.render_budget_us = 0;
    led_modes_enable(false);
    led_beat_enable(false);
    led_audio_brightness_enable(false);
    FILE *marker = fopen("/sdcard/show-no-leds", "rb");
    s_diagnostic_no_leds = marker != NULL;
    if (marker) fclose(marker);
    ESP_LOGI("show_runtime", "diagnostic: LED output %s (rendering and sync remain active)",
             s_diagnostic_no_leds ? "DISABLED by /sdcard/show-no-leds" : "enabled");
    if (s_diagnostic_no_leds) {
        // One blank frame; no recurring physical submissions during this run.
        memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
        (void)led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame,
                                   s_runtime.frame_pixels);
    }
    return ESP_OK;
}

static void release_output(void)
{
    if (!s_runtime.output_owned) return;
    memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
    (void)led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame,
                                s_runtime.frame_pixels);
    led_output_release(LED_OUTPUT_OWNER_SHOW);
    s_runtime.output_owned = false;
    led_modes_enable(s_runtime.restore_modes);
    led_beat_enable(s_runtime.restore_beat);
    led_audio_brightness_enable(s_runtime.restore_audio_brightness);
}

static show_peer_t *find_peer(const uint8_t mac[6], bool create)
{
    if (!mac) return NULL;
    show_peer_t *free_slot = NULL;
    show_peer_t *oldest = NULL;
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        show_peer_t *peer = &s_runtime.peers[i];
        if (peer->used && same_mac(peer->mac, mac)) return peer;
        if (!peer->used && !free_slot) free_slot = peer;
        if (peer->used && (!oldest || peer->updated_us < oldest->updated_us)) oldest = peer;
    }
    if (!create) return NULL;
    show_peer_t *peer = free_slot ? free_slot : oldest;
    if (!peer) return NULL;
    memset(peer, 0, sizeof(*peer));
    peer->used = true;
    memcpy(peer->mac, mac, 6);
    return peer;
}

static void send_ping(int64_t now_us)
{
    show_wire_ping_request_t packet = {
        .protocol = SHOW_SYNC_PROTOCOL,
        .kind = SHOW_WIRE_PING_REQUEST,
        .bytes = sizeof(packet),
        .session_id = s_runtime.session_id,
        .sequence = ++s_runtime.sequence,
        .master_send_us = now_us,
    };
    (void)link_send_frame_to(broadcast_mac, LINK_MSG_CONTROL, (const uint8_t *)&packet,
                          sizeof(packet), false);
}

static void send_transport_to(const uint8_t *peer_mac, int64_t target_anchor_us,
                              int64_t now_us)
{
    int64_t anchor_delay_us = s_runtime.anchor_local_us - now_us;
    if (s_runtime.state != SHOW_RUNTIME_PLAYING || anchor_delay_us < 0) {
        anchor_delay_us = 0;
    }
    if (anchor_delay_us > INT32_MAX) anchor_delay_us = INT32_MAX;
    show_wire_transport_t packet = {
        .protocol = SHOW_SYNC_PROTOCOL,
        .kind = SHOW_WIRE_TRANSPORT,
        .bytes = sizeof(packet),
        .session_id = s_runtime.session_id,
        .generation = s_runtime.generation,
        .show_uid = s_runtime.program.info.show_uid,
        .duration_ms = s_runtime.program.info.duration_ms,
        .playhead_ms = s_runtime.state == SHOW_RUNTIME_PAUSED
            ? s_runtime.paused_playhead_ms
            : (target_anchor_us != 0
                ? s_runtime.anchor_playhead_ms : current_playhead_ms(now_us)),
        .target_anchor_us = target_anchor_us,
        .anchor_delay_us = (int32_t)anchor_delay_us,
        .state = (uint8_t)s_runtime.state,
        .role_count = (uint8_t)s_runtime.program.info.role_count,
        .fps_hint = s_runtime.program.info.fps_hint,
    };
    snprintf(packet.slug, sizeof(packet.slug), "%s", s_runtime.slug);
    (void)link_send_frame_to(peer_mac, LINK_MSG_CONTROL,
                             (const uint8_t *)&packet, sizeof(packet), false);
}

static bool have_listeners(int64_t now_us)
{
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        const show_peer_t *p = &s_runtime.peers[i];
        if (p->used && p->seen_us && now_us - p->seen_us <= SHOW_SYNC_STALE_US) return true;
    }
    return false;
}

static void broadcast_transport(int64_t now_us)
{
    if (!have_listeners(now_us)) return;
    send_transport_to(broadcast_mac, 0, now_us);
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        show_peer_t *peer = &s_runtime.peers[i];
        if (!peer->used || !peer->seen_us || now_us - peer->seen_us > SHOW_SYNC_STALE_US ||
            !peer->updated_us || now_us - peer->updated_us > SHOW_SYNC_STALE_US) continue;
        send_transport_to(peer->mac, s_runtime.anchor_local_us + peer->offset_us, now_us);
    }
}

static void send_release(void)
{
    show_wire_release_t packet = {
        .protocol = SHOW_SYNC_PROTOCOL,
        .kind = SHOW_WIRE_RELEASE,
        .bytes = sizeof(packet),
        .session_id = s_runtime.session_id,
        .generation = s_runtime.generation,
        .show_uid = s_runtime.program.info.show_uid,
    };
    (void)link_send_frame_to(broadcast_mac, LINK_MSG_CONTROL, (const uint8_t *)&packet,
                          sizeof(packet), false);
}

static bool packet_header_valid(const uint8_t *data, size_t len, uint8_t kind,
                                size_t expected)
{
    if (!data || len != expected || len < 4 || data[0] != SHOW_SYNC_PROTOCOL ||
        data[1] != kind) return false;
    uint16_t bytes = 0;
    memcpy(&bytes, data + 2, sizeof(bytes));
    return bytes == expected;
}

static bool peer_present(const show_peer_t *p, int64_t now_us)
{
    return p->used && p->seen_us && now_us - p->seen_us <= SHOW_SYNC_STALE_US;
}

static const uint8_t *role_coordinator(int64_t now_us)
{
    const uint8_t *mac = s_runtime.own_mac;
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i)
        if (peer_present(&s_runtime.peers[i], now_us) &&
            memcmp(s_runtime.peers[i].mac, mac, 6) < 0) mac = s_runtime.peers[i].mac;
    return mac;
}

static void apply_roles(const show_wire_roles_t *map)
{
    s_runtime.roles = *map;
    for (unsigned i = 0; i < map->count; ++i) {
        const show_role_entry_t *e = &map->entries[i];
        if (same_mac(e->mac, s_runtime.own_mac) && e->boot == s_runtime.boot) {
            bool changed = !s_runtime.role_ready || s_runtime.role_id != e->role;
            s_runtime.role_id = e->role;
            s_runtime.role_ready = true;
            if (e->request == s_runtime.role_request) s_runtime.wanted_role = UINT8_MAX;
            if (changed) {
                s_runtime.frame_valid = false;
                s_runtime.role_reload_after_us = 0;
                send_hello();
            }
        }
    }
}

// One small table, one coordinator; playback leadership is entirely separate.
// Repeated HELLO requests and periodic snapshots tolerate lost radio packets.
static void service_roles(int64_t now_us)
{
    if ((!s_runtime.menu_open && !s_runtime.output_owned) ||
        now_us < s_runtime.discover_until_us) return;
    const uint8_t *coordinator = role_coordinator(now_us);
    if (!same_mac(coordinator, s_runtime.coordinator)) {
        memcpy(s_runtime.coordinator, coordinator, 6);
        s_runtime.roles.revision = 0;
    }
    if (!same_mac(coordinator, s_runtime.own_mac)) return;
    show_wire_roles_t next = {.protocol=SHOW_SYNC_PROTOCOL, .kind=SHOW_WIRE_ROLES,
        .bytes=sizeof(next), .boot=s_runtime.boot, .revision=s_runtime.roles.revision};
    uint8_t wanted[SHOW_MAX_ROLES];
    uint32_t requests[SHOW_MAX_ROLES];
    // Established claims first, then newcomers. Joining never displaces a role.
    for (unsigned pass=0; pass<2; ++pass) {
        for (size_t i=0; i<=SHOW_SYNC_MAX_PEERS && next.count<SHOW_MAX_ROLES; ++i) {
            show_peer_t *p = i ? &s_runtime.peers[i-1] : NULL;
            if (p && !peer_present(p, now_us)) continue;
            bool ready = p ? p->role_ready : s_runtime.role_ready;
            if (ready != (pass==0)) continue;
            show_role_entry_t *e = &next.entries[next.count];
            memcpy(e->mac, p ? p->mac : s_runtime.own_mac, 6);
            e->boot = p ? p->boot : s_runtime.boot;
            e->role = ready ? (p ? p->role : s_runtime.role_id) : UINT8_MAX;
            for (unsigned j=0; j<s_runtime.roles.count; ++j) {
                const show_role_entry_t *old = &s_runtime.roles.entries[j];
                if (same_mac(old->mac,e->mac) && old->boot==e->boot) { *e=*old; break; }
            }
            wanted[next.count] = p ? p->wanted : s_runtime.wanted_role;
            requests[next.count] = p ? p->request : s_runtime.role_request;
            ++next.count;
        }
    }
    unsigned used=0;
    for (unsigned i=0; i<next.count; ++i) {
        unsigned role=next.entries[i].role;
        if (role<SHOW_MAX_ROLES && !(used & (1u<<role))) used |= 1u<<role;
        else next.entries[i].role=UINT8_MAX;
    }
    for (unsigned i=0; i<next.count; ++i) {
        show_role_entry_t *e=&next.entries[i];
        if (e->role==UINT8_MAX) {
            for (unsigned role=0; role<SHOW_MAX_ROLES; ++role)
                if (!(used & (1u<<role))) { e->role=role; used|=1u<<role; break; }
        }
    }
    for (unsigned i=0; i<next.count; ++i) {
        show_role_entry_t *e=&next.entries[i];
        if (requests[i]==e->request || (int32_t)(requests[i]-e->request)<0) continue;
        if (wanted[i]<SHOW_MAX_ROLES && wanted[i]!=e->role) {
            for (unsigned j=0; j<next.count; ++j)
                if (j!=i && next.entries[j].role==wanted[i]) next.entries[j].role=e->role;
            e->role=wanted[i];
        }
        e->request=requests[i];
    }
    bool changed = next.count!=s_runtime.roles.count ||
        memcmp(next.entries,s_runtime.roles.entries,sizeof(next.entries))!=0;
    if (!changed && now_us-s_runtime.last_roles_us<500000) return;
    ++next.revision;
    apply_roles(&next);
    s_runtime.last_roles_us=now_us;
    (void)link_send_frame_to(broadcast_mac, LINK_MSG_CONTROL, (const uint8_t *)&next, sizeof(next), false);
}

static void handle_roles(const show_rx_event_t *event, int64_t now_us)
{
    if ((!s_runtime.menu_open && !s_runtime.output_owned) ||
        !packet_header_valid(event->data,event->len,SHOW_WIRE_ROLES,sizeof(show_wire_roles_t)) ||
        !same_mac(event->src_mac,role_coordinator(now_us))) return;
    const show_wire_roles_t *map=(const void *)event->data;
    show_peer_t *peer=find_peer(event->src_mac,false);
    if (!peer || peer->boot!=map->boot || !map->count || map->count>SHOW_MAX_ROLES) return;
    if (same_mac(s_runtime.coordinator,event->src_mac) && s_runtime.roles.boot==map->boot &&
        (int32_t)(map->revision-s_runtime.roles.revision)<=0) return;
    unsigned used=0;
    for (unsigned i=0; i<map->count; ++i) {
        if (map->entries[i].role>=SHOW_MAX_ROLES || (used & (1u<<map->entries[i].role))) return;
        used |= 1u<<map->entries[i].role;
        for (unsigned j=0; j<i; ++j)
            if (same_mac(map->entries[i].mac,map->entries[j].mac)) return;
    }
    memcpy(s_runtime.coordinator,event->src_mac,6);
    apply_roles(map);
}

static void handle_ping_request(const show_rx_event_t *event, int64_t now_us)
{
    if (!packet_header_valid(event->data, event->len, SHOW_WIRE_PING_REQUEST,
                             sizeof(show_wire_ping_request_t))) return;
    const show_wire_ping_request_t *request = (const void *)event->data;
    if (s_runtime.authority || (!s_runtime.menu_open && !s_runtime.output_owned)) return;

    show_wire_ping_response_t response = {
        .protocol = SHOW_SYNC_PROTOCOL,
        .kind = SHOW_WIRE_PING_RESPONSE,
        .bytes = sizeof(response),
        .session_id = request->session_id,
        .sequence = request->sequence,
        .master_send_us = request->master_send_us,
        .follower_receive_us = now_us,
        .follower_send_us = esp_timer_get_time(),
    };
    (void)link_send_frame_to(event->src_mac, LINK_MSG_CONTROL,
                             (const uint8_t *)&response, sizeof(response), false);
}

static void handle_ping_response(const show_rx_event_t *event, int64_t now_us)
{
    if (!s_runtime.authority ||
        !packet_header_valid(event->data, event->len, SHOW_WIRE_PING_RESPONSE,
                             sizeof(show_wire_ping_response_t))) return;
    const show_wire_ping_response_t *response = (const void *)event->data;
    if (response->session_id != s_runtime.session_id ||
        response->follower_send_us < response->follower_receive_us ||
        now_us < response->master_send_us) return;

    int64_t round_trip = (now_us - response->master_send_us) -
                         (response->follower_send_us - response->follower_receive_us);
    if (round_trip < 0 || round_trip > 1000000LL) return;
    int64_t offset = ((response->follower_receive_us - response->master_send_us) +
                      (response->follower_send_us - now_us)) / 2;
    show_peer_t *peer = find_peer(event->src_mac, true);
    if (!peer) return;

    if (peer->updated_us == 0 || (uint32_t)round_trip <= peer->rtt_us) {
        peer->offset_us = offset;
        peer->rtt_us = (uint32_t)round_trip;
    } else {
        peer->offset_us = (peer->offset_us * 3 + offset) / 4;
        peer->rtt_us = (peer->rtt_us * 3u + (uint32_t)round_trip) / 4u;
    }
    peer->updated_us = now_us;
    send_transport_to(peer->mac, s_runtime.anchor_local_us + peer->offset_us, now_us);
}

static bool master_packet_allowed(const uint8_t mac[6], uint32_t session_id,
                                  int64_t now_us)
{
    if (!s_runtime.menu_open && !s_runtime.output_owned) return false;
    if (s_runtime.authority) {
        // Resolve near-simultaneous starts deterministically, before audio starts.
        return !s_runtime.audio_started && memcmp(mac, s_runtime.own_mac, 6) < 0;
    }
    if (!s_runtime.have_master) return true;
    if (same_mac(s_runtime.master_mac, mac)) return true;
    (void)session_id;
    return now_us - s_runtime.last_transport_rx_us > SHOW_SYNC_MASTER_HOLD_US;
}

static void handle_transport(const show_rx_event_t *event, int64_t now_us)
{
    if (!s_runtime.role_ready) return; // Finish discovery before choosing a role file.
    if (!packet_header_valid(event->data, event->len, SHOW_WIRE_TRANSPORT,
                             sizeof(show_wire_transport_t))) return;
    const show_wire_transport_t *packet = (const void *)event->data;
    if (!master_packet_allowed(event->src_mac, packet->session_id, now_us)) return;
    if (packet->state != SHOW_RUNTIME_PLAYING &&
        packet->state != SHOW_RUNTIME_PAUSED) return;
    if (packet->playhead_ms > packet->duration_ms || packet->role_count == 0 ||
        packet->role_count > SHOW_MAX_ROLES || packet->anchor_delay_us < 0 ||
        packet->anchor_delay_us > 2000000 ||
        (packet->target_anchor_us != 0 &&
         packet->target_anchor_us > now_us + 2000000LL)) return;
    if (!memchr(packet->slug, '\0', sizeof(packet->slug))) return;
    if (s_runtime.authority && s_runtime.audio_owned) {
        audio_player_stop();
        s_runtime.audio_owned = false;
    }
    // Coarse broadcasts must not overwrite a clock-corrected unicast anchor.
    if (s_runtime.synced && s_runtime.have_master &&
        same_mac(s_runtime.master_mac, event->src_mac) &&
        packet->session_id == s_runtime.session_id &&
        packet->generation == s_runtime.generation && !packet->target_anchor_us &&
        s_runtime.program.role.role_id == s_runtime.role_id) {
        s_runtime.last_transport_rx_us = now_us;
        return;
    }

    bool need_load = !s_runtime.program_loaded ||
                     s_runtime.program.info.show_uid != packet->show_uid ||
                     strcmp(s_runtime.slug, packet->slug) != 0;
    if (need_load) {
        if (now_us<s_runtime.role_reload_after_us) return;
        s_runtime.role_reload_after_us=now_us+1000000;
        esp_err_t err = show_runtime_load(packet->slug, s_runtime.role_id);
        if (err != ESP_OK || s_runtime.program.info.show_uid != packet->show_uid ||
            s_runtime.program.info.duration_ms != packet->duration_ms) {
            set_error(err == ESP_OK ? "show uid mismatch" : "remote load failed");
            return;
        }
    } else if (s_runtime.program.role.role_id!=s_runtime.role_id) {
        if (now_us<s_runtime.role_reload_after_us) return;
        s_runtime.role_reload_after_us=now_us+1000000;
        if (load_program(s_runtime.slug,s_runtime.role_id)!=ESP_OK) return;
    }
    if (packet->generation < s_runtime.generation &&
        s_runtime.session_id == packet->session_id) return;
    if (acquire_output() != ESP_OK) {
        set_error("LED output busy");
        return;
    }

    memcpy(s_runtime.master_mac, event->src_mac, 6);
    s_runtime.have_master = true;
    s_runtime.authority = false;
    s_runtime.session_id = packet->session_id;
    s_runtime.generation = packet->generation;
    s_runtime.last_transport_rx_us = now_us;
    s_runtime.synced = packet->target_anchor_us != 0;
    if (packet->state == SHOW_RUNTIME_PAUSED) {
        s_runtime.state = SHOW_RUNTIME_PAUSED;
        s_runtime.paused_playhead_ms = packet->playhead_ms;
    } else {
        s_runtime.state = SHOW_RUNTIME_PLAYING;
        s_runtime.anchor_playhead_ms = packet->playhead_ms;
        s_runtime.anchor_local_us = packet->target_anchor_us != 0
            ? packet->target_anchor_us : event->received_us + packet->anchor_delay_us;
    }
    set_error("");
}

static void handle_release(const show_rx_event_t *event, int64_t now_us)
{
    (void)now_us;
    if (!packet_header_valid(event->data, event->len, SHOW_WIRE_RELEASE,
                             sizeof(show_wire_release_t))) return;
    const show_wire_release_t *packet = (const void *)event->data;
    if (s_runtime.authority || !s_runtime.have_master ||
        !same_mac(s_runtime.master_mac, event->src_mac) ||
        packet->session_id != s_runtime.session_id) return;
    if (packet->generation < s_runtime.generation) return;
    show_runtime_stop(false);
}

static void handle_hello(const show_rx_event_t *event, int64_t now_us)
{
    if (!packet_header_valid(event->data,event->len,SHOW_WIRE_HELLO,sizeof(show_wire_hello_t))) return;
    const show_wire_hello_t *p=(const void *)event->data;
    if (p->role>=SHOW_MAX_ROLES || same_mac(event->src_mac,s_runtime.own_mac)) return;
    show_peer_t *peer=find_peer(event->src_mac,p->active);
    if (!peer) return;
    peer->seen_us=p->active ? now_us : 0;
    peer->role=p->role;
    peer->authority=p->authority;
    peer->role_ready=p->ready;
    if (peer->boot!=p->boot || (int32_t)(p->request-peer->request)>=0) {
        peer->wanted=p->wanted;
        peer->request=p->request;
    }
    peer->boot=p->boot;
}

static void process_rx_events(int64_t now_us)
{
    show_rx_event_t event;
    while (s_runtime.rx_queue && xQueueReceive(s_runtime.rx_queue, &event, 0) == pdTRUE) {
        if (event.len < 2 || event.data[0] != SHOW_SYNC_PROTOCOL) continue;
        switch (event.data[1]) {
            case SHOW_WIRE_PING_REQUEST: handle_ping_request(&event, event.received_us); break;
            case SHOW_WIRE_PING_RESPONSE: handle_ping_response(&event, event.received_us); break;
            case SHOW_WIRE_HELLO: handle_hello(&event,now_us); break;
            case SHOW_WIRE_QUERY:
                if (packet_header_valid(event.data,event.len,SHOW_WIRE_QUERY,4) &&
                    (s_runtime.menu_open || s_runtime.output_owned)) send_hello();
                break;
            case SHOW_WIRE_ROLES: handle_roles(&event,now_us); break;
            case SHOW_WIRE_TRANSPORT: handle_transport(&event, now_us); break;
            case SHOW_WIRE_RELEASE: handle_release(&event, now_us); break;
            default: break;
        }
        now_us = esp_timer_get_time();
    }
}

esp_err_t show_runtime_init(void)
{
    if (s_runtime.initialized) return ESP_OK;
    memset(&s_runtime, 0, sizeof(s_runtime));
    s_runtime.renderer = show_renderer_create();
    if (!s_runtime.renderer) return ESP_ERR_NO_MEM;
    s_runtime.rx_queue = xQueueCreate(SHOW_SYNC_QUEUE_DEPTH, sizeof(show_rx_event_t));
    if (!s_runtime.rx_queue) {
        show_renderer_destroy(s_runtime.renderer);
        memset(&s_runtime, 0, sizeof(s_runtime));
        return ESP_ERR_NO_MEM;
    }
    s_runtime.initialized = true;
    s_runtime.boot = esp_random();
    if (!s_runtime.boot) s_runtime.boot=1;
    s_runtime.wanted_role=UINT8_MAX;
    s_runtime.bt_delay_ms = 100;
    (void)esp_read_mac(s_runtime.own_mac, ESP_MAC_WIFI_STA);
    return ESP_OK;
}

static esp_err_t load_program(const char *show_slug, uint8_t role_id)
{
    show_package_program_t loaded = {0};
    esp_err_t err = show_package_load_program(show_slug, role_id, &loaded);
    if (err != ESP_OK) {
        set_error(esp_err_to_name(err));
        return err;
    }
    show_package_program_t previous=s_runtime.program;
    s_runtime.program = loaded;
    err = show_renderer_configure(s_runtime.renderer, &s_runtime.program, role_id);
    if (err != ESP_OK) {
        show_package_program_free(&s_runtime.program);
        s_runtime.program=previous;
        if (s_runtime.program_loaded)
            (void)show_renderer_configure(s_runtime.renderer,&s_runtime.program,previous.role.role_id);
        set_error(esp_err_to_name(err));
        return err;
    }
    show_package_program_free(&previous);
    s_runtime.program_loaded = true;
    s_runtime.frame_valid = false;
    return ESP_OK;
}

esp_err_t show_runtime_load(const char *show_slug, uint8_t role_id)
{
    if (!show_slug || role_id >= SHOW_MAX_ROLES) return ESP_ERR_INVALID_ARG;
    esp_err_t err = show_runtime_init();
    if (err != ESP_OK) return err;
    if (s_runtime.output_owned) show_runtime_stop(s_runtime.authority);
    err=load_program(show_slug,role_id);
    if (err!=ESP_OK) return err;
    s_runtime.role_id = role_id;
    snprintf(s_runtime.slug, sizeof(s_runtime.slug), "%s", show_slug);
    s_runtime.frame_pixels = led_layout_count();
    if (s_runtime.frame_pixels > LED_STRIP_LENGTH) s_runtime.frame_pixels = LED_STRIP_LENGTH;
    s_runtime.state = SHOW_RUNTIME_LOADED;
    s_runtime.authority = false;
    s_runtime.synced = false;
    s_runtime.have_master = false;
    s_runtime.session_id = 0;
    s_runtime.generation = 0;
    s_runtime.paused_playhead_ms = 0;
    set_error("");
    return ESP_OK;
}

esp_err_t show_runtime_set_role(uint8_t role_id)
{
    if (role_id >= SHOW_MAX_ROLES) return ESP_ERR_INVALID_ARG;
    if (!s_runtime.role_ready || s_runtime.wanted_role!=UINT8_MAX) return ESP_ERR_INVALID_STATE;
    if (role_id==s_runtime.role_id) return ESP_OK;
    s_runtime.wanted_role=role_id;
    ++s_runtime.role_request;
    send_hello();
    service_roles(esp_timer_get_time());
    return ESP_OK;
}

esp_err_t show_runtime_start_master(void)
{
    if (s_runtime.state != SHOW_RUNTIME_LOADED &&
        s_runtime.state != SHOW_RUNTIME_PAUSED) return ESP_ERR_INVALID_STATE;
    show_runtime_status_t status = show_runtime_get_status();
    if (!s_runtime.role_ready || s_runtime.wanted_role!=UINT8_MAX) {
        set_error("finding roles");
        return ESP_ERR_INVALID_STATE;
    }
    if (status.master_roles && !s_runtime.authority) {
        set_error("leader already active");
        return ESP_ERR_INVALID_STATE;
    }
    if (status.role_conflict) {
        set_error("duplicate roles");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_runtime.program.role.role_id!=s_runtime.role_id) {
        esp_err_t err=load_program(s_runtime.slug,s_runtime.role_id);
        if (err!=ESP_OK) return err;
    }
    char path[160];
    esp_err_t path_err = show_package_build_path(s_runtime.slug, path, sizeof(path));
    if (path_err != ESP_OK) return path_err;
    char *file = strrchr(path, '/');
    if (!file) return ESP_ERR_INVALID_ARG;
    strcpy(file + 1, "track.wav");
    FILE *audio_file = fopen(path, "rb");
    if (!audio_file) { set_error("missing track.wav"); return ESP_ERR_NOT_FOUND; }
    fclose(audio_file);
    esp_err_t err = acquire_output();
    if (err != ESP_OK) {
        set_error(esp_err_to_name(err));
        return err;
    }

    int64_t now_us = esp_timer_get_time();
    uint32_t start_ms = s_runtime.state == SHOW_RUNTIME_PAUSED
        ? s_runtime.paused_playhead_ms : 0u;
    s_runtime.authority = true;
    s_runtime.synced = true;
    s_runtime.have_master = false;
    s_runtime.session_id = esp_random();
    if (s_runtime.session_id == 0) s_runtime.session_id = 1;
    s_runtime.generation++;
    s_runtime.sequence = 0;
    s_runtime.anchor_playhead_ms = start_ms;
    s_runtime.anchor_local_us = now_us + SHOW_SYNC_START_LEAD_US;
    s_runtime.state = SHOW_RUNTIME_PAUSED; // Wait for actual PCM delivery, not task creation.
    s_runtime.paused_playhead_ms = 0;
    s_runtime.last_transport_tx_us = 0;
    s_runtime.last_ping_tx_us = 0;
    s_runtime.audio_started = false;
    s_runtime.audio_position_ms = 0;
    s_runtime.audio_request_us = now_us;
    s_runtime.audio_owned = true;
    send_hello();
    set_error("");
    broadcast_transport(now_us);
    err = audio_player_play_show(path);
    if (err != ESP_OK) {
        show_runtime_stop(true);
        set_error("audio start failed");
        return err;
    }
    return ESP_OK;
}

esp_err_t show_runtime_toggle_master_pause(void)
{
    if (!s_runtime.audio_started || !audio_player_is_active()) return ESP_ERR_INVALID_STATE;
    if (!s_runtime.authority ||
        (s_runtime.state != SHOW_RUNTIME_PLAYING &&
         s_runtime.state != SHOW_RUNTIME_PAUSED)) return ESP_ERR_INVALID_STATE;
    int64_t now_us = esp_timer_get_time();
    esp_err_t err = audio_player_toggle_play_pause();
    if (err != ESP_OK) return err;
    s_runtime.generation++;
    if (s_runtime.state == SHOW_RUNTIME_PLAYING) {
        s_runtime.paused_playhead_ms = current_playhead_ms(now_us);
        s_runtime.state = SHOW_RUNTIME_PAUSED;
    } else {
        s_runtime.anchor_playhead_ms = s_runtime.paused_playhead_ms;
        s_runtime.anchor_local_us = now_us + SHOW_SYNC_START_LEAD_US / 2;
        s_runtime.state = SHOW_RUNTIME_PLAYING;
    }
    broadcast_transport(now_us);
    return ESP_OK;
}

void show_runtime_stop(bool broadcast)
{
    if (!s_runtime.initialized) return;
    if (broadcast && s_runtime.authority && s_runtime.output_owned) {
        send_release();
        send_release();
    }
    if (s_runtime.audio_owned) audio_player_stop();
    s_runtime.audio_owned = s_runtime.audio_started = false;
    release_output();
    s_runtime.authority = false;
    s_runtime.synced = false;
    s_runtime.have_master = false;
    s_runtime.state = s_runtime.program_loaded
        ? SHOW_RUNTIME_LOADED : SHOW_RUNTIME_STOPPED;
    s_runtime.paused_playhead_ms = 0;
    s_runtime.anchor_playhead_ms = 0;
    send_hello();
}

void show_runtime_service_tick(float dt_sec)
{
    (void)dt_sec;
    if (!s_runtime.initialized) return;
    int64_t now_us = esp_timer_get_time();
    process_rx_events(now_us);
    now_us = esp_timer_get_time();
    service_roles(now_us);

    if (s_runtime.menu_open || s_runtime.output_owned) {
        if (now_us - s_runtime.last_hello_us >= 500000) {
            send_hello();
            s_runtime.last_hello_us = now_us;
        }
    }

    if (s_runtime.authority && s_runtime.audio_owned) {
        audio_progress_t p = audio_progress_get();
        if (p.rate && p.frames) {
            int64_t submitted_us = (int64_t)(p.frames * 1000000ULL / p.rate);
            int64_t delay_us = p.bluetooth ? (int64_t)s_runtime.bt_delay_ms * 1000
                                          : 8LL * 256 * 1000000 / p.rate;
            int64_t elapsed = now_us - p.updated_us;
            if (elapsed < 0) elapsed = 0;
            if (elapsed > delay_us) elapsed = delay_us;
            int64_t audible_us = submitted_us - delay_us + elapsed;
            if (audible_us < 0) audible_us = 0;
            uint32_t position = (uint32_t)(audible_us / 1000);
            if (position < s_runtime.audio_position_ms) position = s_runtime.audio_position_ms;
            s_runtime.audio_position_ms = position;
            if (!s_runtime.audio_started) {
                s_runtime.audio_started = true;
                s_runtime.state = SHOW_RUNTIME_PLAYING;
                ++s_runtime.generation;
                s_runtime.last_transport_tx_us = 0;
            }
            bool stalled = now_us - p.updated_us > delay_us + 100000;
            show_runtime_state_t state = audio_player_is_paused() || stalled
                ? SHOW_RUNTIME_PAUSED : SHOW_RUNTIME_PLAYING;
            if (s_runtime.state != state) {
                s_runtime.state = state;
                ++s_runtime.generation;
                s_runtime.last_transport_tx_us = 0;
            }
            if (state == SHOW_RUNTIME_PLAYING) {
                s_runtime.anchor_playhead_ms = position;
                s_runtime.anchor_local_us = now_us;
            } else {
                s_runtime.paused_playhead_ms = position;
            }
        }
        if (!s_runtime.audio_started &&
            (now_us - s_runtime.audio_request_us > 5000000 || !audio_player_is_active())) {
            show_runtime_stop(true);
            set_error("audio failed to start");
        } else if (s_runtime.audio_started && !audio_player_is_active() &&
                   now_us - p.updated_us > 1000000) {
            show_runtime_stop(true);
        }
    }

    if (s_runtime.authority && s_runtime.output_owned) {
        if (have_listeners(now_us) && now_us - s_runtime.last_ping_tx_us >= SHOW_SYNC_PING_US) {
            send_ping(now_us);
            s_runtime.last_ping_tx_us = now_us;
        }
        if (now_us - s_runtime.last_transport_tx_us >= SHOW_SYNC_TRANSPORT_US) {
            broadcast_transport(now_us);
            s_runtime.last_transport_tx_us = now_us;
        }
    } else if (s_runtime.have_master &&
               now_us - s_runtime.last_transport_rx_us > SHOW_SYNC_STALE_US) {
        s_runtime.synced = false;
    }

    if (!s_runtime.output_owned ||
        (s_runtime.state != SHOW_RUNTIME_PLAYING &&
         s_runtime.state != SHOW_RUNTIME_PAUSED)) return;

    if (s_runtime.program.role.role_id!=s_runtime.role_id) {
        if (now_us<s_runtime.role_reload_after_us) return;
        s_runtime.role_reload_after_us=now_us+1000000;
        if (load_program(s_runtime.slug,s_runtime.role_id)!=ESP_OK) return;
        // Keep the audio owner and clock intact; render at the current time.
        now_us=esp_timer_get_time();
    }

    uint32_t playhead = current_playhead_ms(now_us);
    if (s_runtime.state == SHOW_RUNTIME_PLAYING &&
        playhead >= s_runtime.program.info.duration_ms) {
        s_runtime.state = SHOW_RUNTIME_PAUSED;
        s_runtime.paused_playhead_ms = s_runtime.program.info.duration_ms;
        if (s_runtime.authority) {
            s_runtime.generation++;
            broadcast_transport(now_us);
        }
    }

    // Effects use absolute song time, not frame indices. Older packages' 30 Hz
    // hints need not limit playback smoothness or require re-exporting shows.
    int64_t frame_period_us = 1000000LL / SHOW_RENDER_FPS;
    // Fast backoff, slow recovery: spend at most about half the available wall
    // time rendering. Transport/audio service above still runs on skipped ticks.
    if (2LL * s_runtime.render_budget_us > frame_period_us)
        frame_period_us = 2LL * s_runtime.render_budget_us;
    if (now_us < s_runtime.render_not_before_us) return;
    if (s_runtime.last_render_us != 0 &&
        now_us - s_runtime.last_render_us < frame_period_us) return;
    // Advance the scheduled deadline, not the observed shell tick. Resetting
    // to now quantizes a 30 Hz show into 20 Hz when a 60 Hz tick arrives early.
    if (!s_runtime.last_render_us || now_us - s_runtime.last_render_us >= 2 * frame_period_us)
        s_runtime.last_render_us = now_us;
    else
        s_runtime.last_render_us += frame_period_us;

    if (s_runtime.state == SHOW_RUNTIME_PLAYING && now_us < s_runtime.anchor_local_us) {
        s_runtime.frame_valid = false;
        memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
        (void)submit_show_frame(s_runtime.frame_pixels);
        return;
    }

    uint8_t brightness = led_modes_get_brightness();
    bool unchanged = s_runtime.frame_valid && s_runtime.frame_playhead_ms == playhead &&
                     s_runtime.frame_brightness == brightness;
    uint32_t calc_us = 0, submit_us = 0;
    if (!unchanged) {
        int64_t render_start_us = esp_timer_get_time();
        size_t pixels = 0;
        esp_err_t err = show_renderer_render(s_runtime.renderer, playhead,
                                             brightness,
                                             s_runtime.frame, LED_STRIP_LENGTH, &pixels);
        int64_t calc_end_us = esp_timer_get_time();
        calc_us = (uint32_t)(calc_end_us - render_start_us);
        if (err == ESP_OK) {
            err = submit_show_frame(pixels);
        }
        if (err != ESP_OK) {
            memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
            (void)submit_show_frame(s_runtime.frame_pixels);
            set_error(esp_err_to_name(err));
        }
        submit_us = (uint32_t)(esp_timer_get_time() - calc_end_us);
        s_runtime.frame_valid = err == ESP_OK;
        s_runtime.frame_playhead_ms = playhead;
        s_runtime.frame_brightness = brightness;
    }
    static int64_t report_us;
    static uint32_t frames, render_total_us, render_max_us, calc_max_us, submit_max_us;
    uint32_t cost_us = calc_us + submit_us;
    if (!unchanged) {
        uint32_t previous = s_runtime.render_budget_us;
        s_runtime.render_budget_us = cost_us >= previous ? cost_us :
            previous - (previous - cost_us + 15u) / 16u;
        // Do not immediately retry after an overrun, even if a deadline passed.
        // Always render the latest playhead next time, never missed frames.
        s_runtime.render_not_before_us = esp_timer_get_time() + s_runtime.render_budget_us;
    }
    if (!report_us) report_us = now_us;
    if (!unchanged) ++frames;
    render_total_us += cost_us;
    if (cost_us > render_max_us) render_max_us = cost_us;
    if (calc_us > calc_max_us) calc_max_us = calc_us;
    if (submit_us > submit_max_us) submit_max_us = submit_us;
    if (now_us - report_us >= 5000000) {
        ESP_LOGI("show_runtime", "render %.1ffps target=%.1ffps avg=%lums max=%lums calc_max=%lums submit_max=%lums internal=%u largest=%u",
                 frames * 1000000.0 / (now_us - report_us),
                 1000000.0 / frame_period_us,
                 (unsigned long)(frames ? render_total_us / frames / 1000 : 0),
                 (unsigned long)(render_max_us / 1000),
                 (unsigned long)(calc_max_us / 1000),
                 (unsigned long)(submit_max_us / 1000),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        report_us = now_us;
        frames = render_total_us = render_max_us = 0;
        calc_max_us = submit_max_us = 0;
    }
}

void show_runtime_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac,
                                    const uint8_t *payload, size_t len)
{
    if (type != LINK_MSG_CONTROL || !src_mac || !payload || len < 2 ||
        payload[0] != SHOW_SYNC_PROTOCOL || len > SHOW_SYNC_MAX_PACKET ||
        !s_runtime.initialized || !s_runtime.rx_queue) return;
    show_rx_event_t event = {.len = len};
    event.received_us = esp_timer_get_time();
    memcpy(event.src_mac, src_mac, 6);
    memcpy(event.data, payload, len);
    (void)xQueueSend(s_runtime.rx_queue, &event, 0);
}

bool show_runtime_owns_leds(void)
{
    return s_runtime.output_owned;
}

show_runtime_status_t show_runtime_get_status(void)
{
    show_runtime_status_t status = {
        .initialized = s_runtime.initialized,
        .loaded = s_runtime.program_loaded,
        .owns_leds = s_runtime.output_owned,
        .authority = s_runtime.authority,
        .synced = s_runtime.synced,
        .state = s_runtime.state,
        .role_id = s_runtime.role_id,
        .role_ready = s_runtime.role_ready,
        .role_pending = s_runtime.wanted_role!=UINT8_MAX,
        .show_uid = s_runtime.program.info.show_uid,
        .duration_ms = s_runtime.program.info.duration_ms,
        .playhead_ms = current_playhead_ms(esp_timer_get_time()),
    };
    snprintf(status.slug, sizeof(status.slug), "%s", s_runtime.slug);
    snprintf(status.error, sizeof(status.error), "%s", s_runtime.error);
    uint32_t best_rtt = UINT_MAX;
    status.present_roles = s_runtime.role_ready ? 1u << s_runtime.role_id : 0;
    status.master_roles = s_runtime.authority ? status.present_roles : 0;
    int64_t now_us = esp_timer_get_time();
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        const show_peer_t *peer = &s_runtime.peers[i];
        if (!peer->used || !peer->seen_us || now_us - peer->seen_us > SHOW_SYNC_STALE_US) continue;
        status.peer_count++;
        uint8_t role=peer->role;
        bool ready=peer->role_ready;
        for (unsigned j=0;j<s_runtime.roles.count;++j) {
            const show_role_entry_t *e=&s_runtime.roles.entries[j];
            if (same_mac(e->mac,peer->mac) && e->boot==peer->boot) { role=e->role; ready=true; break; }
        }
        if (!ready) continue;
        uint8_t bit = 1u << role;
        if (status.present_roles & bit) status.role_conflict = true;
        status.present_roles |= bit;
        if (peer->authority) status.master_roles |= bit;
        if (peer->rtt_us < best_rtt) best_rtt = peer->rtt_us;
    }
    status.best_rtt_ms = best_rtt == UINT_MAX ? 0u : (uint16_t)(best_rtt / 1000u);
    return status;
}

const char *show_runtime_state_name(show_runtime_state_t state)
{
    switch (state) {
        case SHOW_RUNTIME_LOADED: return "ready";
        case SHOW_RUNTIME_PLAYING: return "play";
        case SHOW_RUNTIME_PAUSED: return "pause";
        default: return "stop";
    }
}
