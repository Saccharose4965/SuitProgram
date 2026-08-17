#include "show_runtime.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "led.h"
#include "led_layout.h"
#include "led_modes.h"
#include "show_package.h"
#include "show_renderer.h"

#define SHOW_SYNC_PROTOCOL       0x51u
#define SHOW_SYNC_QUEUE_DEPTH    12u
#define SHOW_SYNC_MAX_PACKET     112u
#define SHOW_SYNC_MAX_PEERS      8u
#define SHOW_SYNC_TRANSPORT_US   250000LL
#define SHOW_SYNC_PING_US        1000000LL
#define SHOW_SYNC_START_LEAD_US  500000LL
#define SHOW_SYNC_STALE_US       2000000LL
#define SHOW_SYNC_MASTER_HOLD_US 5000000LL

typedef enum {
    SHOW_WIRE_PING_REQUEST = 1,
    SHOW_WIRE_PING_RESPONSE,
    SHOW_WIRE_TRANSPORT,
    SHOW_WIRE_RELEASE,
} show_wire_kind_t;

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
} show_rx_event_t;

typedef struct {
    bool used;
    uint8_t mac[6];
    int64_t offset_us; /* follower clock - master clock */
    uint32_t rtt_us;
    int64_t updated_us;
} show_peer_t;

typedef struct {
    bool initialized;
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
    int64_t last_transport_tx_us;
    int64_t last_ping_tx_us;
    int64_t last_transport_rx_us;
    uint8_t master_mac[6];
    bool have_master;
    show_peer_t peers[SHOW_SYNC_MAX_PEERS];
} show_runtime_t;

static show_runtime_t s_runtime;

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
    led_modes_enable(false);
    led_beat_enable(false);
    led_audio_brightness_enable(false);
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
    (void)link_send_frame(LINK_MSG_CONTROL, (const uint8_t *)&packet,
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

static void broadcast_transport(int64_t now_us)
{
    send_transport_to(NULL, 0, now_us);
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        show_peer_t *peer = &s_runtime.peers[i];
        if (!peer->used || now_us - peer->updated_us > SHOW_SYNC_STALE_US) continue;
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
    (void)link_send_frame(LINK_MSG_CONTROL, (const uint8_t *)&packet,
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

static void handle_ping_request(const show_rx_event_t *event, int64_t now_us)
{
    if (!packet_header_valid(event->data, event->len, SHOW_WIRE_PING_REQUEST,
                             sizeof(show_wire_ping_request_t))) return;
    const show_wire_ping_request_t *request = (const void *)event->data;
    if (s_runtime.authority) return;

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
    if (s_runtime.authority) return false;
    if (!s_runtime.have_master) return true;
    if (same_mac(s_runtime.master_mac, mac)) return true;
    (void)session_id;
    return now_us - s_runtime.last_transport_rx_us > SHOW_SYNC_MASTER_HOLD_US;
}

static void handle_transport(const show_rx_event_t *event, int64_t now_us)
{
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

    bool need_load = s_runtime.state == SHOW_RUNTIME_STOPPED ||
                     s_runtime.program.info.show_uid != packet->show_uid ||
                     strcmp(s_runtime.slug, packet->slug) != 0;
    if (need_load) {
        esp_err_t err = show_runtime_load(packet->slug, s_runtime.role_id);
        if (err != ESP_OK || s_runtime.program.info.show_uid != packet->show_uid ||
            s_runtime.program.info.duration_ms != packet->duration_ms) {
            set_error(err == ESP_OK ? "show uid mismatch" : "remote load failed");
            return;
        }
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
            ? packet->target_anchor_us : now_us + packet->anchor_delay_us;
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
    show_runtime_stop(false);
}

static void process_rx_events(int64_t now_us)
{
    show_rx_event_t event;
    while (s_runtime.rx_queue && xQueueReceive(s_runtime.rx_queue, &event, 0) == pdTRUE) {
        if (event.len < 2 || event.data[0] != SHOW_SYNC_PROTOCOL) continue;
        switch (event.data[1]) {
            case SHOW_WIRE_PING_REQUEST: handle_ping_request(&event, now_us); break;
            case SHOW_WIRE_PING_RESPONSE: handle_ping_response(&event, now_us); break;
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
    return ESP_OK;
}

esp_err_t show_runtime_load(const char *show_slug, uint8_t role_id)
{
    if (!show_slug || role_id >= SHOW_MAX_ROLES) return ESP_ERR_INVALID_ARG;
    esp_err_t err = show_runtime_init();
    if (err != ESP_OK) return err;
    if (s_runtime.output_owned) show_runtime_stop(s_runtime.authority);

    show_package_program_t loaded = {0};
    err = show_package_load_program(show_slug, role_id, &loaded);
    if (err != ESP_OK) {
        set_error(esp_err_to_name(err));
        return err;
    }
    show_package_program_free(&s_runtime.program);
    s_runtime.program = loaded;
    s_runtime.program_loaded = true;
    err = show_renderer_configure(s_runtime.renderer, &s_runtime.program, role_id);
    if (err != ESP_OK) {
        show_package_program_free(&s_runtime.program);
        s_runtime.program_loaded = false;
        s_runtime.state = SHOW_RUNTIME_STOPPED;
        set_error(esp_err_to_name(err));
        return err;
    }
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
    if (s_runtime.output_owned) return ESP_ERR_INVALID_STATE;
    if (s_runtime.state == SHOW_RUNTIME_STOPPED) {
        s_runtime.role_id = role_id;
        return ESP_OK;
    }
    char slug[sizeof(s_runtime.slug)];
    snprintf(slug, sizeof(slug), "%s", s_runtime.slug);
    return show_runtime_load(slug, role_id);
}

esp_err_t show_runtime_start_master(void)
{
    if (s_runtime.state != SHOW_RUNTIME_LOADED &&
        s_runtime.state != SHOW_RUNTIME_PAUSED) return ESP_ERR_INVALID_STATE;
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
    s_runtime.state = SHOW_RUNTIME_PLAYING;
    s_runtime.last_transport_tx_us = 0;
    s_runtime.last_ping_tx_us = 0;
    memset(s_runtime.peers, 0, sizeof(s_runtime.peers));
    set_error("");
    broadcast_transport(now_us);
    return ESP_OK;
}

esp_err_t show_runtime_toggle_master_pause(void)
{
    if (!s_runtime.authority ||
        (s_runtime.state != SHOW_RUNTIME_PLAYING &&
         s_runtime.state != SHOW_RUNTIME_PAUSED)) return ESP_ERR_INVALID_STATE;
    int64_t now_us = esp_timer_get_time();
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
    release_output();
    s_runtime.authority = false;
    s_runtime.synced = false;
    s_runtime.have_master = false;
    s_runtime.state = s_runtime.program_loaded
        ? SHOW_RUNTIME_LOADED : SHOW_RUNTIME_STOPPED;
    s_runtime.paused_playhead_ms = 0;
    s_runtime.anchor_playhead_ms = 0;
}

void show_runtime_service_tick(float dt_sec)
{
    (void)dt_sec;
    if (!s_runtime.initialized) return;
    int64_t now_us = esp_timer_get_time();
    process_rx_events(now_us);
    now_us = esp_timer_get_time();

    if (s_runtime.authority && s_runtime.output_owned) {
        if (now_us - s_runtime.last_ping_tx_us >= SHOW_SYNC_PING_US) {
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

    uint16_t fps = s_runtime.program.info.fps_hint;
    if (fps < 15u) fps = 15u;
    if (fps > 60u) fps = 60u;
    int64_t frame_period_us = 1000000LL / fps;
    if (s_runtime.last_render_us != 0 &&
        now_us - s_runtime.last_render_us < frame_period_us) return;

    if (s_runtime.state == SHOW_RUNTIME_PLAYING && now_us < s_runtime.anchor_local_us) {
        memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
        (void)led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame,
                                    s_runtime.frame_pixels);
        s_runtime.last_render_us = now_us;
        return;
    }

    size_t pixels = 0;
    esp_err_t err = show_renderer_render(s_runtime.renderer, playhead,
                                         led_modes_get_brightness(),
                                         s_runtime.frame, LED_STRIP_LENGTH, &pixels);
    if (err == ESP_OK) {
        err = led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame, pixels);
    }
    if (err != ESP_OK) {
        memset(s_runtime.frame, 0, sizeof(s_runtime.frame));
        (void)led_show_pixels_owned(LED_OUTPUT_OWNER_SHOW, s_runtime.frame,
                                    s_runtime.frame_pixels);
        set_error(esp_err_to_name(err));
    }
    s_runtime.last_render_us = now_us;
}

void show_runtime_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac,
                                    const uint8_t *payload, size_t len)
{
    if (type != LINK_MSG_CONTROL || !src_mac || !payload || len < 2 ||
        payload[0] != SHOW_SYNC_PROTOCOL || len > SHOW_SYNC_MAX_PACKET ||
        !s_runtime.initialized || !s_runtime.rx_queue) return;
    show_rx_event_t event = {.len = len};
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
        .show_uid = s_runtime.program.info.show_uid,
        .duration_ms = s_runtime.program.info.duration_ms,
        .playhead_ms = current_playhead_ms(esp_timer_get_time()),
    };
    snprintf(status.slug, sizeof(status.slug), "%s", s_runtime.slug);
    snprintf(status.error, sizeof(status.error), "%s", s_runtime.error);
    uint32_t best_rtt = UINT_MAX;
    int64_t now_us = esp_timer_get_time();
    for (size_t i = 0; i < SHOW_SYNC_MAX_PEERS; ++i) {
        const show_peer_t *peer = &s_runtime.peers[i];
        if (!peer->used || now_us - peer->updated_us > SHOW_SYNC_STALE_US) continue;
        status.peer_count++;
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
