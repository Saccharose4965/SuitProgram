#include "remote_control.h"

#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define REMOTE_PROTOCOL 1u
#define REMOTE_TIMEOUT_US 3000000LL
#define REMOTE_HEARTBEAT_US 500000LL
#define REMOTE_PEER_EXPIRY_US 5000000LL
#define REMOTE_QUEUE_LEN 8u

typedef enum {
    WIRE_START = 1,
    WIRE_INPUT,
    WIRE_VIEW,
    WIRE_HEARTBEAT,
    WIRE_END,
} wire_kind_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol;
    uint8_t kind;
    uint16_t bytes;
    uint32_t session;
    uint32_t sequence;
} wire_header_t;

typedef struct __attribute__((packed)) {
    wire_header_t header;
    uint8_t button;
    uint8_t event_type;
} wire_input_t;

typedef struct __attribute__((packed)) {
    wire_header_t header;
    uint8_t view_kind;
    uint8_t total_count;
    uint8_t selected;
    uint8_t first;
    uint8_t entry_count;
    uint8_t legend[4];
    char app_id[REMOTE_CONTROL_APP_ID_LEN];
    char title[REMOTE_CONTROL_TITLE_LEN];
    char labels[REMOTE_CONTROL_MAX_ENTRIES][REMOTE_CONTROL_LABEL_LEN];
} wire_view_t;

_Static_assert(sizeof(wire_view_t) <= 200, "remote view exceeds Link payload");

typedef struct {
    uint8_t mac[6];
    int64_t seen_us;
} peer_entry_t;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static remote_control_role_t s_role;
static uint8_t s_other_mac[6];
static uint32_t s_session;
static uint32_t s_tx_sequence;
static uint32_t s_rx_sequence;
static int64_t s_last_rx_us;
static int64_t s_last_heartbeat_us;
static peer_entry_t s_peers[REMOTE_CONTROL_MAX_PEERS];
static remote_control_view_t s_view;
static bool s_view_valid;
static uint32_t s_view_hash;
static remote_control_command_t s_commands[REMOTE_QUEUE_LEN];
static unsigned s_command_read;
static unsigned s_command_write;

static bool same_mac(const uint8_t a[6], const uint8_t b[6])
{
    return a && b && memcmp(a, b, 6) == 0;
}

static uint32_t bytes_hash(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; ++i) hash = (hash ^ p[i]) * 16777619u;
    return hash;
}

static void queue_command(remote_control_command_t command)
{
    portENTER_CRITICAL(&s_mux);
    unsigned next = (s_command_write + 1u) % REMOTE_QUEUE_LEN;
    if (next != s_command_read) {
        s_commands[s_command_write] = command;
        s_command_write = next;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void track_peer(const uint8_t mac[6], int64_t now_us)
{
    if (!mac) return;
    portENTER_CRITICAL(&s_mux);
    int slot = -1;
    int oldest = 0;
    for (int i = 0; i < REMOTE_CONTROL_MAX_PEERS; ++i) {
        if (same_mac(s_peers[i].mac, mac)) { slot = i; break; }
        if (!s_peers[i].seen_us) oldest = i;
        else if (s_peers[i].seen_us < s_peers[oldest].seen_us) oldest = i;
    }
    if (slot < 0) slot = oldest;
    memcpy(s_peers[slot].mac, mac, 6);
    s_peers[slot].seen_us = now_us;
    portEXIT_CRITICAL(&s_mux);
}

static bool header_valid(const wire_header_t *header, size_t len, wire_kind_t kind,
                         size_t expected)
{
    return header && len == expected && header->protocol == REMOTE_PROTOCOL &&
        header->kind == kind && header->bytes == expected;
}

static esp_err_t send_header(wire_kind_t kind, bool require_ack)
{
    wire_header_t packet;
    uint8_t mac[6];
    portENTER_CRITICAL(&s_mux);
    packet = (wire_header_t){REMOTE_PROTOCOL, kind, sizeof(packet),
                             s_session, ++s_tx_sequence};
    memcpy(mac, s_other_mac, sizeof(mac));
    portEXIT_CRITICAL(&s_mux);
    return link_send_frame_to(mac, LINK_MSG_REMOTE_CONTROL,
                              (const uint8_t *)&packet, sizeof(packet), require_ack);
}

void remote_control_receive(link_msg_type_t type, const uint8_t *src_mac,
                            const uint8_t *payload, size_t len)
{
    int64_t now_us = esp_timer_get_time();
    if (type == LINK_MSG_INFO) track_peer(src_mac, now_us);
    if (type != LINK_MSG_REMOTE_CONTROL || !src_mac || !payload ||
        len < sizeof(wire_header_t)) return;

    const wire_header_t *header = (const void *)payload;
    if (header->protocol != REMOTE_PROTOCOL || header->bytes != len) return;

    if (header->kind == WIRE_START && header_valid(header, len, WIRE_START, sizeof(*header))) {
        bool accept = false;
        bool new_lease = false;
        portENTER_CRITICAL(&s_mux);
        bool repeated = s_role == REMOTE_CONTROL_TARGET &&
            s_session == header->session && same_mac(s_other_mac, src_mac);
        if (s_role == REMOTE_CONTROL_OFF || repeated) {
            new_lease = !repeated;
            s_role = REMOTE_CONTROL_TARGET;
            memcpy(s_other_mac, src_mac, 6);
            s_session = header->session;
            s_rx_sequence = header->sequence;
            if (new_lease) {
                s_tx_sequence = 0;
                s_command_read = s_command_write;
            }
            s_last_rx_us = now_us;
            s_view_hash = 0;
            accept = true;
        }
        portEXIT_CRITICAL(&s_mux);
        if (accept && new_lease)
            queue_command((remote_control_command_t){.kind=REMOTE_CONTROL_COMMAND_HOME});
        return;
    }

    portENTER_CRITICAL(&s_mux);
    bool belongs = s_role != REMOTE_CONTROL_OFF && s_session == header->session &&
        same_mac(s_other_mac, src_mac);
    bool fresh = belongs && (int32_t)(header->sequence - s_rx_sequence) > 0;
    if (fresh) {
        s_rx_sequence = header->sequence;
        s_last_rx_us = now_us;
    }
    remote_control_role_t role = s_role;
    portEXIT_CRITICAL(&s_mux);
    if (!fresh) return;

    if (header->kind == WIRE_HEARTBEAT &&
        header_valid(header, len, WIRE_HEARTBEAT, sizeof(*header))) return;
    if (header->kind == WIRE_END &&
        header_valid(header, len, WIRE_END, sizeof(*header))) {
        portENTER_CRITICAL(&s_mux);
        s_role = REMOTE_CONTROL_OFF;
        s_view_valid = false;
        portEXIT_CRITICAL(&s_mux);
        queue_command((remote_control_command_t){.kind=REMOTE_CONTROL_COMMAND_END});
        return;
    }
    if (header->kind == WIRE_INPUT && role == REMOTE_CONTROL_TARGET &&
        header_valid(header, len, WIRE_INPUT, sizeof(wire_input_t))) {
        const wire_input_t *packet = (const void *)payload;
        if (packet->button >= INPUT_BTN_A && packet->button <= INPUT_BTN_D &&
            packet->event_type == INPUT_EVENT_PRESS) {
            remote_control_command_t command = {.kind=REMOTE_CONTROL_COMMAND_INPUT};
            command.input.button = (input_button_t)packet->button;
            command.input.type = INPUT_EVENT_PRESS;
            command.input.at_ticks = 0;
            queue_command(command);
        }
        return;
    }
    if (header->kind == WIRE_VIEW && role == REMOTE_CONTROL_CONTROLLER &&
        header_valid(header, len, WIRE_VIEW, sizeof(wire_view_t))) {
        const wire_view_t *packet = (const void *)payload;
        if (packet->entry_count > REMOTE_CONTROL_MAX_ENTRIES ||
            packet->view_kind > REMOTE_CONTROL_VIEW_GRID) return;
        remote_control_view_t view = {0};
        view.kind = (remote_control_view_kind_t)packet->view_kind;
        view.total_count = packet->total_count;
        view.selected = packet->selected;
        view.first = packet->first;
        view.entry_count = packet->entry_count;
        memcpy(view.legend, packet->legend, sizeof(view.legend));
        memcpy(view.app_id, packet->app_id, sizeof(view.app_id));
        memcpy(view.title, packet->title, sizeof(view.title));
        memcpy(view.labels, packet->labels, sizeof(view.labels));
        view.app_id[sizeof(view.app_id)-1] = '\0';
        view.title[sizeof(view.title)-1] = '\0';
        for (unsigned i=0; i<REMOTE_CONTROL_MAX_ENTRIES; ++i)
            view.labels[i][REMOTE_CONTROL_LABEL_LEN-1] = '\0';
        portENTER_CRITICAL(&s_mux);
        s_view = view;
        s_view_valid = true;
        portEXIT_CRITICAL(&s_mux);
    }
}

void remote_control_service_tick(void)
{
    int64_t now_us = esp_timer_get_time();
    bool heartbeat = false;
    bool expired = false;
    portENTER_CRITICAL(&s_mux);
    if (s_role != REMOTE_CONTROL_OFF) {
        expired = now_us - s_last_rx_us > REMOTE_TIMEOUT_US;
        heartbeat = !expired && now_us - s_last_heartbeat_us >= REMOTE_HEARTBEAT_US;
        if (heartbeat) s_last_heartbeat_us = now_us;
        if (expired) {
            s_role = REMOTE_CONTROL_OFF;
            s_view_valid = false;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (heartbeat) (void)send_header(WIRE_HEARTBEAT, false);
    if (expired) queue_command((remote_control_command_t){.kind=REMOTE_CONTROL_COMMAND_END});
}

size_t remote_control_get_peers(remote_control_peer_t *out, size_t capacity)
{
    int64_t now_us = esp_timer_get_time();
    size_t count = 0;
    portENTER_CRITICAL(&s_mux);
    for (int i=0; i<REMOTE_CONTROL_MAX_PEERS && count<capacity; ++i) {
        if (!s_peers[i].seen_us || now_us-s_peers[i].seen_us>REMOTE_PEER_EXPIRY_US) continue;
        memcpy(out[count].mac, s_peers[i].mac, 6);
        out[count].age_ms = (int)((now_us-s_peers[i].seen_us)/1000);
        count++;
    }
    portEXIT_CRITICAL(&s_mux);
    return count;
}

esp_err_t remote_control_begin(const uint8_t target_mac[6])
{
    if (!target_mac) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_mux);
    if (s_role != REMOTE_CONTROL_OFF) {
        portEXIT_CRITICAL(&s_mux);
        return ESP_ERR_INVALID_STATE;
    }
    s_role = REMOTE_CONTROL_CONTROLLER;
    memcpy(s_other_mac, target_mac, 6);
    s_session = esp_random() | 1u;
    s_tx_sequence = 0;
    s_rx_sequence = 0;
    s_last_rx_us = esp_timer_get_time();
    s_last_heartbeat_us = 0;
    s_view_valid = false;
    s_command_read = s_command_write;
    portEXIT_CRITICAL(&s_mux);
    esp_err_t err = send_header(WIRE_START, true);
    if (err != ESP_OK) remote_control_end();
    return err;
}

void remote_control_end(void)
{
    remote_control_role_t role;
    portENTER_CRITICAL(&s_mux);
    role = s_role;
    portEXIT_CRITICAL(&s_mux);
    if (role != REMOTE_CONTROL_OFF) (void)send_header(WIRE_END, true);
    portENTER_CRITICAL(&s_mux);
    s_role = REMOTE_CONTROL_OFF;
    s_view_valid = false;
    portEXIT_CRITICAL(&s_mux);
}

remote_control_role_t remote_control_role(void)
{
    portENTER_CRITICAL(&s_mux);
    remote_control_role_t role = s_role;
    portEXIT_CRITICAL(&s_mux);
    return role;
}

bool remote_control_active(void) { return remote_control_role() != REMOTE_CONTROL_OFF; }

bool remote_control_get_view(remote_control_view_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&s_mux);
    bool valid = s_view_valid;
    if (valid) *out = s_view;
    portEXIT_CRITICAL(&s_mux);
    return valid;
}

esp_err_t remote_control_send_input(const input_event_t *event)
{
    if (!event || event->type != INPUT_EVENT_PRESS ||
        event->button < INPUT_BTN_A || event->button > INPUT_BTN_D) return ESP_ERR_INVALID_ARG;
    wire_input_t packet;
    uint8_t mac[6];
    portENTER_CRITICAL(&s_mux);
    if (s_role != REMOTE_CONTROL_CONTROLLER) {
        portEXIT_CRITICAL(&s_mux);
        return ESP_ERR_INVALID_STATE;
    }
    packet.header = (wire_header_t){REMOTE_PROTOCOL, WIRE_INPUT, sizeof(packet),
                                    s_session, ++s_tx_sequence};
    packet.button = event->button;
    packet.event_type = event->type;
    memcpy(mac, s_other_mac, 6);
    portEXIT_CRITICAL(&s_mux);
    return link_send_frame_to(mac, LINK_MSG_REMOTE_CONTROL,
                              (const uint8_t *)&packet, sizeof(packet), true);
}

bool remote_control_poll_command(remote_control_command_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&s_mux);
    if (s_command_read == s_command_write) {
        portEXIT_CRITICAL(&s_mux);
        return false;
    }
    *out = s_commands[s_command_read];
    s_command_read = (s_command_read + 1u) % REMOTE_QUEUE_LEN;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

void remote_control_publish_view(const char *app_id,
                                 const remote_control_view_t *view)
{
    if (!view) return;
    wire_view_t packet = {0};
    uint8_t mac[6];
    portENTER_CRITICAL(&s_mux);
    if (s_role != REMOTE_CONTROL_TARGET) {
        portEXIT_CRITICAL(&s_mux);
        return;
    }
    packet.header = (wire_header_t){REMOTE_PROTOCOL, WIRE_VIEW, sizeof(packet),
                                    s_session, s_tx_sequence + 1u};
    packet.view_kind = view->kind;
    packet.total_count = view->total_count;
    packet.selected = view->selected;
    packet.first = view->first;
    packet.entry_count = view->entry_count;
    memcpy(packet.legend, view->legend, sizeof(packet.legend));
    snprintf(packet.app_id, sizeof(packet.app_id), "%s", app_id ? app_id : "");
    snprintf(packet.title, sizeof(packet.title), "%s", view->title);
    memcpy(packet.labels, view->labels, sizeof(packet.labels));
    uint32_t hash = bytes_hash((const uint8_t *)&packet + sizeof(packet.header),
                               sizeof(packet) - sizeof(packet.header));
    if (hash == s_view_hash) {
        portEXIT_CRITICAL(&s_mux);
        return;
    }
    s_view_hash = hash;
    packet.header.sequence = ++s_tx_sequence;
    memcpy(mac, s_other_mac, 6);
    portEXIT_CRITICAL(&s_mux);
    (void)link_send_frame_to(mac, LINK_MSG_REMOTE_CONTROL,
                             (const uint8_t *)&packet, sizeof(packet), false);
}
