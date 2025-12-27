#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINK_PATH_NONE = 0,
    LINK_PATH_ESPNOW,
    LINK_PATH_WIFI,
} link_path_t;

typedef enum {
    LINK_MSG_INFO    = 1, // status/telemetry
    LINK_MSG_MESSAGE = 2, // user data / file header etc.
    LINK_MSG_GAME    = 3, // game control (e.g., Pong)
} link_msg_type_t;

typedef struct {
    // Wi-Fi/UDP fallback target
    const char *ssid;
    const char *pass;
    const char *pc_ip;
    uint16_t    pc_port;

    // Optional ESP-NOW peer MAC (6 bytes) if/when supported
    const uint8_t *espnow_peer_mac;
} link_config_t;

typedef void (*link_rx_cb_t)(const uint8_t *data, size_t len, void *user_ctx);
typedef void (*link_frame_cb_t)(link_msg_type_t type, const uint8_t *payload, size_t len, void *user_ctx);

// Initialize link layer. Will attempt ESP-NOW first (stub for now), fall back to Wi-Fi/UDP.
esp_err_t link_init(const link_config_t *cfg);

// Returns active path in use.
link_path_t link_active_path(void);
const char *link_path_name(link_path_t p);

// Send arbitrary payload (info/message). Best-effort; uses active path with fallback.
esp_err_t link_send(const uint8_t *data, size_t len);

// Optional receive callback. If set, ESP-NOW frames are delivered; when Wi-Fi is
// active, a UDP listener on pc_port delivers payloads as well.
esp_err_t link_set_rx(link_rx_cb_t cb, void *user_ctx);

// Framed send with optional ACK on ESP-NOW. Flags: ack requested only if supported.
esp_err_t link_send_frame(link_msg_type_t type, const uint8_t *payload, size_t len, bool require_ack);
esp_err_t link_set_frame_rx(link_frame_cb_t cb, void *user_ctx);

// Optional periodic info broadcast using system_state snapshot (battery, LED mode, connection, time).
esp_err_t link_start_info_broadcast(uint32_t period_ms);

#ifdef __cplusplus
}
#endif
