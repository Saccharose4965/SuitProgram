#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "input.h"
#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    REMOTE_CONTROL_MAX_PEERS = 3,
    REMOTE_CONTROL_MAX_ENTRIES = 9,
    REMOTE_CONTROL_LABEL_LEN = 16,
    REMOTE_CONTROL_TITLE_LEN = 16,
    REMOTE_CONTROL_APP_ID_LEN = 16,
};

typedef enum {
    REMOTE_CONTROL_OFF = 0,
    REMOTE_CONTROL_CONTROLLER,
    REMOTE_CONTROL_TARGET,
} remote_control_role_t;

typedef enum {
    REMOTE_CONTROL_VIEW_NONE = 0,
    REMOTE_CONTROL_VIEW_MENU,
    REMOTE_CONTROL_VIEW_GRID,
} remote_control_view_kind_t;

typedef struct {
    uint8_t mac[6];
    int age_ms;
} remote_control_peer_t;

typedef struct {
    remote_control_view_kind_t kind;
    uint8_t total_count;
    uint8_t selected;
    uint8_t first;
    uint8_t entry_count;
    uint8_t legend[4];
    char app_id[REMOTE_CONTROL_APP_ID_LEN];
    char title[REMOTE_CONTROL_TITLE_LEN];
    char labels[REMOTE_CONTROL_MAX_ENTRIES][REMOTE_CONTROL_LABEL_LEN];
} remote_control_view_t;

typedef enum {
    REMOTE_CONTROL_COMMAND_NONE = 0,
    REMOTE_CONTROL_COMMAND_HOME,
    REMOTE_CONTROL_COMMAND_INPUT,
    REMOTE_CONTROL_COMMAND_END,
} remote_control_command_kind_t;

typedef struct {
    remote_control_command_kind_t kind;
    input_event_t input;
} remote_control_command_t;

void remote_control_receive(link_msg_type_t type, const uint8_t *src_mac,
                            const uint8_t *payload, size_t len);
void remote_control_service_tick(void);

size_t remote_control_get_peers(remote_control_peer_t *out, size_t capacity);
esp_err_t remote_control_begin(const uint8_t target_mac[6]);
void remote_control_end(void);
remote_control_role_t remote_control_role(void);
bool remote_control_active(void);
bool remote_control_get_view(remote_control_view_t *out);
esp_err_t remote_control_send_input(const input_event_t *event);
bool remote_control_poll_command(remote_control_command_t *out);
void remote_control_publish_view(const char *app_id,
                                 const remote_control_view_t *view);

#ifdef __cplusplus
}
#endif
