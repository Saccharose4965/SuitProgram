#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SHOW_RUNTIME_STOPPED = 0,
    SHOW_RUNTIME_LOADED,
    SHOW_RUNTIME_PLAYING,
    SHOW_RUNTIME_PAUSED,
} show_runtime_state_t;

typedef struct {
    bool initialized;
    bool loaded;
    bool owns_leds;
    bool authority;
    bool synced;
    show_runtime_state_t state;
    uint8_t role_id;
    uint64_t show_uid;
    uint32_t duration_ms;
    uint32_t playhead_ms;
    uint16_t peer_count;
    uint16_t best_rtt_ms;
    char slug[32];
    char error[32];
} show_runtime_status_t;

esp_err_t show_runtime_init(void);
esp_err_t show_runtime_load(const char *show_slug, uint8_t role_id);
esp_err_t show_runtime_set_role(uint8_t role_id);
esp_err_t show_runtime_start_master(void);
esp_err_t show_runtime_toggle_master_pause(void);
void show_runtime_stop(bool broadcast);

/* Called from the shell task. Handles radio events, transport and rendering. */
void show_runtime_service_tick(float dt_sec);

/* Safe in the link receive callback; work is queued to the shell task. */
void show_runtime_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac,
                                    const uint8_t *payload, size_t len);

bool show_runtime_owns_leds(void);
show_runtime_status_t show_runtime_get_status(void);
const char *show_runtime_state_name(show_runtime_state_t state);

#ifdef __cplusplus
}
#endif
