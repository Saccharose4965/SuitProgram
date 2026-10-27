#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool following;
    bool linked;
    bool role_ready;
    uint8_t local_role;
    uint8_t source_role;
  float bpm;
  int age_ms;
} live_sync_status_t;

/* Called from the shell task. Publishes local lighting or advances a follower.
 */
void live_sync_service_tick(float dt_sec);

/* Safe in Link's receive callback; packet work is applied from service_tick. */
void live_sync_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac,
                                 const uint8_t *payload, size_t len);

bool live_sync_blocks_app_tick(const char *app_id);
void live_sync_get_status(live_sync_status_t *out);

#ifdef __cplusplus
}
#endif
