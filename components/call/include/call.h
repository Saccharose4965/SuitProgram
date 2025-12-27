#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALL_ROLE_RX = 0,
    CALL_ROLE_TX = 1,
} call_role_t;

typedef struct {
    const char *ssid;
    const char *pass;
    const char *peer_ip;   // receiver IP (or broadcast, but unicast preferred)
    uint16_t    peer_port; // telemetry hello port
    uint16_t    call_port; // UDP audio port
    call_role_t role;      // CALL_ROLE_RX or CALL_ROLE_TX
} call_cfg_t;

// Starts the call application in RX or TX role.
void call_start(const call_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
