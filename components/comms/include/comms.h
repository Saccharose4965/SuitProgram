#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ssid;
    const char *pass;
    const char *peer_ip;   // other ESP IP (or broadcast, but prefer unicast)
    uint16_t    peer_port; // telemetry/message port
    uint16_t    call_port; // UDP port for live call stream
} comms_cfg_t;

// Start comms: brings up telemetry, call TX/RX (if hardware present),
// and a control loop that lets button A send the last recording.
void comms_start(const comms_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
