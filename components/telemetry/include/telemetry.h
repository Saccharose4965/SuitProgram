#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char* ssid;
    const char* pass;
    const char* pc_ip;    // e.g. "192.168.1.123" (LAN IP)
    uint16_t    pc_port;  // e.g. 9000
} telemetry_cfg_t;

esp_err_t telemetry_start(const telemetry_cfg_t* cfg);  // Wi-Fi STA + UDP sockets
esp_err_t telemetry_send_line(const char* line);        // Send ASCII line
esp_err_t telemetry_send_mpu_sample(float ax,float ay,float az,
                                    float gx,float gy,float gz,
                                    float temp_c, uint64_t t_us);

typedef void (*telemetry_progress_cb_t)(size_t sent, size_t total, void *user_ctx);

// Send a file over UDP to a peer (best-effort + final ACK). Path should be
// accessible (e.g., last audio recording). peer_ip is dotted decimal, port the
// peer listens on. Optional progress callback per chunk.
esp_err_t telemetry_send_file_udp_ex(const char *path,
                                     const char *peer_ip,
                                     uint16_t peer_port,
                                     telemetry_progress_cb_t cb,
                                     void *user_ctx);

// Convenience wrapper without progress/ctx
static inline esp_err_t telemetry_send_file_udp(const char *path,
                                                const char *peer_ip,
                                                uint16_t peer_port){
    return telemetry_send_file_udp_ex(path, peer_ip, peer_port, NULL, NULL);
}

// Start a simple UDP file receiver (best-effort). Saves incoming files to the
// given directory. Call after telemetry_start(). Runs until reboot.
esp_err_t telemetry_start_file_receiver(uint16_t listen_port, const char *save_dir);

// Stream an in-memory buffer to a peer over UDP (best-effort + final ACK).
// Useful for real-time “call”-style audio without SD. Buffer must stay valid
// for the duration of the send.
esp_err_t telemetry_send_buffer_udp(const uint8_t *data, size_t len,
                                    const char *peer_ip, uint16_t peer_port,
                                    telemetry_progress_cb_t cb,
                                    void *user_ctx);

#ifdef __cplusplus
}
#endif
