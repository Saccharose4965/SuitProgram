#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default TCP listen port for incoming audio
#define AUDIO_RX_DEFAULT_PORT 5000

// Start the audio receiver server.
// - port: TCP port to listen on (use AUDIO_RX_DEFAULT_PORT for default)
// Requirements:
//   * Wi-Fi/network is already up and has an IP.
//   * SD card is already mounted at /sdcard.
esp_err_t audio_rx_start(int port);

// Stop the receiver server and clean up resources.
esp_err_t audio_rx_stop(void);

// Returns true if the server task is running.
bool audio_rx_is_running(void);

#ifdef __cplusplus
}
#endif
