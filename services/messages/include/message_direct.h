#pragma once
#include "link.h"
#include "telemetry.h"
esp_err_t message_direct_init(void);
void message_direct_receive(link_msg_type_t type, const uint8_t *mac, const uint8_t *data, size_t len);
esp_err_t message_direct_send(const char *path, telemetry_progress_cb_t cb, void *ctx);
