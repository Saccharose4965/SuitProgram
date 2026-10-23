#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "link.h"

typedef struct {
    bool active;
    bool talking;
    esp_err_t error;
    uint32_t sent;
    uint32_t received;
} call_service_status_t;

esp_err_t call_service_start(void);
void call_service_stop(void);
void call_service_toggle_talk(void);
call_service_status_t call_service_get_status(void);
void call_service_receive(link_msg_type_t type, const uint8_t *src,
                          const uint8_t *data, size_t len);
