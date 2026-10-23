#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PONG_VIEW_MENU = 0,
    PONG_VIEW_WAITING,
    PONG_VIEW_CONNECTING,
    PONG_VIEW_HUMAN,
    PONG_VIEW_AI,
} pong_view_mode_t;

typedef struct {
    pong_view_mode_t mode;
    bool human_available;
    bool paused;
    uint8_t side;
    float paddle[2];
    float ball_x;
    float ball_y;
    uint16_t score[2];
} pong_view_t;

void pong_service_init(void);
void pong_service_deinit(void);
void pong_service_reset(void);
void pong_service_start_ai(void);
void pong_service_start_human(void);
void pong_service_move_paddle(int direction);
void pong_service_toggle_pause(void);
void pong_service_tick(void);
void pong_service_snapshot(pong_view_t *out);
bool pong_service_human_available(void);

void pong_handle_link_frame(link_msg_type_t type, const uint8_t *src_mac, const uint8_t *payload, size_t len);

#ifdef __cplusplus
}
#endif
