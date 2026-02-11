#include "pong.h"
#include "oled.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define HUD_HEIGHT    8
#define LEGEND_HEIGHT 7
#define CONTENT_Y     (HUD_HEIGHT)
#define CONTENT_H     (PANEL_H - HUD_HEIGHT - LEGEND_HEIGHT)

typedef struct {
    float paddle_p1_y;
    float paddle_p2_y;
    float ball_x;
    float ball_y;
    float vx;
    float vy;
    int   p1_score;
    int   p2_score;
    bool  is_host;
    bool  paused;
    bool  waiting_peer;
    float tx_accum;
} pong_state_t;

static pong_state_t s_pong = {0};
static pong_request_switch_fn s_switch_cb = NULL;
static void *s_switch_user = NULL;

static inline float clampf_local(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

static inline void fb_pset_local(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill_local(uint8_t *fb, int x0, int y0, int w, int h)
{
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            fb_pset_local(fb, x, y);
        }
    }
}

void pong_set_request_switch(pong_request_switch_fn fn, void *user_data)
{
    s_switch_cb = fn;
    s_switch_user = user_data;
}

static void pong_reset_ball(bool to_p1){
    s_pong.ball_x = PANEL_W / 2.0f;
    s_pong.ball_y = PANEL_H / 2.0f;
    float speed = 60.0f;
    s_pong.vx = to_p1 ? -speed : speed;
    s_pong.vy = (float)(rand() % 40 - 20);
}

static void pong_send_role(bool host_flag){
    struct __attribute__((packed)) {
        uint8_t game;
        uint8_t kind;
        uint8_t host;
    } pkt = { .game = 1, .kind = 3, .host = host_flag ? 1 : 0 };
    bool ack = (link_active_path() == LINK_PATH_ESPNOW);
    link_send_frame(LINK_MSG_GAME, (const uint8_t*)&pkt, sizeof(pkt), ack);
}

static void pong_send_paddle(void){
    struct __attribute__((packed)) {
        uint8_t game;
        uint8_t kind;
        float   y;
    } pkt = { .game = 1, .kind = 1, .y = s_pong.paddle_p1_y };
    bool ack = (link_active_path() == LINK_PATH_ESPNOW);
    link_send_frame(LINK_MSG_GAME, (const uint8_t*)&pkt, sizeof(pkt), ack);
}

void pong_handle_link_frame(link_msg_type_t type, const uint8_t *payload, size_t len)
{
    if (type != LINK_MSG_GAME || !payload || len < 2) return;
    uint8_t game = payload[0];
    uint8_t kind = payload[1];
    if (game != 1) return;

    if (kind == 1 && len >= 6){
        float y = 0.0f;
        memcpy(&y, payload + 2, sizeof(float));
        s_pong.paddle_p2_y = clampf_local(y, (float)CONTENT_Y, (float)(CONTENT_Y + CONTENT_H - 14));
    } else if (kind == 2 && len >= 22){
        memcpy(&s_pong.ball_x, payload + 2, sizeof(float));
        memcpy(&s_pong.ball_y, payload + 6, sizeof(float));
        memcpy(&s_pong.vx,     payload + 10, sizeof(float));
        memcpy(&s_pong.vy,     payload + 14, sizeof(float));
        int16_t s1=0,s2=0;
        memcpy(&s1, payload + 18, sizeof(int16_t));
        memcpy(&s2, payload + 20, sizeof(int16_t));
        s_pong.p1_score = s1;
        s_pong.p2_score = s2;
        s_pong.is_host = false;
    } else if (kind == 3 && len >= 3){
        bool remote_host = payload[2] ? true : false;
        s_pong.is_host = !remote_host;
    }
}

void pong_app_init(void){
    s_pong.paddle_p1_y = PANEL_H / 2.0f - 8;
    s_pong.paddle_p2_y = PANEL_H / 2.0f - 8;
    s_pong.p1_score = 0;
    s_pong.p2_score = 0;
    s_pong.paused = false;
    s_pong.waiting_peer = false; // placeholder: would flip until remote syncs
    s_pong.is_host = true;
    s_pong.tx_accum = 0.0f;
    pong_reset_ball(false);
    pong_send_role(s_pong.is_host);
}

void pong_app_handle_input(const input_event_t *ev){
    if (!ev) return;
    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) {
            s_pong.paused = !s_pong.paused;
        }
        if (ev->button == INPUT_BTN_B) {
            s_pong.paddle_p1_y -= 4.0f;
            pong_send_paddle();
        } else if (ev->button == INPUT_BTN_C) {
            s_pong.paddle_p1_y += 4.0f;
            pong_send_paddle();
        } else if (ev->button == INPUT_BTN_D) {
            s_pong.is_host = !s_pong.is_host;
            if (s_pong.is_host) {
                pong_reset_ball(false);
            }
            pong_send_role(s_pong.is_host);
        }
    }
}

void pong_app_tick(float dt_sec){
    if (s_pong.paused) return;
    const float paddle_h = 14.0f;
    const float paddle_w = 2.0f;
    const float ball_r = 2.0f;

    // Clamp paddles
    if (s_pong.paddle_p1_y < CONTENT_Y) s_pong.paddle_p1_y = CONTENT_Y;
    if (s_pong.paddle_p1_y + paddle_h > CONTENT_Y + CONTENT_H) s_pong.paddle_p1_y = CONTENT_Y + CONTENT_H - paddle_h;

    // Paddle 2 position is driven by remote updates; just clamp it.
    if (s_pong.paddle_p2_y < CONTENT_Y) s_pong.paddle_p2_y = CONTENT_Y;
    if (s_pong.paddle_p2_y + paddle_h > CONTENT_Y + CONTENT_H) s_pong.paddle_p2_y = CONTENT_Y + CONTENT_H - paddle_h;

    // Host advances ball and broadcasts state; follower just draws latest state.
    if (!s_pong.is_host) return;

    // Move ball
    s_pong.ball_x += s_pong.vx * dt_sec;
    s_pong.ball_y += s_pong.vy * dt_sec;

    // Walls
    if (s_pong.ball_y - ball_r < CONTENT_Y) { s_pong.ball_y = CONTENT_Y + ball_r; s_pong.vy = -s_pong.vy; }
    if (s_pong.ball_y + ball_r > CONTENT_Y + CONTENT_H) { s_pong.ball_y = CONTENT_Y + CONTENT_H - ball_r; s_pong.vy = -s_pong.vy; }

    // Player paddle (left)
    float p_x = 4.0f;
    if (s_pong.ball_x - ball_r < p_x + paddle_w &&
        s_pong.ball_x - ball_r > p_x &&
        s_pong.ball_y > s_pong.paddle_p1_y - ball_r &&
        s_pong.ball_y < s_pong.paddle_p1_y + paddle_h + ball_r){
        s_pong.ball_x = p_x + paddle_w + ball_r;
        s_pong.vx = fabsf(s_pong.vx);
        float offset = (s_pong.ball_y - (s_pong.paddle_p1_y + paddle_h * 0.5f)) / (paddle_h * 0.5f);
        s_pong.vy += offset * 30.0f;
    }

    // Player 2 paddle (right)
    float p2_x = PANEL_W - 6.0f;
    if (s_pong.ball_x + ball_r > p2_x &&
        s_pong.ball_x + ball_r < p2_x + paddle_w &&
        s_pong.ball_y > s_pong.paddle_p2_y - ball_r &&
        s_pong.ball_y < s_pong.paddle_p2_y + paddle_h + ball_r){
        s_pong.ball_x = p2_x - ball_r;
        s_pong.vx = -fabsf(s_pong.vx);
        float offset = (s_pong.ball_y - (s_pong.paddle_p2_y + paddle_h * 0.5f)) / (paddle_h * 0.5f);
        s_pong.vy += offset * 30.0f;
    }

    // Scoring
    if (s_pong.ball_x < 0) {
        s_pong.p2_score++;
        pong_reset_ball(false);
    } else if (s_pong.ball_x > PANEL_W) {
        s_pong.p1_score++;
        pong_reset_ball(true);
    }

    // Periodic state broadcast
    s_pong.tx_accum += dt_sec;
    if (s_pong.tx_accum >= 0.05f){ // ~20 Hz
        s_pong.tx_accum = 0.0f;
        struct __attribute__((packed)) {
            uint8_t game;
            uint8_t kind;
            float bx, by, vx, vy;
            int16_t s1, s2;
        } pkt = {
            .game = 1, .kind = 2,
            .bx = s_pong.ball_x, .by = s_pong.ball_y,
            .vx = s_pong.vx, .vy = s_pong.vy,
            .s1 = (int16_t)s_pong.p1_score,
            .s2 = (int16_t)s_pong.p2_score,
        };
        bool ack = (link_active_path() == LINK_PATH_ESPNOW);
        link_send_frame(LINK_MSG_GAME, (const uint8_t*)&pkt, sizeof(pkt), ack);
    }
}

void pong_app_draw(uint8_t *fb, int x, int y, int w, int h){
    (void)x; (void)y; (void)w; (void)h;
    const int paddle_h = 14;
    const int paddle_w = 2;
    // Center line
    for (int yy = CONTENT_Y; yy < CONTENT_Y + CONTENT_H; yy += 4){
        fb_pset_local(fb, PANEL_W / 2, yy);
        fb_pset_local(fb, PANEL_W / 2, yy + 1);
    }
    // Paddles
    fb_rect_fill_local(fb, 4, (int)s_pong.paddle_p1_y, paddle_w, paddle_h);
    fb_rect_fill_local(fb, PANEL_W - 6, (int)s_pong.paddle_p2_y, paddle_w, paddle_h);
    // Ball
    fb_rect_fill_local(fb, (int)(s_pong.ball_x - 1), (int)(s_pong.ball_y - 1), 3, 3);
    // Score
    char score[24];
    snprintf(score, sizeof(score), "%d - %d %s", s_pong.p1_score, s_pong.p2_score, s_pong.is_host ? "H" : "C");
    oled_draw_text3x5(fb, PANEL_W / 2 - 10, CONTENT_Y + 2, score);
    if (s_pong.paused){
        oled_draw_text3x5(fb, PANEL_W / 2 - 10, CONTENT_Y + 10, "PAUSE");
    }
}
