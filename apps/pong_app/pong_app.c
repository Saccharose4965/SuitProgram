#include "app_shell.h"

#include "pong.h"

#include <math.h>
#include <stdio.h>

#include "oled.h"
#include "ui_draw.h"

#define PONG_TOP 10
#define PONG_BOTTOM (PANEL_H - 9)
#define PONG_PADDLE_H 14
#define PONG_CENTER_X (PANEL_W / 2)
#define PONG_CENTER_Y ((PONG_TOP + PONG_BOTTOM) / 2)

static unsigned s_selection;

static void pixel(uint8_t *fb,int x,int y)
{
    if ((unsigned)x>=PANEL_W || (unsigned)y>=PANEL_H) return;
    int i=y*PANEL_W+x; fb[i>>3]|=(uint8_t)(1u<<(7-(i&7)));
}

static void rect(uint8_t *fb,int x,int y,int w,int h)
{
    for (int row=y;row<y+h;++row)
        for (int col=x;col<x+w;++col) pixel(fb,col,row);
}

const shell_legend_t PONG_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_SELECT },
};

void pong_app_init_wrapper(shell_app_context_t *ctx)
{
    if (!ctx) return;
    s_selection=0;
    pong_service_init();
}

void pong_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type!=INPUT_EVENT_PRESS) return;
    pong_view_t view; pong_service_snapshot(&view);
    if (ev->button==INPUT_BTN_A) {
        if (view.mode==PONG_VIEW_MENU) {
            if (ctx->request_switch) ctx->request_switch("menu",ctx->request_user_data);
        } else pong_service_reset();
        return;
    }
    if (view.mode==PONG_VIEW_MENU) {
        if (ev->button==INPUT_BTN_B || ev->button==INPUT_BTN_C) s_selection^=1u;
        if (ev->button==INPUT_BTN_D) {
            if (s_selection) pong_service_start_human();
            else pong_service_start_ai();
        }
        return;
    }
    if (view.mode==PONG_VIEW_AI || view.mode==PONG_VIEW_HUMAN) {
        if (ev->button==INPUT_BTN_B) pong_service_move_paddle(-1);
        if (ev->button==INPUT_BTN_C) pong_service_move_paddle(1);
        if (ev->button==INPUT_BTN_D) pong_service_toggle_pause();
    }
}

void pong_app_deinit_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    pong_service_deinit();
}

void pong_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;
    pong_service_tick();
}

void pong_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb || w<20 || h<16) return;
    pong_view_t view; pong_service_snapshot(&view);
    if (view.mode==PONG_VIEW_MENU) {
        oled_draw_text3x5(fb,x+4,y+2,"Pong");
        oled_draw_text3x5(fb,x+4,y+12,s_selection==0 ? "> AI" : "  AI");
        oled_draw_text3x5(fb,x+4,y+22,s_selection==1 ? "> Human" : "  Human");
        if (!view.human_available) oled_draw_text3x5(fb,x+4,y+32,"Human unavailable");
        return;
    }
    if (view.mode!=PONG_VIEW_AI && view.mode!=PONG_VIEW_HUMAN) {
        oled_draw_text3x5(fb,x+3,y+8,view.mode==PONG_VIEW_WAITING ? "Waiting for player" : "Connecting...");
        oled_draw_text3x5(fb,x+3,y+20,"A: back"); return;
    }
    for (int row=PONG_TOP;row<PONG_BOTTOM;row+=4) pixel(fb,PONG_CENTER_X,row);
    rect(fb,4,(int)view.paddle[0],2,PONG_PADDLE_H);
    rect(fb,PANEL_W-6,(int)view.paddle[1],2,PONG_PADDLE_H);
    rect(fb,(int)lroundf(view.ball_x)-1,(int)lroundf(view.ball_y)-1,3,3);
    char text[28]; snprintf(text,sizeof(text),"%u - %u",view.score[0],view.score[1]);
    oled_draw_text3x5(fb,PONG_CENTER_X-12,PONG_TOP,text);
    oled_draw_text3x5(fb,view.side ? PANEL_W-30 : 12,PONG_BOTTOM-5,"YOU");
    if (view.paused) oled_draw_text3x5(fb,PONG_CENTER_X-10,PONG_CENTER_Y,"PAUSE");
}
