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
static shell_menu_motion_t s_menu_motion;
static const shell_menu_entry_t s_menu_entries[]={
    {.id="ai",.label="AI"},
    {.id="human",.label="Human"},
};

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

shell_legend_t PONG_LEGEND;

static void update_legend(pong_view_mode_t mode)
{
    if (mode==PONG_VIEW_MENU) {
        PONG_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_NONE,SHELL_ICON_SELECT},
        };
    } else if (mode==PONG_VIEW_AI) {
        PONG_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_NONE,SHELL_ICON_PAUSE},
        };
    } else if (mode==PONG_VIEW_HUMAN) {
        PONG_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_NONE,SHELL_ICON_NONE},
        };
    } else {
        PONG_LEGEND=(shell_legend_t){.slots={SHELL_ICON_NONE}};
    }
}

void pong_app_init_wrapper(shell_app_context_t *ctx)
{
    if (!ctx) return;
    s_selection=0;
    pong_service_init();
    shell_ui_menu_reset(&s_menu_motion,s_selection);
    update_legend(PONG_VIEW_MENU);
}

void pong_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type!=INPUT_EVENT_PRESS) return;
    pong_view_t view; pong_service_snapshot(&view);
    if (view.mode==PONG_VIEW_MENU) {
        if (ev->button==INPUT_BTN_A && s_selection>0) --s_selection;
        if (ev->button==INPUT_BTN_B && s_selection<1) ++s_selection;
        if (ev->button==INPUT_BTN_D) {
            if (s_selection) pong_service_start_human();
            else pong_service_start_ai();
            pong_service_snapshot(&view);
            update_legend(view.mode);
        }
        return;
    }
    if (view.mode==PONG_VIEW_AI || view.mode==PONG_VIEW_HUMAN) {
        if (ev->button==INPUT_BTN_D && view.mode==PONG_VIEW_AI) pong_service_toggle_pause();
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
    pong_view_t view;
    pong_service_snapshot(&view);
    int paddle_input=0;
    if(view.mode==PONG_VIEW_AI || view.mode==PONG_VIEW_HUMAN) {
        input_button_t held=input_current_button();
        if(held==INPUT_BTN_A) paddle_input=-1;
        else if(held==INPUT_BTN_B) paddle_input=1;
    }
    pong_service_set_paddle_input(paddle_input);
    pong_service_tick();
    pong_service_snapshot(&view);
    update_legend(view.mode);
    if(view.mode==PONG_VIEW_MENU)
        shell_ui_menu_tick(&s_menu_motion,dt_sec,s_selection);
}

void pong_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb || w<20 || h<16) return;
    pong_view_t view; pong_service_snapshot(&view);
    if (view.mode==PONG_VIEW_MENU) {
        shell_menu_entry_t entries[2]={s_menu_entries[0],s_menu_entries[1]};
        if(!view.human_available) entries[1].label="Human unavailable";
        shell_menu_view_t menu={
            .entries=entries,.count=2,.selected=s_selection,.title="PONG",
            .motion=&s_menu_motion,
        };
        shell_ui_draw_menu(fb,x,y,w,h,&menu);
        return;
    }
    if (view.mode!=PONG_VIEW_AI && view.mode!=PONG_VIEW_HUMAN) {
        oled_draw_text3x5(fb,x+3,y+8,view.mode==PONG_VIEW_WAITING ? "Waiting for player" : "Connecting...");
        return;
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
