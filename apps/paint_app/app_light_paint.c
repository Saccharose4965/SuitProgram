#include "app_shell.h"
#include "ui_draw.h"
#include "audio_policy.h"
#include "paint_service.h"
#include "oled.h"

static int s_choice;
static shell_menu_motion_t s_menu_motion;
shell_legend_t LIGHT_PAINT_LEGEND;

static void update_legend(paint_mode_t mode)
{
    if(mode==PAINT_MENU) {
        LIGHT_PAINT_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_NONE,SHELL_ICON_SELECT},
        };
    } else if(mode==PAINT_PAINTER) {
        LIGHT_PAINT_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_LEFT,SHELL_ICON_REFRESH,SHELL_ICON_RIGHT,SHELL_ICON_SELECT},
        };
    } else {
        LIGHT_PAINT_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_LEFT,SHELL_ICON_REFRESH,SHELL_ICON_NONE,SHELL_ICON_NONE},
        };
    }
}
static const char *error_text(paint_error_t error)
{
    switch(error) {
    case PAINT_NO_MEMORY: return "No memory";
    case PAINT_NO_LAYOUT: return "No front LED layout";
    case PAINT_LEDS_BUSY: return "LEDs busy (show)";
    case PAINT_NEED_HORIZONTAL: return "Hold forearm horizontal";
    case PAINT_IMU_UNAVAILABLE: return "Waiting for IMU";
    case PAINT_LINK_UNAVAILABLE: return "Link unavailable";
    default: return "";
    }
}
void light_paint_init(shell_app_context_t *ctx)
{
    (void)ctx;
    audio_policy_fft_set_paint_app(true);
    s_choice=0;
    paint_service_open();
    shell_ui_menu_reset(&s_menu_motion,s_choice);
    update_legend(PAINT_MENU);
}
void light_paint_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    paint_service_leave();
    audio_policy_fft_set_paint_app(false);
}
void light_paint_tick(shell_app_context_t *ctx,float dt)
{
    (void)ctx;
    paint_service_tick(dt);
    paint_status_t status=paint_service_status();
    update_legend(status.mode);
    if(status.mode==PAINT_MENU)
        shell_ui_menu_tick(&s_menu_motion,dt,(size_t)s_choice);
}
void light_paint_input(shell_app_context_t *ctx,const input_event_t *ev)
{
    if(!ev || ev->type!=INPUT_EVENT_PRESS) return;
    paint_service_command(PAINT_DISMISS_ERROR);
    paint_status_t status=paint_service_status();
    if(ev->button==INPUT_BTN_A && status.mode!=PAINT_MENU) {
        paint_service_command(PAINT_LEAVE);
        return;
    }
    if(status.mode==PAINT_MENU) {
        if(ev->button==INPUT_BTN_A && s_choice>0) --s_choice;
        if(ev->button==INPUT_BTN_B && s_choice<3) ++s_choice;
        if(ev->button==INPUT_BTN_D) {
            if(s_choice==3) {
                if(ctx && ctx->request_switch) ctx->request_switch("menu_sim",ctx->request_user_data);
            } else {
                static const paint_command_t commands[]={PAINT_SELECT_PAINTER,PAINT_SELECT_CANVAS,PAINT_TOGGLE_ARM};
                paint_service_command(commands[s_choice]);
            }
        }
    } else if(status.mode==PAINT_PAINTER) {
        if(ev->button==INPUT_BTN_B) paint_service_command(PAINT_CALIBRATE);
        else if(ev->button==INPUT_BTN_C) paint_service_command(PAINT_NEXT_COLOR);
        else if(ev->button==INPUT_BTN_D) paint_service_command(PAINT_TOGGLE_BRUSH);
    } else if(ev->button==INPUT_BTN_B) paint_service_command(PAINT_CLEAR);
}

void light_paint_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx;(void)w;(void)h;
    paint_status_t status=paint_service_status();
    const char *title=status.mode==PAINT_MENU?"Light painting":status.mode==PAINT_PAINTER?
        (status.left_arm?"Painter (left arm)":"Painter (right arm)"):"Canvas - fading trails";
    if(status.mode==PAINT_MENU) {
        const shell_menu_entry_t entries[]={
            {.id="painter",.label="Painter"},
            {.id="canvas",.label="Canvas"},
            {.id="arm",.label=status.left_arm?"Arm: left":"Arm: right"},
            {.id="back",.label="Back"},
        };
        shell_menu_view_t menu={
            .entries=entries,.count=4,.selected=(size_t)s_choice,.title=title,
            .motion=&s_menu_motion,
        };
        shell_ui_draw_menu(fb,x,y,w,h,&menu);
    } else if(status.mode==PAINT_PAINTER) {
        oled_draw_text3x5(fb,x+2,y+2,title);
        oled_draw_text3x5(fb,x+2,y+13,!status.calibrated?"Fold arm; B calibrates":status.on_canvas?"Aiming at canvas":"Unfold / aim forward");
        oled_draw_text3x5(fb,x+2,y+23,status.painting?"D: paint OFF (ON now)":"D: paint ON (OFF now)");
        static const char *names[]={"C: cyan","C: orange","C: violet","C: green","C: white"};
        oled_draw_text3x5(fb,x+2,y+33,names[status.color]);
    } else {
        oled_draw_text3x5(fb,x+2,y+2,title);
        oled_draw_text3x5(fb,x+2,y+14,status.connected?"Painter connected":"Waiting for painter");
        oled_draw_text3x5(fb,x+2,y+25,"B clear / A back");
    }
    if(status.error!=PAINT_OK) oled_draw_text3x5(fb,x+2,y+43,error_text(status.error));
}
