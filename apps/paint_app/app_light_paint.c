#include "app_shell.h"
#include "ui_draw.h"
#include "audio_policy.h"
#include "paint_service.h"
#include "oled.h"

static int s_choice;
const shell_legend_t LIGHT_PAINT_LEGEND={
    .slots={SHELL_ICON_LEFT,SHELL_ICON_REFRESH,SHELL_ICON_RIGHT,SHELL_ICON_SELECT}
};
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
    oled_draw_text3x5(fb,x+2,y+2,title);
    if(status.mode==PAINT_MENU) {
        oled_draw_text3x5(fb,x+2,y+14,s_choice==0?"> Painter":"  Painter");
        oled_draw_text3x5(fb,x+2,y+24,s_choice==1?"> Canvas":"  Canvas");
        oled_draw_text3x5(fb,x+2,y+34,status.left_arm?
            (s_choice==2?"> Arm: left":"  Arm: left"):
            (s_choice==2?"> Arm: right":"  Arm: right"));
        oled_draw_text3x5(fb,x+2,y+44,s_choice==3?"> Back":"  Back");
    } else if(status.mode==PAINT_PAINTER) {
        oled_draw_text3x5(fb,x+2,y+13,!status.calibrated?"Fold arm; B calibrates":status.on_canvas?"Aiming at canvas":"Unfold / aim forward");
        oled_draw_text3x5(fb,x+2,y+23,status.painting?"D: paint OFF (ON now)":"D: paint ON (OFF now)");
        static const char *names[]={"C: cyan","C: orange","C: violet","C: green","C: white"};
        oled_draw_text3x5(fb,x+2,y+33,names[status.color]);
    } else {
        oled_draw_text3x5(fb,x+2,y+14,status.connected?"Painter connected":"Waiting for painter");
        oled_draw_text3x5(fb,x+2,y+25,"B clear / A back");
    }
    if(status.error!=PAINT_OK) oled_draw_text3x5(fb,x+2,y+43,error_text(status.error));
}
