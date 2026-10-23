#include "app_shell.h"
#include "ui_draw.h"
#include "gyro_calibration.h"
#include "oled.h"
#include <stdio.h>

void gyro_app_init(shell_app_context_t *ctx) { (void)ctx;gyro_calibration_cancel(); }
void gyro_app_deinit(shell_app_context_t *ctx) { (void)ctx;gyro_calibration_cancel(); }
void gyro_app_tick(shell_app_context_t *ctx,float dt) { (void)ctx;(void)dt;gyro_calibration_tick(); }
void gyro_app_input(shell_app_context_t *ctx,const input_event_t *ev)
{
    if(!ev || ev->type!=INPUT_EVENT_PRESS) return;
    if(ev->button==INPUT_BTN_A) {
        gyro_calibration_cancel();
        if(ctx && ctx->request_switch) ctx->request_switch("menu_settings",ctx->request_user_data);
    } else if(ev->button==INPUT_BTN_D) {
        gyro_calibration_state_t state=gyro_calibration_status().state;
        if(state==GYRO_READY || state==GYRO_SAVE_FAILED) (void)gyro_calibration_save();
        else if(state!=GYRO_SETTLING && state!=GYRO_SAMPLING) gyro_calibration_begin();
    }
}
void gyro_app_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx;(void)w;(void)h;
    gyro_calibration_status_t status=gyro_calibration_status();
    char line[32]; const char *text="D start / A back";
    switch(status.state) {
    case GYRO_SETTLING: text="Settling... hold still";break;
    case GYRO_SAMPLING:
        snprintf(line,sizeof(line),"Measuring %d%%",(int)(status.progress*100));text=line;break;
    case GYRO_READY: text="Ready: D save / A back";break;
    case GYRO_MOVED: text="Movement: D retry";break;
    case GYRO_NO_DATA: text="No IMU data: D retry";break;
    case GYRO_SAVED: text="Saved / A back";break;
    case GYRO_SAVE_FAILED: text="SD error: D retry save";break;
    default: break;
    }
    oled_draw_text3x5(fb,x+2,y+2,"Gyro calibration");
    oled_draw_text3x5(fb,x+2,y+13,"Rest sensor on table");
    oled_draw_text3x5(fb,x+2,y+24,text);
    if(status.state==GYRO_READY || status.state==GYRO_SAVED || status.state==GYRO_SAVE_FAILED) {
        snprintf(line,sizeof(line),"%.2f %.2f %.2f d/s",status.bias[0],status.bias[1],status.bias[2]);
        oled_draw_text3x5(fb,x+2,y+35,line);
    }
}
