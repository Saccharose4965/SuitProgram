#include "app_shell.h"
#include "shell_apps.h"
#include "app_led_support.h"
#include "arm_pose.h"
#include "orientation_service.h"
#include "led.h"
#include "led_layout.h"
#include "show_geometry.h"
#include "oled.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

// Explicit opt-in Canvas, front surface only. No world-position estimation.
enum { PAINT_MENU, PAINT_PAINTER, PAINT_CANVAS, PAINT_VERSION=0x51 };
typedef struct {
    uint8_t version, active, color, reserved;
    uint32_t session, sequence, stroke;
    int16_t x,y;
} paint_packet_t;
typedef struct { paint_packet_t p; uint8_t mac[6]; int64_t received; } paint_rx_t;
static StaticQueue_t s_queue_storage;
static uint8_t s_queue_bytes[8*sizeof(paint_rx_t)];
static QueueHandle_t s_queue;
static portMUX_TYPE s_rx_mux=portMUX_INITIALIZER_UNLOCKED;
static bool s_accept;
static int s_mode, s_choice;
static arm_pose_t s_pose;
static bool s_painting, s_on_canvas;
static bool s_left_arm = true;
static uint8_t s_color;
static uint32_t s_session,s_sequence,s_stroke;
static int64_t s_last_send,s_entered,s_last_rx;
static float s_x,s_y,s_previous_x,s_previous_y;
static uint8_t s_peer[6];
static uint32_t s_peer_session,s_peer_sequence,s_peer_stroke;
static bool s_have_previous;
static size_t s_count;
static EXT_RAM_BSS_ATTR float s_xy[LED_LAYOUT_MAX_PIXELS][2];
static EXT_RAM_BSS_ATTR bool s_front[LED_LAYOUT_MAX_PIXELS];
static EXT_RAM_BSS_ATTR float s_rgb[LED_LAYOUT_MAX_PIXELS][3];
static EXT_RAM_BSS_ATTR uint8_t s_frame[LED_LAYOUT_MAX_PIXELS*3];
static const uint8_t s_colors[][3]={{0,180,255},{255,50,10},{160,30,255},{20,255,70},{255,255,255}};
static const char *s_error;

const shell_legend_t LIGHT_PAINT_LEGEND={
    .slots={SHELL_ICON_LEFT,SHELL_ICON_REFRESH,SHELL_ICON_RIGHT,SHELL_ICON_SELECT}
};

void light_paint_receive(link_msg_type_t type,const uint8_t *mac,const uint8_t *data,size_t len)
{
    if(type!=LINK_MSG_CONTROL || !mac || !data || len!=sizeof(paint_packet_t)) return;
    paint_rx_t rx={.received=esp_timer_get_time()};
    memcpy(&rx.p,data,len); memcpy(rx.mac,mac,6);
    if(rx.p.version!=PAINT_VERSION || rx.p.active>1 || rx.p.color>=5 || rx.p.reserved ||
       !rx.p.session || rx.p.x < -10000 || rx.p.x>10000 || rx.p.y < -10000 || rx.p.y>10000) return;
    portENTER_CRITICAL(&s_rx_mux);
    bool accept=s_accept;
    QueueHandle_t queue=s_queue;
    portEXIT_CRITICAL(&s_rx_mux);
    if(accept && queue) (void)xQueueSend(queue,&rx,0);
}

static void leave_canvas(void)
{
    portENTER_CRITICAL(&s_rx_mux); s_accept=false; portEXIT_CRITICAL(&s_rx_mux);
    if(s_mode==PAINT_CANVAS) {
        memset(s_frame,0,sizeof(s_frame));
        (void)led_show_pixels_owned(LED_OUTPUT_OWNER_PAINT,s_frame,s_count);
        led_output_release(LED_OUTPUT_OWNER_PAINT);
    }
    s_painting=false; s_on_canvas=false;
}

static bool prepare_canvas(void)
{
    led_layout_config_t *cfg=heap_caps_malloc(sizeof(*cfg),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!cfg) { s_error="No memory"; return false; }
    led_layout_snapshot(cfg);
    s_count=cfg->total_leds;
    memset(s_front,0,sizeof(s_front)); memset(s_rgb,0,sizeof(s_rgb));
    float minx=INFINITY,maxx=-INFINITY,miny=INFINITY,maxy=-INFINITY;
    for(size_t s=0;s<cfg->section_count;s++) {
        const led_layout_section_t *section=&cfg->sections[s];
        bool front=!strncmp(section->name,"front_",6) || strstr(section->name,"thigh_front") ||
            strstr(section->name,"shin_f_") || strstr(section->name,"arm");
        show_section_id_t id;
        if(!front || !show_section_id_from_name(section->name,&id)) continue;
        for(size_t j=0;j<section->length;j++) {
            size_t i=section->logical_start+j; led_point_t p;
            if(i>=LED_LAYOUT_MAX_PIXELS || !led_layout_get_from_config(cfg,i,&p)) continue;
            p=show_geometry_pose(id,p,section->length>1?(float)j/(section->length-1):.5f);
            if(!isfinite(p.x) || !isfinite(p.y)) continue;
            s_front[i]=true; s_xy[i][0]=p.x; s_xy[i][1]=p.y;
            minx=fminf(minx,p.x); maxx=fmaxf(maxx,p.x);
            miny=fminf(miny,p.y); maxy=fmaxf(maxy,p.y);
        }
    }
    heap_caps_free(cfg);
    if(!s_count || s_count>LED_LAYOUT_MAX_PIXELS || maxx-minx<1e-4f || maxy-miny<1e-4f) {
        s_error="No front LED layout"; return false;
    }
    for(size_t i=0;i<s_count;i++) if(s_front[i]) {
        // Geometry is in cm: body centreline and belt/waist are fixed
        // anatomical anchors, independent of which LED sections exist.
        s_xy[i][0]=s_xy[i][0]/30.0f;
        s_xy[i][1]=(s_xy[i][1]-33.5f)/75.0f;
    }
    if(led_output_claim(LED_OUTPUT_OWNER_PAINT)!=ESP_OK) {s_error="LEDs busy (show)";return false;}
    xQueueReset(s_queue); s_have_previous=false; s_last_rx=0; s_entered=esp_timer_get_time();
    portENTER_CRITICAL(&s_rx_mux); s_accept=true; portEXIT_CRITICAL(&s_rx_mux);
    return true;
}

void light_paint_init(shell_app_context_t *ctx)
{
    (void)ctx;
    led_ui_fft_set_paint_app(true);
    if(!s_queue) s_queue=xQueueCreateStatic(8,sizeof(paint_rx_t),s_queue_bytes,&s_queue_storage);
    s_mode=PAINT_MENU; s_choice=0; s_error=NULL; s_pose.calibrated=false;
    s_painting=false; s_on_canvas=false; s_color=0;
    s_session=esp_random(); if(!s_session) s_session=1;
    s_sequence=s_stroke=0; s_last_send=0;
}
void light_paint_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    leave_canvas();
    s_mode=PAINT_MENU;
    led_ui_fft_set_paint_app(false);
}

void light_paint_input(shell_app_context_t *ctx,const input_event_t *ev)
{
    if(!ev || ev->type!=INPUT_EVENT_PRESS) return;
    s_error=NULL;
    if(ev->button==INPUT_BTN_A && s_mode!=PAINT_MENU) {
        leave_canvas();s_mode=PAINT_MENU;
        return;
    }
    if(s_mode==PAINT_MENU) {
        if(ev->button==INPUT_BTN_A && s_choice>0) --s_choice;
        if(ev->button==INPUT_BTN_B && s_choice<3) ++s_choice;
        if(ev->button==INPUT_BTN_D) {
            if(s_choice==3) {if(ctx && ctx->request_switch) ctx->request_switch("menu_sim",ctx->request_user_data);}
            else if(s_choice==2) {
                s_left_arm=!s_left_arm;
                s_pose.calibrated=false; s_painting=false; s_on_canvas=false; ++s_stroke;
            }
            else if(s_choice==1) {if(s_queue && prepare_canvas()) s_mode=PAINT_CANVAS;}
            else {s_mode=PAINT_PAINTER;s_pose.calibrated=false;s_painting=false;}
        }
    } else if(s_mode==PAINT_PAINTER) {
        if(ev->button==INPUT_BTN_B) {
            imu_quat_t q; uint64_t t;
            s_painting=false; ++s_stroke;
            if(orientation_service_snapshot(&q,&t) && esp_timer_get_time()-(int64_t)t<250000) {
                if(!arm_pose_calibrate_heading(&s_pose,q)) s_error="Hold forearm horizontal";
            } else s_error="Waiting for IMU";
        } else if(ev->button==INPUT_BTN_C) {s_color=(s_color+1)%5;++s_stroke;}
        else if(ev->button==INPUT_BTN_D && s_pose.calibrated) {s_painting=!s_painting;++s_stroke;}
    } else if(ev->button==INPUT_BTN_B) {memset(s_rgb,0,sizeof(s_rgb));s_have_previous=false;}
}

void light_paint_tick(shell_app_context_t *ctx,float dt)
{
    (void)ctx;
    int64_t now=esp_timer_get_time();
    if(s_mode==PAINT_PAINTER) {
        imu_quat_t q; uint64_t t; arm_vec3_t d;
        s_on_canvas=orientation_service_snapshot(&q,&t) && now-(int64_t)t<250000 &&
            arm_pose_direction(&s_pose,q,s_left_arm,&d) && arm_pose_canvas(d,&s_x,&s_y);
        if(now-s_last_send<50000) return; // at most 20 Hz, no per-LED network data
        s_last_send=now;
        paint_packet_t p={.version=PAINT_VERSION,.active=s_painting&&s_on_canvas,.color=s_color,
            .session=s_session,.sequence=++s_sequence,.stroke=s_stroke,
            .x=(int16_t)lroundf(s_x*10000),.y=(int16_t)lroundf(s_y*10000)};
        static const uint8_t broadcast[6]={255,255,255,255,255,255};
        if(link_send_frame_to(broadcast,LINK_MSG_CONTROL,(const uint8_t*)&p,sizeof(p),false)!=ESP_OK)
            s_error="Link unavailable";
        else s_error=NULL;
    } else if(s_mode==PAINT_CANVAS) {
        float fade=expf(-fmaxf(0,dt)/1.5f);
        for(size_t i=0;i<s_count;i++) for(int c=0;c<3;c++) s_rgb[i][c]*=fade;
        paint_rx_t rx;
        while(xQueueReceive(s_queue,&rx,0)==pdTRUE) {
            if(rx.received<s_entered || now-rx.received>250000) continue;
            bool same=!memcmp(s_peer,rx.mac,6) && s_peer_session==rx.p.session;
            if(s_last_rx && rx.received-s_last_rx<2000000 && !same) continue;
            if(same && s_last_rx && (int32_t)(rx.p.sequence-s_peer_sequence)<=0) continue;
            bool join=same && s_have_previous && rx.p.stroke==s_peer_stroke && rx.received-s_last_rx<200000;
            memcpy(s_peer,rx.mac,6);s_peer_session=rx.p.session;s_peer_sequence=rx.p.sequence;
            s_peer_stroke=rx.p.stroke;s_last_rx=rx.received;
            s_x=rx.p.x/10000.0f;s_y=rx.p.y/10000.0f;
            if(rx.p.active) {
                for(size_t i=0;i<s_count;i++) if(s_front[i]) {
                    float distance=arm_brush_distance(s_xy[i][0],s_xy[i][1],
                        join?s_previous_x:s_x,join?s_previous_y:s_y,s_x,s_y);
                    float strength=fmaxf(0,1-distance/.12f);
                    for(int c=0;c<3;c++) s_rgb[i][c]=fmaxf(s_rgb[i][c],strength*s_colors[rx.p.color][c]);
                }
            }
            s_have_previous=rx.p.active;s_previous_x=s_x;s_previous_y=s_y;
        }
        for(size_t i=0;i<s_count;i++) led_set_pixel_rgb(s_frame,i,
            (uint8_t)s_rgb[i][0],(uint8_t)s_rgb[i][1],(uint8_t)s_rgb[i][2]);
        (void)led_show_pixels_owned(LED_OUTPUT_OWNER_PAINT,s_frame,s_count);
    }
}

void light_paint_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx;(void)w;(void)h;
    const char *title=s_mode==PAINT_MENU?"Light painting":s_mode==PAINT_PAINTER?
        (s_left_arm?"Painter (left arm)":"Painter (right arm)"):"Canvas - fading trails";
    oled_draw_text3x5(fb,x+2,y+2,title);
    if(s_mode==PAINT_MENU) {
        oled_draw_text3x5(fb,x+2,y+14,s_choice==0?"> Painter":"  Painter");
        oled_draw_text3x5(fb,x+2,y+24,s_choice==1?"> Canvas":"  Canvas");
        oled_draw_text3x5(fb,x+2,y+34,s_left_arm?
            (s_choice==2?"> Arm: left":"  Arm: left"):
            (s_choice==2?"> Arm: right":"  Arm: right"));
        oled_draw_text3x5(fb,x+2,y+44,s_choice==3?"> Back":"  Back");
    } else if(s_mode==PAINT_PAINTER) {
        oled_draw_text3x5(fb,x+2,y+13,!s_pose.calibrated?"Fold arm; B calibrates":s_on_canvas?"Aiming at canvas":"Unfold / aim forward");
        oled_draw_text3x5(fb,x+2,y+23,s_painting?"D: paint OFF (ON now)":"D: paint ON (OFF now)");
        static const char *names[]={"C: cyan","C: orange","C: violet","C: green","C: white"};
        oled_draw_text3x5(fb,x+2,y+33,names[s_color]);
    } else {
        oled_draw_text3x5(fb,x+2,y+14,s_last_rx && esp_timer_get_time()-s_last_rx<2000000?"Painter connected":"Waiting for painter");
        oled_draw_text3x5(fb,x+2,y+25,"B clear / A back");
    }
    if(s_error) oled_draw_text3x5(fb,x+2,y+43,s_error);
}
