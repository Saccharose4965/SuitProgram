#include "paint_service.h"
#include "arm_pose.h"
#include "orientation_service.h"
#include "led.h"
#include "led_layout.h"
#include "show_geometry.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

// Explicit opt-in Canvas, front surface only. No world-position estimation.
enum { PAINT_VERSION=0x51 };
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
static paint_mode_t s_mode;
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
static paint_error_t s_error;

void paint_service_receive(link_msg_type_t type,const uint8_t *mac,const uint8_t *data,size_t len)
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
    if(!cfg) { s_error=PAINT_NO_MEMORY; return false; }
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
        s_error=PAINT_NO_LAYOUT; return false;
    }
    for(size_t i=0;i<s_count;i++) if(s_front[i]) {
        // Geometry is in cm: body centreline and belt/waist are fixed
        // anatomical anchors, independent of which LED sections exist.
        s_xy[i][0]=s_xy[i][0]/30.0f;
        s_xy[i][1]=(s_xy[i][1]-33.5f)/75.0f;
    }
    if(led_output_claim(LED_OUTPUT_OWNER_PAINT)!=ESP_OK) {s_error=PAINT_LEDS_BUSY;return false;}
    xQueueReset(s_queue); s_have_previous=false; s_last_rx=0; s_entered=esp_timer_get_time();
    portENTER_CRITICAL(&s_rx_mux); s_accept=true; portEXIT_CRITICAL(&s_rx_mux);
    return true;
}

void paint_service_open(void)
{
    if(!s_queue) s_queue=xQueueCreateStatic(8,sizeof(paint_rx_t),s_queue_bytes,&s_queue_storage);
    s_mode=PAINT_MENU; s_error=PAINT_OK; s_pose.calibrated=false;
    s_painting=false; s_on_canvas=false; s_color=0;
    s_session=esp_random(); if(!s_session) s_session=1;
    s_sequence=s_stroke=0; s_last_send=0;
}
void paint_service_leave(void)
{
    leave_canvas();
    s_mode=PAINT_MENU;
}
void paint_service_command(paint_command_t command)
{
    s_error=PAINT_OK;
    switch(command) {
    case PAINT_DISMISS_ERROR: break;
    case PAINT_LEAVE: paint_service_leave(); break;
    case PAINT_SELECT_PAINTER:
        if(s_mode!=PAINT_MENU) break;
        s_mode=PAINT_PAINTER;s_pose.calibrated=false;s_painting=false;
        break;
    case PAINT_SELECT_CANVAS:
        if(s_mode==PAINT_MENU && s_queue && prepare_canvas()) s_mode=PAINT_CANVAS;
        break;
    case PAINT_TOGGLE_ARM:
        if(s_mode!=PAINT_MENU) break;
        s_left_arm=!s_left_arm;
        s_pose.calibrated=false;s_painting=false;s_on_canvas=false;++s_stroke;
        break;
    case PAINT_CALIBRATE: {
        if(s_mode!=PAINT_PAINTER) break;
        imu_quat_t q; uint64_t t;
        s_painting=false;++s_stroke;
        if(orientation_service_snapshot(&q,&t) && esp_timer_get_time()-(int64_t)t<250000) {
            if(!arm_pose_calibrate_heading(&s_pose,q)) s_error=PAINT_NEED_HORIZONTAL;
        } else s_error=PAINT_IMU_UNAVAILABLE;
        break;
    }
    case PAINT_NEXT_COLOR:
        if(s_mode==PAINT_PAINTER) {s_color=(s_color+1)%5;++s_stroke;}
        break;
    case PAINT_TOGGLE_BRUSH:
        if(s_mode==PAINT_PAINTER && s_pose.calibrated) {s_painting=!s_painting;++s_stroke;}
        break;
    case PAINT_CLEAR:
        if(s_mode==PAINT_CANVAS) {memset(s_rgb,0,sizeof(s_rgb));s_have_previous=false;}
        break;
    }
}
paint_status_t paint_service_status(void)
{
    return (paint_status_t){.mode=s_mode,.left_arm=s_left_arm,
        .calibrated=s_pose.calibrated,.painting=s_painting,.on_canvas=s_on_canvas,
        .color=s_color,.connected=s_last_rx && esp_timer_get_time()-s_last_rx<2000000,
        .error=s_error};
}

void paint_service_tick(float dt)
{
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
            s_error=PAINT_LINK_UNAVAILABLE;
        else s_error=PAINT_OK;
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
