"""Production brush maths/app with fake clock, IMU, link and LED sink."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class PaintTest(unittest.TestCase):
    def test_service_boundary(self):
        service=(ROOT/'services/paint/paint_service.c').read_text()
        app=(ROOT/'apps/paint_app/app_light_paint.c').read_text()
        for forbidden in ('app_shell.h', 'oled.h', 'audio_policy.h',
                          'input_event_t', 'INPUT_BTN_', 'shell_app_context_t'):
            self.assertNotIn(forbidden, service)
        for forbidden in ('orientation_service_snapshot', 'led_output_claim',
                          'link_send_frame_to', 'xQueueReceive', 'show_geometry_pose'):
            self.assertNotIn(forbidden, app)

    def test_canvas_and_pose(self):
        paths=['services/paint/include/paint_service.h',
               'services/paint/paint_service.c','apps/paint_app/app_light_paint.c']
        source='\n'.join(l for path in paths for l in (ROOT/path).read_text().splitlines()
                         if not l.startswith(('#include','#pragma once')))
        renderer=(ROOT/'apps/threedee/threedee.c').read_text()
        reference=renderer[renderer.index('static inline quat q_ident('):renderer.index('static inline vec3 mat3_mul_vec(')]
        prefix=r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "arm_pose.h"
#include "led_layout.h"
#define EXT_RAM_BSS_ATTR
#define MALLOC_CAP_SPIRAM 0
#define MALLOC_CAP_8BIT 0
#define heap_caps_malloc(n,c) malloc(n)
#define heap_caps_free(p) free(p)
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define pdTRUE 1
#define SHELL_ICON_LEFT 0
#define SHELL_ICON_REFRESH 0
#define SHELL_ICON_RIGHT 0
#define SHELL_ICON_SELECT 0
#define LINK_MSG_CONTROL 4
#define LED_OUTPUT_OWNER_PAINT 2
typedef int link_msg_type_t;
typedef struct {void (*request_switch)(const char*,void*);void *request_user_data;} shell_app_context_t;
typedef struct {int slots[4];} shell_legend_t;
enum {INPUT_BTN_A=1,INPUT_BTN_B,INPUT_BTN_C,INPUT_BTN_D,INPUT_EVENT_PRESS=0};
typedef struct {int button,type;} input_event_t;
typedef struct {unsigned size,read,write,count;uint8_t *data;} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
static QueueHandle_t xQueueCreateStatic(int n,int size,uint8_t *data,StaticQueue_t *q)
{assert(n==8);q->size=size;q->data=data;return q;}
static void xQueueReset(QueueHandle_t q){q->read=q->write=q->count=0;}
static int xQueueSend(QueueHandle_t q,const void *p,int t)
{(void)t;if(q->count==8)return 0;memcpy(q->data+q->write*q->size,p,q->size);q->write=(q->write+1)%8;q->count++;return 1;}
static int xQueueReceive(QueueHandle_t q,void *p,int t)
{(void)t;if(!q->count)return 0;memcpy(p,q->data+q->read*q->size,q->size);q->read=(q->read+1)%8;q->count--;return 1;}
static int64_t now=1000000; static bool busy,claimed; static int sends;
static bool fft_suspended;
static void audio_policy_fft_set_paint_app(bool active){fft_suspended=active;}
static imu_quat_t sensor={1,0,0,0}; static uint64_t sample=1000000;
static int64_t esp_timer_get_time(void){return now;}
static uint32_t esp_random(void){return 123;}
static bool orientation_service_snapshot(imu_quat_t *q,uint64_t *t){*q=sensor;*t=sample;return sample!=0;}
static int led_output_claim(int o){(void)o;if(busy)return -1;claimed=true;return ESP_OK;}
static void led_output_release(int o){(void)o;claimed=false;}
static int led_show_pixels_owned(int o,const uint8_t *p,size_t n){(void)o;(void)p;assert(claimed && n==3);return 0;}
static void led_set_pixel_rgb(uint8_t *p,size_t i,uint8_t r,uint8_t g,uint8_t b){p[i*3]=r;p[i*3+1]=g;p[i*3+2]=b;}
static int link_send_frame_to(const uint8_t *m,int ty,const uint8_t *p,size_t n,bool ack)
{(void)m;(void)ty;(void)p;(void)n;assert(!ack);sends++;return 0;}
static void oled_draw_text3x5(uint8_t *p,int x,int y,const char *s){(void)p;(void)x;(void)y;(void)s;}
typedef int show_section_id_t;
static bool show_section_id_from_name(const char *s,int *id){(void)s;*id=1;return true;}
static led_point_t show_geometry_pose(int id,led_point_t p,float u){(void)id;(void)u;return p;}
void led_layout_snapshot(led_layout_config_t *cfg){memset(cfg,0,sizeof(*cfg));cfg->total_leds=3;cfg->section_count=1;strcpy(cfg->sections[0].name,"front_left_top");cfg->sections[0].length=3;}
bool led_layout_get_from_config(const led_layout_config_t *cfg,int i,led_point_t *p){(void)cfg;*p=(led_point_t){(i-1)*30,33.5f+(i-1)*75,0};return true;}
'''
        test=r'''
static void press(int key){input_event_t e={.button=key};light_paint_input(NULL,&e);}
int main(void) {
    // Heading-only calibration is invariant to local finger-axis roll.
    for(int i=0;i<100;i++) {
        float a=i*.061f;
        quat yaw={cosf(.35f),0,0,sinf(.35f)};
        quat roll={cosf(a),0,sinf(a),0};
        quat tilted=q_mul(yaw,q_mul((quat){cosf(.1f),sinf(.1f),0,0},roll));
        arm_pose_t p={0};
        assert(arm_pose_calibrate_heading(&p,tilted));
        assert(fabsf(p.zero.w-yaw.w)<1e-5f && fabsf(p.zero.z-yaw.z)<1e-5f);
        assert(p.zero.x==0 && p.zero.y==0);
        arm_pose_t other={0};
        assert(arm_pose_calibrate(&other,tilted));
        assert(fabsf(other.zero.w-p.zero.w)<1e-5f && fabsf(other.zero.z-p.zero.z)<1e-5f);
        assert(other.zero.x==0 && other.zero.y==0);
        imu_quat_t saved=other.zero;
        assert(!arm_pose_calibrate(&other,(quat){NAN,0,0,0}));
        assert(other.calibrated && other.zero.w==saved.w && other.zero.z==saved.z);
    }
    arm_pose_t invalid={0};
    assert(!arm_pose_calibrate_heading(&invalid,(quat){.70710678f,.70710678f,0,0}));
    float edge_x,edge_y;
    assert(arm_pose_canvas((arm_vec3_t){.2f,-.5f,1},&edge_x,&edge_y));
    assert(fabsf(edge_x-1)<1e-5f && fabsf(edge_y-1)<1e-5f);
    assert(!arm_pose_canvas((arm_vec3_t){.21f,0,1},&edge_x,&edge_y));
    // Use the actual unchanged 3D-renderer maths, not a second guessed mapping.
    // Its camera matrix is inverse(device); here we compare the device rotation.
    for(int i=0;i<1000;i++) {
        quat z=q_normed((quat){.3f+sinf(i*.17f),.4f,cosf(i*.11f),.6f});
        quat q=q_normed((quat){.7f,cosf(i*.13f),.2f+sinf(i*.07f),.1f});
        arm_pose_t p={0};arm_vec3_t actual;
        // Arbitrary stored reference still tests vector conversion independently
        // of calibration policy (whose heading/roll checks are above).
        p.zero=z;p.calibrated=true;
        quat relative=q_mul(q_conj(z),q);
        quat mounted=apply_mount(relative,(quat){.70710678f,0,0,.70710678f});
        float r[9];q_to_mat3(mounted,r);
        for(int left=0;left<2;left++) {
            assert(arm_pose_direction(&p,(imu_quat_t){q.w,q.x,q.y,q.z},left,&actual));
            // Raw +Y is mounted -X. Undo mount on the rotated beam, then
            // map reference +Y/right, +Z/up, -X/forward into the body frame.
            assert(fabsf(actual.x-(left?r[0]:-r[0]))<2e-5f);
            assert(fabsf(actual.y+r[6])<2e-5f);
            assert(fabsf(actual.z-r[3])<2e-5f);
        }
    }
    arm_pose_t pose={0}; arm_vec3_t d; float x,y;
    assert(!arm_pose_direction(&pose,sensor,false,&d));
    assert(!arm_pose_calibrate(&pose,(imu_quat_t){NAN,0,0,0}));
    assert(arm_pose_calibrate(&pose,sensor));
    assert(arm_pose_direction(&pose,sensor,false,&d));
    assert(fabsf(d.x+1)<1e-5 && !arm_pose_canvas(d,&x,&y));
    imu_quat_t unfolded={.70710678f,0,0,.70710678f};
    assert(arm_pose_direction(&pose,unfolded,false,&d));
    assert(arm_pose_canvas(d,&x,&y) && fabsf(x)<1e-5 && fabsf(y)<1e-5);
    imu_quat_t unfolded_left=unfolded;
    assert(arm_pose_direction(&pose,sensor,true,&d));
    assert(fabsf(d.x-1)<1e-5 && !arm_pose_canvas(d,&x,&y));
    assert(arm_pose_direction(&pose,unfolded_left,true,&d));
    assert(arm_pose_canvas(d,&x,&y) && fabsf(x)<1e-5 && fabsf(y)<1e-5);
    // Local forearm-axis roll must not steer the brush away from folded.
    imu_quat_t roll={.70710678f,0,.70710678f,0};
    assert(arm_pose_direction(&pose,roll,true,&d));
    assert(fabsf(d.x-1)<1e-5 && fabsf(d.y)<1e-5 && fabsf(d.z)<1e-5);
    assert(arm_pose_direction(&pose,roll,false,&d));
    assert(fabsf(d.x+1)<1e-5 && fabsf(d.y)<1e-5 && fabsf(d.z)<1e-5);
    imu_quat_t lift={.70710678f,.70710678f,0,0};
    assert(arm_pose_direction(&pose,lift,true,&d));
    assert(fabsf(d.y-1)<1e-5 && fabsf(d.x)<1e-5 && fabsf(d.z)<1e-5);
    assert(arm_pose_vector(&pose,roll,true,(arm_vec3_t){1,0,0},&d));
    assert(fabsf(d.y+1)<1e-5); // wrist roll remains visible in hand span
    assert(!arm_pose_canvas((arm_vec3_t){0,0,-1},&x,&y));
    assert(!arm_pose_canvas((arm_vec3_t){NAN,0,1},&x,&y));
    assert(arm_brush_distance(0,0,-1,0,1,0)==0);
    light_paint_init(NULL); assert(fft_suspended); press(INPUT_BTN_B);busy=true;press(INPUT_BTN_D);
    assert(s_mode==PAINT_MENU && !s_accept && !claimed);
    busy=false;press(INPUT_BTN_D);assert(s_mode==PAINT_CANVAS && claimed);
    paint_status_t status=paint_service_status();
    assert(status.mode==PAINT_CANVAS && !status.painting && status.error==PAINT_OK);
    paint_service_command(PAINT_TOGGLE_ARM); // invalid in Canvas, no side effects
    assert(paint_service_status().left_arm==status.left_arm && claimed);
    assert(s_xy[1][0]==0 && s_xy[1][1]==0); // waist, not LED bounding-box centre
    uint8_t mac[6]={1};paint_packet_t p={.version=PAINT_VERSION,.active=1,.session=8,.sequence=1,.stroke=1};
    paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p)-1);assert(!s_queue->count);
    paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p));light_paint_tick(NULL,.05f);
    assert(s_rgb[1][2]==255 && s_rgb[0][2]==0);
    now+=50000;light_paint_tick(NULL,.5f);float faded=s_rgb[1][2];assert(faded>0 && faded<255);
    paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p));light_paint_tick(NULL,0);
    assert(s_rgb[1][2]==faded); // duplicate cannot repaint
    p.sequence++;p.x=20000;paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p));assert(!s_queue->count);
    p.x=0;p.sequence++;paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p));now+=300000;light_paint_tick(NULL,0);
    assert(s_rgb[1][2]==faded); // stale queue entry cannot repaint
    uint8_t other[6]={2};p.sequence++;paint_service_receive(LINK_MSG_CONTROL,other,(uint8_t*)&p,sizeof(p));light_paint_tick(NULL,0);
    assert(s_rgb[1][2]==faded); // one painter at a time
    press(INPUT_BTN_B);assert(s_rgb[1][2]==0);
    light_paint_deinit(NULL);assert(!claimed && !s_accept && !fft_suspended);
    paint_service_receive(LINK_MSG_CONTROL,mac,(uint8_t*)&p,sizeof(p));assert(!s_queue->count);
    light_paint_init(NULL);press(INPUT_BTN_D);sample=now;press(INPUT_BTN_B);
    assert(s_pose.calibrated);press(INPUT_BTN_D);light_paint_tick(NULL,.05f);
    assert(s_painting && !s_on_canvas);
    sensor=unfolded_left;now+=50000;sample=now;light_paint_tick(NULL,.05f);
    assert(s_on_canvas && sends==2);
    status=paint_service_status();
    assert(status.mode==PAINT_PAINTER && status.calibrated && status.painting && status.on_canvas);
    now+=300000;light_paint_tick(NULL,.3f);assert(!s_on_canvas);
    press(INPUT_BTN_A);press(INPUT_BTN_B);press(INPUT_BTN_B);assert(s_choice==2);
    press(INPUT_BTN_D);assert(!s_left_arm && !s_pose.calibrated && !s_painting);
    press(INPUT_BTN_A);press(INPUT_BTN_A);press(INPUT_BTN_D);assert(s_mode==PAINT_PAINTER);
    sensor=(imu_quat_t){1,0,0,0};sample=now;press(INPUT_BTN_B);press(INPUT_BTN_D);
    sensor=unfolded;now+=50000;sample=now;light_paint_tick(NULL,.05f);
    assert(s_on_canvas && s_painting);
    light_paint_deinit(NULL);light_paint_init(NULL);assert(!s_left_arm);
    press(INPUT_BTN_B);press(INPUT_BTN_B);press(INPUT_BTN_D);assert(s_left_arm && !s_pose.calibrated);
    press(INPUT_BTN_C);assert(s_choice==2); // C is not list navigation
    press(INPUT_BTN_B);press(INPUT_BTN_B);assert(s_choice==3); // bounded at Back
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-paint-') as tmp:
            p=Path(tmp)
            (p/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n')
            (p/'test.c').write_text(prefix+'\ntypedef imu_quat_t quat;\n'+reference+source+
                                   test)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',tmp,'-I',str(ROOT/'services/orientation/include'),'-I',str(ROOT/'services/led_layout/include'),
                str(p/'test.c'),str(ROOT/'services/orientation/arm_pose.c'),'-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
