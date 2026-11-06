"""Controller lifecycle is passive; explicit actions own timing changes."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class ControllerTest(unittest.TestCase):
    def test_entry_draw_and_actions(self):
        source = (ROOT / 'apps/led_control/fft_sync_app.c').read_text()
        source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
        harness = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fft.h"
#include "led_source.h"
enum {INPUT_EVENT_PRESS, INPUT_EVENT_LONG_PRESS};
enum {INPUT_BTN_A, INPUT_BTN_B, INPUT_BTN_C, INPUT_BTN_D};
enum {SHELL_ICON_NONE,SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_LEFT,
      SHELL_ICON_RIGHT,SHELL_ICON_SELECT};
typedef struct {int type,button;} input_event_t;
typedef struct {void (*request_switch)(const char *,void *);void *request_user_data;} shell_app_context_t;
typedef struct {int slots[4];} shell_legend_t;
typedef struct {float scroll;} shell_menu_motion_t;
typedef struct {const char *id,*label;} shell_menu_entry_t;
typedef struct {const shell_menu_entry_t *entries;size_t count,selected;
                const char *title;const shell_menu_motion_t *motion;} shell_menu_view_t;
static fft_sync_state_t state;
static int audio_initializations,source_ticks,resets,forces,sets,trims;
bool audio_bus_init_if_needed(void){audio_initializations++;return true;}
void led_source_service_tick(float dt){(void)dt;source_ticks++;}
#define ESP_LOGE(tag,...) ((void)(tag))
void fft_get_sync_state(fft_sync_state_t *out){*out=state;}
led_source_mode_t led_source_mode_get(void){return LED_SOURCE_FFT;}
bool led_source_is_following(void){return false;}
led_timing_mode_t led_source_timing_get(void){return LED_TIMING_ON_BEAT;}
const char *led_source_timing_name(led_timing_mode_t mode){(void)mode;return "auto";}
float led_source_phase_offset_get(void){return state.phase_offset;}
void led_source_phase_offset_adjust(float delta){trims++;state.phase_offset+=delta;}
void led_source_timing_press(void){sets++;}
void fft_detector_reset(void){resets++;}
void fft_beat_lock_set(bool locked){forces++;state.bpm_locked=locked;}
void shell_ui_menu_reset(shell_menu_motion_t *m,size_t selected){m->scroll=selected;}
void shell_ui_menu_tick(shell_menu_motion_t *m,float dt,size_t selected){(void)m;(void)dt;(void)selected;}
void shell_ui_draw_menu(uint8_t *fb,int x,int y,int w,int h,const shell_menu_view_t *v){
 (void)fb;(void)x;(void)y;(void)w;(void)h;assert(v->count==5&&v->selected<5);
}
@@CONTROLLER@@
static void input(shell_app_context_t *ctx,int type,int button){
 input_event_t ev={type,button};fft_sync_app_handle_input(ctx,&ev);
}
int main(void){
 shell_app_context_t ctx={0};uint8_t fb[1024]={0};
 state=(fft_sync_state_t){.held_bpm=150,.bpm=150,.clock_base_bpm=150,
     .clock_origin=.23f,.beat_phase=.7f,.ratio=1,.running=true,.beat_enabled=true};
 fft_sync_state_t before=state;
 for(int n=0;n<100;n++){
  fft_sync_app_init(&ctx);
  fft_sync_app_tick(&ctx,.016f);
  fft_sync_app_draw(&ctx,fb,0,0,128,64);
  // Holding the D used to enter must not invoke Reset via its long event.
  input(&ctx,INPUT_EVENT_LONG_PRESS,INPUT_BTN_D);
  fft_sync_app_deinit(&ctx);
  assert(!memcmp(&state,&before,sizeof(state)));
 }
 assert(!audio_initializations&&!source_ticks&&!resets&&!forces&&!sets&&!trims);
 fft_sync_app_init(&ctx);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_D);assert(resets==1);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_B);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_D);assert(forces==1);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_B);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_D);assert(sets==1);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_B);
 input(&ctx,INPUT_EVENT_PRESS,INPUT_BTN_D);assert(trims==1);
 assert(fabsf(state.phase_offset-.01f)<1e-6f);
 input(&ctx,INPUT_EVENT_LONG_PRESS,INPUT_BTN_C);assert(trims==2);
 assert(fabsf(state.phase_offset+.04f)<1e-6f);
 assert(!audio_initializations&&!source_ticks);
}
'''.replace('@@CONTROLLER@@', source)
        with tempfile.TemporaryDirectory(prefix='suit-controller-') as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(harness)
            subprocess.run([
                'gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=address,undefined', '-I'+str(ROOT/'services/fft/include'),
                '-I'+str(ROOT/'services/fft/tests/host_stubs'),
                '-I'+str(ROOT/'services/led_source/include'),
                str(path/'test.c'), '-lm', '-o', str(path/'test')], check=True)
            subprocess.run([str(path/'test')], check=True)


if __name__ == '__main__':
    unittest.main()
