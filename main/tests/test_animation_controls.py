"""Compile the production animation input handler against simple UI stubs."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

class ControlsTest(unittest.TestCase):
    def test_grid(self):
        source = (ROOT / 'main/app_leds.c').read_text()
        handler = source[source.index('void leds_app_handle_input('):source.index('void leds_app_draw(')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
enum { INPUT_EVENT_PRESS, INPUT_EVENT_RELEASE };
enum { INPUT_BTN_A, INPUT_BTN_B, INPUT_BTN_C, INPUT_BTN_D };
enum { LEDS_ENTRY_KIND_BACK, LEDS_ENTRY_KIND_MODE, LEDS_ENTRY_KIND_CFG_SPEED,
       LEDS_ENTRY_KIND_CFG_PLANE_BG, LEDS_ENTRY_KIND_CFG_RING_BG };
typedef struct {int type,button;} input_event_t;
typedef struct {void (*request_switch)(const char *,void *);void *request_user_data;} shell_app_context_t;
typedef struct {int kind,backend,value;} leds_menu_meta_t;
static leds_menu_meta_t s_meta[7];
static struct {size_t selected;uint8_t custom_speed_percent;int current_backend,current_mode;
               bool plane_bg,ring_bg;} s_leds;
static bool s_speed_editing;
static int applied,speed;
static size_t count=7;
static size_t build_entries(void){return count;}
static size_t clamp_sel(size_t i,size_t n){return i<n?i:n-1;}
static int clamp_int(int v,int a,int b){return v<a?a:v>b?b:v;}
static void leds_apply_current_mode(void){applied++;}
static void led_modes_set_speed_percent(int v){speed=v;}
HANDLER
static void press(int button){shell_app_context_t ctx={0};input_event_t e={INPUT_EVENT_PRESS,button};leds_app_handle_input(&ctx,&e);}
int main(void){
    for(int i=0;i<7;i++)s_meta[i]=(leds_menu_meta_t){LEDS_ENTRY_KIND_MODE,0,i};
    press(INPUT_BTN_A);assert(s_leds.selected==0);
    press(INPUT_BTN_C);assert(s_leds.selected==1 && !applied);
    press(INPUT_BTN_C);assert(s_leds.selected==1);
    press(INPUT_BTN_B);assert(s_leds.selected==3);
    press(INPUT_BTN_D);assert(s_leds.current_mode==3 && applied==1);
    press(INPUT_BTN_B);assert(s_leds.selected==5);
    press(INPUT_BTN_B);assert(s_leds.selected==1);
    s_leds.selected=6;press(INPUT_BTN_C);assert(s_leds.selected==6);
    press(INPUT_BTN_B);assert(s_leds.selected==0);
    s_meta[0].kind=LEDS_ENTRY_KIND_CFG_SPEED;s_leds.custom_speed_percent=100;
    press(INPUT_BTN_D);assert(s_speed_editing);
    press(INPUT_BTN_A);assert(speed==90 && s_leds.selected==0);
    press(INPUT_BTN_C);assert(speed==100);
    for(int i=0;i<30;i++){press(INPUT_BTN_C);} assert(speed==250);
    for(int i=0;i<30;i++){press(INPUT_BTN_A);} assert(speed==10);
    press(INPUT_BTN_D);assert(!s_speed_editing && s_leds.selected==0);
    count=1;press(INPUT_BTN_B);assert(s_leds.selected==0);
}
'''.replace('HANDLER', handler)
        with tempfile.TemporaryDirectory(prefix='suit-controls-') as tmp:
            path = Path(tmp)
            (path/'test.c').write_text(code)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined',
                            str(path/'test.c'), '-o', str(path/'test')], check=True)
            subprocess.run([str(path/'test')], check=True)

if __name__ == '__main__':
    unittest.main()
