"""Production debounce and audio policy with synthetic ladder samples."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]

class InputTest(unittest.TestCase):
    def test_transitions(self):
        paths=['drivers/input/include/input.h','drivers/input/input.c',
               'services/input_audio_policy/input_audio_policy.c']
        code='\n'.join(l for path in paths for l in (ROOT/path).read_text().splitlines()
                       if not l.startswith(('#include','#pragma once')))
        self.assertNotIn('fft', (ROOT/'drivers/input/input.c').read_text())
        prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1
#define ESP_OK 0
#define HW_BUTTONS_ADC_CH 0
#define ESP_LOGW(tag,...) ((void)(tag))
static int voltage,calls,backfill,future;
static bool fail;
static TickType_t clock_tick=1;
static TickType_t xTaskGetTickCount(void){return clock_tick;}
static int hw_adc1_init_default(void){return 0;}
static int hw_adc1_read_mv(int ch,int *mv){(void)ch;*mv=voltage;return fail?-1:0;}
static void fft_suppress_novelty_timed_ms(int b,int f){calls++;backfill=b;future=f;}
'''
        test=r'''
int main(void) {
    input_event_t ev;
    input_init();input_set_activity_callback(input_audio_policy_activity);
    voltage=825;assert(!input_poll(&ev,10));assert(!input_poll(&ev,34));
    assert(input_poll(&ev,35));assert(ev.button==INPUT_BTN_A && calls==1 && backfill==45 && future==2);
    assert(!input_poll(&ev,40));assert(calls==1);
    // Short ADC excursion must not create a notification.
    voltage=1650;assert(!input_poll(&ev,50));voltage=825;assert(!input_poll(&ev,60));
    assert(!input_poll(&ev,90));assert(calls==1);
    // Direct A -> B, with delayed polling: suppression includes entire latency.
    voltage=1650;assert(!input_poll(&ev,100));assert(input_poll(&ev,150));
    assert(ev.button==INPUT_BTN_B && calls==2 && backfill==70);
    assert(input_poll(&ev,1350));assert(ev.type==INPUT_EVENT_LONG_PRESS && calls==2);
    assert(!input_poll(&ev,1400));
    // Release reports activity even though it returns no UI event.
    voltage=0;assert(!input_poll(&ev,1410));assert(!input_poll(&ev,1440));
    assert(calls==3 && backfill==50);
    fail=true;assert(!input_poll(&ev,1500));assert(calls==3);fail=false;
    input_set_activity_callback(NULL);
    voltage=3300;assert(!input_poll(&ev,1600));assert(input_poll(&ev,1630));
    assert(ev.button==INPUT_BTN_D && calls==3);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-input-') as tmp:
            p=Path(tmp);(p/'test.c').write_text(prefix+code+test)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)
if __name__=='__main__':unittest.main()
