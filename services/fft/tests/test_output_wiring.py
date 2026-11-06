"""Exercise production phase acquisition, offbeat controls, publication and dispatch."""
from pathlib import Path
import subprocess
import re
import tempfile
root=Path(__file__).resolve().parents[3];s=(root/'services/fft/fft.c').read_text()
def fn(name):
 a=re.search(r'^(?:static )?\w+(?: \*)? '+re.escape(name)+r'\(',s,re.M).start();b=s.index('{',a)+1;level=1
 while level:
  level+=(s[b]=='{')-(s[b]=='}');b+=1
 return s[a:b]+'\n'
h=r'''
#include "fft_phase.h"
#include "fft_display.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct {detector_control_t detector;uint32_t epoch,revision;bool beat_enabled,export_leds,offbeat;unsigned ratio;float offset;} controls_t;
typedef struct {fft_phase_evidence_t evidence;double time;float bpm,milliseconds;uint32_t epoch,reset_revision;bool valid;} phase_result_t;
static struct {detector_result_t detector;float milliseconds;} s_current;
static controls_t s_controls;
static fft_sync_state_t s_status;
static fft_phase_tracker_t s_phase_tracker;
static fft_phase_decision_t s_phase_decision;
static fft_beat_clock_t s_clock;
static unsigned s_clock_ratio=1;
static bool s_clock_offbeat,s_phase_busy;
static float s_clock_offset;
static double s_last_emit,now;
static uint32_t s_skipped,s_capture_errors;
static int s_mux,s_beats,s_phase_results; static void *s_capture_task=(void *)1;
static phase_result_t queued;
static bool ready;
static unsigned triggers;
static int receive(void *p){if(!ready)return 0;memcpy(p,&queued,sizeof(queued));ready=false;return 1;}
static int64_t esp_timer_get_time(void){return llround(now*1e6);}
static fft_sync_state_t display_output;
void fft_history_update_output(const fft_sync_state_t *out){display_output=*out;}
static void led_trigger_beat(uint8_t r,uint8_t g,uint8_t b){
 (void)r;(void)g;(void)b;++triggers;
 assert(display_output.clock_origin==s_status.clock_origin);
 assert(display_output.offbeat==s_clock_offbeat&&display_output.ratio==s_clock_ratio);
 assert(display_output.beat_phase<.051f);
}
void fft_history_update_detector(const detector_result_t *d){(void)d;}
#define pdTRUE 1
#define xQueueReceive(q,p,t) ((void)(q),(void)(t),receive(p))
#define xQueueSend(q,p,t) ((void)(q),(void)(p),(void)(t))
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
FUNCTIONS
int main(void){
 unsigned cases=0,emits=0;
 for(int bin=0;bin<128;bin++)for(int ratio=1;ratio<=2;ratio++)for(int manual=0;manual<2;manual++){
  memset(&s_clock,0,sizeof(s_clock));memset(&s_phase_tracker,0,sizeof(s_phase_tracker));
  memset(&s_phase_decision,0,sizeof(s_phase_decision));s_last_emit=0;
  s_clock_ratio=1;s_clock_offbeat=false;s_clock_offset=0;
  s_controls=(controls_t){.ratio=ratio,.offbeat=manual,.beat_enabled=true,.export_leds=true};
  s_current.detector=(detector_result_t){.held_bpm=150,.candidate_bpm=150,.confidence=1,.trusted=true,.history_coverage=5};
  for(int step=0;step<900;step++){
   now=1000+step*.01;
   if(step==300||step==600){
    fft_beat_offbeat_set(!s_controls.offbeat);
    fft_output_controls_t requested;fft_get_output_controls(&requested);
    fft_sync_state_t applied;fft_get_sync_state(&applied);
    assert(requested.offbeat==s_controls.offbeat);
    assert(applied.offbeat==s_clock_offbeat); // requested must not overwrite clock state
   }
   if(step%19==0){
    int family=bin%64;
    queued=(phase_result_t){.valid=true,.bpm=150,.time=now};
    queued.evidence.best_family=family;queued.evidence.best_side=bin/64;
    queued.evidence.raw_best_bin=bin;queued.evidence.prominence=2;
    // The stronger side changes later, but the acquired full-cycle branch must stay.
    queued.evidence.score[bin]=step<450?2:1;
    queued.evidence.score[(bin+64)%128]=step<450?1:2;
    queued.evidence.folded[family]=3;
    ready=true;poll_phase(controls_get());
   }
   unsigned before=triggers;emit_and_publish(now,controls_get());
   fft_sync_state_t status;fft_get_sync_state(&status);
   if(triggers!=before){
    assert(status.beat_phase<.051f);
    double origin=s_clock.beat_start*150/60.;origin-=floor(origin);
    double relative=now*150/60.-origin;relative-=round(relative);
    assert(fabs(relative)<.026);++emits;
   }
   double selected=s_phase_decision.anchor*150/60.-bin/128.;selected-=round(selected);
   assert(fabs(selected)<1e-8);
   double shift=(phase_clock_anchor(controls_get())-s_phase_decision.anchor)/.4;
   assert(fabs(shift-((s_controls.offbeat&&ratio==1)?.5:0))<1e-8);
  }
  ++cases;
 }
 printf("%u full acquisition/control cases, %u simulated LED dispatches through production code: both initial sides, stronger-side inversions, offbeat toggles and ratios; no extra half-cycle applied.\n",cases,emits);
}
'''
names=('controls_get','phase_clock_anchor','poll_phase','emit_and_publish','fft_get_sync_state','fft_get_output_controls','fft_beat_offbeat_set')
h=h.replace('FUNCTIONS',''.join(fn(n) for n in names))
with tempfile.TemporaryDirectory(prefix='suit-half-phase-') as tmp:
 p=Path(tmp)/'test.c';p.write_text(h)
 binary=str(Path(tmp)/'test')
 subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-Iservices/fft/include','-Iapps/fft_view/include','-Iservices/fft/tests/host_stubs',str(p),'services/fft/fft_phase.c','-lm','-o',binary],cwd=root,check=True)
 subprocess.run([binary],check=True)
