#pragma once
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "show_format.h"

/* Keys: intensity,width,softness,rate,phase,repeats,center_x,center_y,center_z,blink_level. */
#define SHOW_ANIMATION_KEYS 16u
static inline bool show_animation_valid(const show_animation_t *a)
{
    static const float low[] = {0,.01f,0,0,0,1,-10000,-10000,-10000,0,0,0,0,0,0,0};
    static const float high[] = {1,2,2,40,1,64,10000,10000,10000,1,1,1,1,1,1,1};
    return a->key < SHOW_ANIMATION_KEYS && a->mode <= 4 && a->tempo_sync <= 1 && a->shape <= 4 &&
        (a->mode != 4 || a->key <= 2 || a->key == 5 || a->key >= 10) &&
        (a->key != 9 || a->mode == 3) &&
        isfinite(a->duty) && a->duty >= 0 && a->duty <= 1 &&
        isfinite(a->from) && isfinite(a->to) && isfinite(a->rate) &&
        a->from >= low[a->key] && a->from <= high[a->key] &&
        a->to >= low[a->key] && a->to <= high[a->key] && a->rate >= 0 && a->rate <= 40;
}

static inline float show_amplitude_value(const show_animation_t *a, const uint8_t *samples,
                                         show_amplitude_curve_t curve, uint32_t elapsed_ms)
{
    unsigned index=elapsed_ms/curve.step_ms;
    if (index >= (unsigned)curve.count-1) index=curve.count-2;
    float fraction=fminf(1,(elapsed_ms-index*curve.step_ms)/(float)curve.step_ms);
    uint16_t left,right;
    memcpy(&left,samples+index*2,2);
    memcpy(&right,samples+(index+1)*2,2);
    float weight=(left+((float)right-left)*fraction)/65535.0f;
    return a->from+(a->to-a->from)*weight;
}

static inline float show_animation_value(const show_animation_t *a, float t, float duration,
                                         float bpm, float *integral)
{
    float u = t/fmaxf(.001f,duration), weight=0, area=0;
    if (a->mode == 1) { weight=u; area=t*u*.5f; }
    else if (a->mode == 2) { weight=u*u*(3-2*u); area=duration*(u*u*u-u*u*u*u*.5f); }
    else if (a->mode == 3) {
        float omega=6.28318530718f*a->rate*(a->tempo_sync ? bpm/60 : 1);
        weight=(1-cosf(omega*t))*.5f;
        area=omega > 1e-9f ? t*.5f-sinf(omega*t)/(2*omega) : 0;
        float rate=omega/6.28318530718f, cycles=rate*t, whole=floorf(cycles), f=cycles-whole;
        if (a->shape == 1) {
            float x=a->duty>0 ? fminf(1,f/a->duty) : 1;
            weight=1-fabsf(2*x-1);
            float partial=x<=.5f ? x*x : 2*x-x*x-.5f;
            area=rate>0 ? a->duty*(whole*.5f+partial)/rate : 0;
        } else if (a->shape == 2) {
            weight=f<a->duty ? 1 : 0;
            area=rate>0 ? (whole*a->duty+fminf(f,a->duty))/rate : t*weight;
        } else if (a->shape == 3 || a->shape == 4) {
            float start=a->shape==4 ? a->duty : 0;
            float length=a->shape==4 ? 1-a->duty : a->duty;
            float x=length>0 ? fmaxf(0,fminf(1,(f-start)/length)) : (start>0 ? 0 : 1);
            const float floor=0.006737947f;
            weight=(expf(-5*x)-floor)/(1-floor);
            float partial=((1-expf(-5*x))/5-x*floor)/(1-floor);
            float total=.2f-floor/(1-floor);
            area=rate>0 ? (whole*(start+length*total)+fminf(f,start)+length*partial)/rate : t*weight;
        } else {
            float x=a->duty>0 ? fminf(1,f/a->duty) : 1;
            weight=(1-cosf(6.28318530718f*x))*.5f;
            area=rate>0 ? a->duty*(whole*.5f+x*.5f-sinf(6.28318530718f*x)/12.56637061436f)/rate : 0;
        }
    }
    if (integral) *integral=a->from*t+(a->to-a->from)*area;
    return a->from+(a->to-a->from)*weight;
}
