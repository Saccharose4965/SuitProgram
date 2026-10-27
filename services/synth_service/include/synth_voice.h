#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define SYNTH_RATE 24000
typedef struct { float phase, hz, gain, rate, table[257]; } synth_voice_t;
static inline void synth_voice_init(synth_voice_t *v)
{
    *v = (synth_voice_t){.hz=220,.rate=SYNTH_RATE};
    for (int i=0;i<=256;i++) v->table[i]=sinf(i*6.28318530718f/256);
}
static inline void synth_voice_render(synth_voice_t *v, int16_t *out,
                                      size_t n, float hz, float gain)
{
    for (size_t i=0;i<n;i++) {
        v->hz += (hz-v->hz)*(.002f*SYNTH_RATE/v->rate);
        v->gain += (gain-v->gain)*(.002f*SYNTH_RATE/v->rate);
        float p=v->phase*256;
        unsigned k=(unsigned)p;
        float wave=v->table[k]+(v->table[k+1]-v->table[k])*(p-k);
        out[i]=(int16_t)(wave*v->gain*12000);
        v->phase+=v->hz/v->rate;
        if(v->phase>=1) v->phase-=1;
    }
}
