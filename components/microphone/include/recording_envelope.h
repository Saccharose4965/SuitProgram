#pragma once
#include <stdint.h>
#include <stddef.h>

#define MIC_REC_ENVELOPE_COLUMNS 96
typedef struct {
    uint8_t columns[MIC_REC_ENVELOPE_COLUMNS];
    unsigned count, next, frames, peak, window;
} recording_envelope_t;
static inline void recording_envelope_push(recording_envelope_t *e)
{
    unsigned value=e->peak/128;
    e->columns[e->next]=value>255?255:value;
    e->next=(e->next+1)%MIC_REC_ENVELOPE_COLUMNS;
    if(e->count<MIC_REC_ENVELOPE_COLUMNS)++e->count;
    e->frames=e->peak=0;
}
static inline void recording_envelope_feed(recording_envelope_t *e,const int16_t *pcm,size_t n)
{
    for(size_t i=0;i<n;i++) {
        unsigned value=pcm[i]<0?-(int)pcm[i]:pcm[i];
        if(value>e->peak)e->peak=value;
        if(++e->frames>=e->window)recording_envelope_push(e);
    }
}
static inline size_t recording_envelope_copy(const recording_envelope_t *e,uint8_t *out,size_t capacity)
{
    size_t n=e->count<capacity?e->count:capacity;
    for(size_t i=0;i<n;i++)out[i]=e->columns[(e->next+MIC_REC_ENVELOPE_COLUMNS-n+i)%MIC_REC_ENVELOPE_COLUMNS];
    return n;
}
