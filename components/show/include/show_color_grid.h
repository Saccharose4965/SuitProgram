#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "show_noise_color.h"

typedef struct __attribute__((packed)) {
    uint8_t mode, rows, columns, loop, time_blend, space_blend, tempo_sync, reserved;
    float rate;
} show_color_grid_header_t;

typedef struct { unsigned rows, blend; float color[8][4]; show_noise_color_t noise; float drift; } show_color_grid_t;

static inline float grid_weight(float t,unsigned blend)
{
    return blend==0 ? 0 : (blend==2 ? t*t*(3-2*t) : t);
}

static inline void show_grid_prepare(show_color_grid_t *out,const uint8_t *data,
                                    float elapsed,float duration,float beats)
{
    show_color_grid_header_t h; memcpy(&h,data,sizeof(h)); data+=sizeof(h);
    if (h.mode==5) { memcpy(&out->noise,data,sizeof(out->noise)); data+=sizeof(out->noise); out->drift=beats*out->noise.speed; }
    out->rows=h.rows; out->blend=h.space_blend;
    float t=h.loop ? (h.tempo_sync ? beats : elapsed)*h.rate : fminf(1,fmaxf(0,elapsed/fmaxf(.001f,duration)));
    float x=h.loop ? (t-floorf(t))*h.columns : t*(h.columns-1);
    unsigned left=(unsigned)x; if(left>=h.columns)left=h.columns-1;
    unsigned right=h.loop ? (left+1)%h.columns : (left+1<h.columns ? left+1 : left);
    float weight=grid_weight(x-left,h.time_blend);
    for(unsigned r=0;r<h.rows;++r)for(unsigned c=0;c<4;++c) {
        float a=data[(r*h.columns+left)*4+c],b=data[(r*h.columns+right)*4+c];
        out->color[r][c]=(a+(b-a)*weight)/255;
    }
}

static inline void show_grid_sample(const show_color_grid_t *grid,float u,float out[4])
{
    float x=fminf(1,fmaxf(0,u))*(grid->rows-1);
    unsigned a=(unsigned)x,b=a+1<grid->rows?a+1:a;
    float weight=grid_weight(x-a,grid->blend);
    for(unsigned c=0;c<4;++c)out[c]=grid->color[a][c]+(grid->color[b][c]-grid->color[a][c])*weight;
}
