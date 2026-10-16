#pragma once
#include <math.h>
#include <stdint.h>

typedef struct __attribute__((packed)) { float scale, width, speed; } show_noise_color_t;

static inline float show_noise_gradient(int32_t ix, int32_t iy, int32_t iz, float x, float y, float z)
{
    uint32_t h=(uint32_t)ix*UINT32_C(0x9e3779b1) ^ (uint32_t)iy*UINT32_C(0x85ebca77) ^ (uint32_t)iz*UINT32_C(0xc2b2ae3d) ^ UINT32_C(0xa341316c);
    h^=h>>16; h*=UINT32_C(0x7feb352d); h^=h>>15;
    h &= 15;
    float u=h<8 ? x : y, v=h<4 ? y : (h==12 || h==14 ? x : z);
    return (h&1 ? -u : u)+(h&2 ? -v : v);
}

static inline float show_noise_fade(float t) { return t*t*t*(t*(t*6-15)+10); }

static inline float show_noise_contour(float x, float y, float time, float width)
{
    int32_t ix=(int32_t)floorf(x), iy=(int32_t)floorf(y), iz=(int32_t)floorf(time);
    x-=ix; y-=iy; float z=time-iz;
    float u=show_noise_fade(x), v=show_noise_fade(y), w=show_noise_fade(z), value=0;
    for (int dz=0;dz<2;++dz) for (int dy=0;dy<2;++dy) for (int dx=0;dx<2;++dx)
        value+=show_noise_gradient(ix+dx,iy+dy,iz+dz,x-dx,y-dy,z-dz)*(dx?u:1-u)*(dy?v:1-v)*(dz?w:1-w);
    return fminf(1,fabsf(value)/width);
}
