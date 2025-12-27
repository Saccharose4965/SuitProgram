// components/logo/logo.c
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======= Offscreen buffer (64x64 1bpp) =======
enum { W = 64, H = 64 };
static uint8_t gBuf[W * H / 8];

static inline void buf_clear(void){ memset(gBuf, 0, sizeof(gBuf)); }
static inline void pset(int x,int y){
    if ((unsigned)x >= W || (unsigned)y >= H) return;
    int idx = y * W + x;                            // <-- use W here
    gBuf[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

// ===== Geometry stampers =====
static void stamp_disc(float cx,float cy,float r){
    if (r<=0) return;
    int x0=(int)floorf(cx-r-1), x1=(int)ceilf(cx+r+1);
    int y0=(int)floorf(cy-r-1), y1=(int)ceilf(cy+r+1);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= W) x1 = W - 1;
    if (y1 >= H) y1 = H - 1;
    float r2=r*r;
    for (int y=y0;y<=y1;++y){
        float dy=(y+0.5f)-cy; float dy2=dy*dy;
        for (int x=x0;x<=x1;++x){
            float dx=(x+0.5f)-cx;
            if (dx*dx+dy2<=r2) pset(x,y);
        }
    }
}
static void stamp_line(float x0,float y0,float x1,float y1,float w){
    float dx=x1-x0, dy=y1-y0, L=hypotf(dx,dy);
    float r=w*0.5f;
    if (L<=0.0001f){ stamp_disc(x0,y0,r); return; }
    int steps=(int)(L*2)+1; // ~0.5 px step
    for (int i=0;i<=steps;++i){
        float t=(float)i/steps;
        stamp_disc(x0+dx*t, y0+dy*t, r);
    }
}
static void stamp_arc(float cx,float cy,float r,float a0,float a1,float w){
    // CCW from a0 to a1 (deg), wrap-safe
    float d=fmodf(a1-a0+360.0f,360.0f); if (d<=0.0f) return;
    float rr=w*0.5f;
    int steps=(int)fmaxf(8.0f, (2.0f*(float)M_PI*r*(d/360.0f))/0.6f);
    for (int i=0;i<=steps;++i){
        float a=(a0 + d*((float)i/steps))*(float)M_PI/180.0f;
        stamp_disc(cx + r*cosf(a), cy + r*sinf(a), rr);
    }
}
static inline void polar(float ang_deg, float R, float cx,float cy, float* x,float* y){
    float a=ang_deg*(float)M_PI/180.0f;
    *x=cx + R*cosf(a); *y=cy + R*sinf(a);
}

// ======= Easing =======
static inline float clamp01(float t){ return t<0?0:t>1?1:t; }
static inline float easeInOut(float t){ return t*t*(3.0f-2.0f*t); }
static inline float easeOut(float t){ float u=1.0f-t; return 1.0f-u*u; }
static float bounce(float t,float target,float overshoot){
    t=clamp01(t);
    if (t<=0.5f){ float u=easeOut(t/0.5f); return fmaxf(0.0f, target*(u*overshoot)); }
    float u=(t-0.5f)/0.5f; return target*(overshoot - (overshoot-1.0f)*easeInOut(u));
}

// ======= Animation timing/geometry (v13) =======
static const int NFRAMES=160;
static const int FPS=30;
// Windows
static const int A0=0,   A1=25;     // center bounce
static const int B0=20,  B1=51;     // arms
static const int C0=35,  C1=61;     // tip bounces
static const int D0=60,  D1=151;    // big arcs
static const int E0=140, E1=160;    // links

// Layout
static const float CX=32.0f, CY=32.0f;
static const float globalRot = 60.0f;

// Sizes
static const float armLen=25.0f, armW=2.0f;
static const float centerR=4.0f;      // Ø8
static const float tipR=4.5f;         // Ø9
static const float bigR=8.0f;         // big circle radius
static const float bigCdist=20.0f;    // center radius
static const float linkW=2.0f;

// Angles
static const float tri0[3] = { -90.0f, 30.0f, 150.0f };
static const float tri1[3] = { tri0[0]+globalRot, tri0[1]+globalRot, tri0[2]+globalRot };

// ======= Frame builder =======
static void draw_frame(int f){
    buf_clear();

    // A: center bounce
    if (f < A1){
        float t = (float)(f-A0)/(float)(A1-A0-1);
        float r = bounce(t, centerR, 1.35f);
        stamp_disc(CX, CY, r);
    } else {
        stamp_disc(CX, CY, centerR);
    }

    // B: arms
    if (f >= B0){
        float t = clamp01((float)(f-B0)/(float)(B1-B0));
        t = easeInOut(t);
        for (int i=0;i<3;++i){
            float x1,y1; polar(tri0[i], armLen*t, CX, CY, &x1,&y1);
            stamp_line(CX,CY, x1,y1, armW);
        }
    }

    // C: tip bounces
    if (f >= C0){
        float t = clamp01((float)(f-C0)/(float)(C1-C0-1));
        float r = bounce(t, tipR, 1.25f);
        for (int i=0;i<3;++i){
            float tx,ty; polar(tri0[i], armLen, CX, CY, &tx,&ty);
            stamp_disc(tx,ty,r);
        }
    }

    // D: big arcs (moving head/tail)
    if (f >= D0){
        float t = clamp01((float)(f-D0)/(float)(D1-D0));
        float u  = easeInOut(t);
        float base = 360.0f * 1.25f * u;   // rotation while drawing
        float span = 360.0f * u;           // span grows 0 -> 360
        float head = base + span*0.5f;
        float tail = base - span*0.5f;

        for (int i=0;i<3;++i){
            float cx,cy; polar(tri1[i], bigCdist, CX, CY, &cx,&cy);
            if (span >= 359.9f){
                stamp_arc(cx,cy,bigR, 0.0f, 359.999f, armW);
            } else if (span > 0.0f){
                stamp_arc(cx,cy,bigR, tail, head, armW);
            }
        }
    }

    // E: links — one-sided growth
    if (f >= E0){
        float t = clamp01((float)(f-E0)/(float)(E1-E0));
        t = easeInOut(t);

        float cx[3], cy[3];
        for (int i=0;i<3;++i) polar(tri1[i], bigCdist, CX, CY, &cx[i], &cy[i]);

        const int pairs[3][2] = {{0,1},{1,2},{2,0}};
        for (int k=0;k<3;++k){
            int i0 = pairs[k][0], i1 = pairs[k][1];
            float dx = cx[i1]-cx[i0], dy = cy[i1]-cy[i0];
            float L  = hypotf(dx,dy); if (L < 1e-3f) continue;
            float ux = dx/L, uy = dy/L;

            float sx = cx[i0] + ux*bigR;
            float sy = cy[i0] + uy*bigR;
            float ex = cx[i1] - ux*bigR;
            float ey = cy[i1] - uy*bigR;

            float gx = sx + (ex - sx)*t;
            float gy = sy + (ey - sy)*t;
            stamp_line(sx, sy, gx, gy, linkW);
        }
    }
}

// ======= Public: run once and return =======
void anim_logo(void){
    oled_init();
    oled_clear();

    const TickType_t frame_delay = pdMS_TO_TICKS(1000 / FPS);
    const TickType_t hold_delay  = pdMS_TO_TICKS(1000);
    const int slide_steps = (int)(3.5f * FPS);

    // Play centered
    for (int f = 0; f < NFRAMES; ++f){
        draw_frame(f);
        oled_blit64_center(gBuf);
        vTaskDelay(frame_delay);
    }

    // Hold centered
    vTaskDelay(hold_delay);

    // Slingshot slide (reuse last gBuf)
    for (int s = 0; s < slide_steps; ++s){
        int xoff = oled_calc_scroll_px_slick(s, slide_steps);
        oled_blit64_offset(gBuf, xoff);
        vTaskDelay(frame_delay);
    }

    // Safety: ensure fully off (SCROLL_TARGET == 104 in oled driver)
    oled_blit64_offset(gBuf, -104);
}

