#include "pong_physics.h"
#include <math.h>

void pong_advance(pong_ball_t *b, float seconds, float top, float bottom)
{
    b->x += b->vx * seconds;
    float span=bottom-top;
    float p=fmodf(b->y-top+b->vy*seconds,2*span);
    if (p<0) p+=2*span;
    if (p>=span) { b->y=bottom-(p-span); b->vy=-b->vy; }
    else b->y=top+p;
    if (b->y<=top) b->vy=fabsf(b->vy);
    if (b->y>=bottom) b->vy=-fabsf(b->vy);
}

bool pong_paddle_hit(const pong_ball_t *b,float y,float height,float radius)
{ return b->y>=y-radius && b->y<=y+height+radius; }

void pong_bounce(pong_ball_t *b,unsigned side,float y,float height)
{
    b->vx=side ? -fabsf(b->vx) : fabsf(b->vx);
    float offset=(b->y-(y+height*.5f))/(height*.5f);
    b->vy=fmaxf(-70,fminf(70,b->vy+offset*30));
}

float pong_correction_weight(float elapsed,float duration)
{
    if (duration<=0 || elapsed>=duration) return 0;
    float t=fmaxf(0,elapsed/duration);
    return 1-t*t*(3-2*t);
}
