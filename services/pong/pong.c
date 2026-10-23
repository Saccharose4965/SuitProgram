#include "pong.h"
#include "pong_physics.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PONG_COURT_W 128.0f
#define PONG_COURT_H 64.0f
#define TOP 10.0f
#define BOTTOM (PONG_COURT_H-9.0f)
#define PADDLE_H 14.0f
#define LEFT_X 8.0f
#define RIGHT_X (PONG_COURT_W-8.0f)
#define CENTER_X (PONG_COURT_W*.5f)
#define CENTER_Y ((TOP+BOTTOM)*.5f)
#define HEARTBEAT_US 100000
#define TIMEOUT_US 3000000
#define SERVE_US 400000
#define PONG_VERSION 2

enum { MENU, WAITING, OFFERING, CONNECTING, HUMAN, AI };
enum { HELLO=1, OFFER, ACCEPT, STATE, LEAVE };
enum { EVENT_VALID=1, EVENT_SERVE=2 };

typedef struct __attribute__((packed)) {
    uint32_t sequence, flags, age_us;
    pong_ball_t ball;
    float paddle;
    uint16_t score[2];
} pong_event_t;

typedef struct __attribute__((packed)) {
    uint8_t game, version, kind, side;
    uint32_t nonce, target_nonce, session, waiting_ms;
    int64_t sent_us, echo_us;
    uint32_t echo_delay_us;
    float paddle;
    pong_event_t event;
} pong_packet_t;

typedef struct { uint8_t mac[6]; int64_t received_us; pong_packet_t packet; } pong_rx_t;
typedef struct {
    unsigned mode, selection, side;
    bool paused, stalled;
    uint8_t mac[6], peer[6];
    uint32_t nonce, peer_nonce, session, sequence;
    int64_t waiting_since, last_rx, last_tx, peer_sent_us, peer_received_us;
    int64_t last_tick, serve_until, event_time, correction_start;
    uint32_t rtt_us;
    float paddle[2], correction_x, correction_y, correction_seconds;
    pong_ball_t ball;
    uint16_t score[2];
    pong_event_t local_event;
} pong_state_t;

static pong_state_t s_pong;
static QueueHandle_t s_rx;
static volatile bool s_active;
static const uint8_t broadcast[6]={255,255,255,255,255,255};

static float clamp(float x,float lo,float hi) { return fmaxf(lo,fminf(hi,x)); }
static bool same_mac(const uint8_t *a,const uint8_t *b) { return memcmp(a,b,6)==0; }
static float plane(unsigned side) { return side ? RIGHT_X : LEFT_X; }

static void visual_position(int64_t now,float *x,float *y)
{
    float w=pong_correction_weight((now-s_pong.correction_start)*1e-6f,s_pong.correction_seconds);
    *x=s_pong.ball.x+s_pong.correction_x*w;
    *y=clamp(s_pong.ball.y+s_pong.correction_y*w,TOP,BOTTOM);
}

static void send_packet(unsigned kind)
{
    int64_t now=esp_timer_get_time();
    pong_packet_t p={.game=1,.version=PONG_VERSION,.kind=kind,.side=s_pong.side,
        .nonce=s_pong.nonce,.target_nonce=s_pong.peer_nonce,.session=s_pong.session,
        .waiting_ms=(uint32_t)((now-s_pong.waiting_since)/1000),.sent_us=now,
        .echo_us=s_pong.peer_sent_us,.echo_delay_us=s_pong.peer_received_us ?
            (uint32_t)(now-s_pong.peer_received_us) : 0,
        .paddle=s_pong.paddle[s_pong.side],
        .event=s_pong.local_event};
    if (p.event.flags & EVENT_VALID) p.event.age_us=(uint32_t)(now-s_pong.event_time);
    // Repeat collision snapshots; never wait for radio ACKs in the game loop.
    (void)link_send_frame_to(kind==HELLO ? broadcast : s_pong.peer,
        LINK_MSG_GAME,(const uint8_t *)&p,sizeof(p),false);
    s_pong.last_tx=now;
}

static void enter_waiting(int64_t now)
{
    uint8_t mac[6]; memcpy(mac,s_pong.mac,6);
    memset(&s_pong,0,sizeof(s_pong)); memcpy(s_pong.mac,mac,6);
    s_pong.mode=WAITING; s_pong.waiting_since=now;
    s_pong.nonce=esp_random(); if (!s_pong.nonce) s_pong.nonce=1;
    s_pong.paddle[0]=s_pong.paddle[1]=CENTER_Y-PADDLE_H*.5f;
    send_packet(HELLO);
}

static void reset_ball(unsigned next_side,int64_t now)
{
    s_pong.ball=(pong_ball_t){CENTER_X,CENTER_Y,next_side ? 60 : -60,
        (s_pong.sequence & 2) ? -17 : 17};
    s_pong.serve_until=now+SERVE_US;
    s_pong.stalled=false; s_pong.correction_seconds=0;
}

static void record_event(bool serve,int64_t when)
{
    s_pong.local_event=(pong_event_t){.sequence=s_pong.sequence,
        .flags=EVENT_VALID|(serve ? EVENT_SERVE : 0),.ball=s_pong.ball,
        .paddle=s_pong.paddle[s_pong.side],.score={s_pong.score[0],s_pong.score[1]}};
    s_pong.event_time=when;
    send_packet(STATE);
}

static void start_host(int64_t now)
{
    s_pong.mode=HUMAN; s_pong.side=0; s_pong.sequence=0;
    s_pong.last_tick=now;
    reset_ball(1,now); record_event(true,now);
}

static bool valid_event(const pong_event_t *e)
{
    pong_ball_t b=e->ball;
    return (e->flags & EVENT_VALID) && !(e->flags & ~(EVENT_VALID|EVENT_SERVE)) &&
        isfinite(b.x) && isfinite(b.y) && isfinite(b.vx) && isfinite(b.vy) && isfinite(e->paddle) &&
        b.y>=TOP && b.y<=BOTTOM && fabsf(b.vx)==60 && fabsf(b.vy)<=70 &&
        e->paddle>=TOP && e->paddle<=BOTTOM-PADDLE_H && e->score[0]<=999 && e->score[1]<=999 &&
        ((e->flags & EVENT_SERVE) ? b.x==CENTER_X : b.x==plane(e->sequence & 1));
}

static void accept_event(const pong_event_t *e,int64_t received,int64_t now)
{
    float old_x,old_y; visual_position(now,&old_x,&old_y);
    s_pong.sequence=e->sequence; s_pong.ball=e->ball;
    s_pong.score[0]=e->score[0]; s_pong.score[1]=e->score[1];
    s_pong.stalled=false;
    int64_t age=(int64_t)e->age_us+(now-received)+s_pong.rtt_us/2;
    int64_t hold=(e->flags & EVENT_SERVE) ? SERVE_US : 0;
    s_pong.serve_until=now+(age<hold ? hold-age : 0);
    if (age>hold) {
        // Never extrapolate through our own paddle; local simulation adjudicates it.
        float travel=(plane(s_pong.side)-s_pong.ball.x)/s_pong.ball.vx;
        float seconds=fminf((age-hold)*1e-6f,fmaxf(0,travel));
        pong_advance(&s_pong.ball,seconds,TOP,BOTTOM);
    }
    s_pong.last_tick=now;
    s_pong.correction_start=now;
    s_pong.correction_x=old_x-s_pong.ball.x;
    s_pong.correction_y=old_y-s_pong.ball.y;
    float to_middle=(CENTER_X-s_pong.ball.x)/s_pong.ball.vx;
    // Late packets beyond midfield use a short bounded correction instead.
    s_pong.correction_seconds=(e->flags & EVENT_SERVE) ? 0 : (to_middle>0 ? to_middle : .08f);
}

static void handle_packet(const pong_rx_t *rx,int64_t now)
{
    const pong_packet_t *p=&rx->packet;
    if (s_pong.mode==MENU || s_pong.mode==AI || same_mac(rx->mac,s_pong.mac)) return;
    if (p->kind==HELLO && s_pong.mode==WAITING) {
        int64_t ours=(rx->received_us-s_pong.waiting_since)/1000;
        bool first=ours>(int64_t)p->waiting_ms+100 ||
            (llabs(ours-(int64_t)p->waiting_ms)<=100 && memcmp(s_pong.mac,rx->mac,6)<0);
        if (!first) return;
        memcpy(s_pong.peer,rx->mac,6); s_pong.peer_nonce=p->nonce;
        s_pong.side=0; s_pong.session=esp_random(); if (!s_pong.session) s_pong.session=1;
        s_pong.mode=OFFERING; s_pong.last_rx=now;
        send_packet(OFFER); return;
    }
    if (p->kind==OFFER && s_pong.mode==WAITING && p->target_nonce==s_pong.nonce && p->session) {
        memcpy(s_pong.peer,rx->mac,6); s_pong.peer_nonce=p->nonce;
        s_pong.session=p->session; s_pong.side=1;
        s_pong.mode=CONNECTING; s_pong.last_rx=now;
        send_packet(ACCEPT); return;
    }
    if (!same_mac(rx->mac,s_pong.peer) || p->nonce!=s_pong.peer_nonce ||
        p->target_nonce!=s_pong.nonce || p->session!=s_pong.session || !p->session) return;
    if (p->kind==LEAVE) { enter_waiting(now); return; }
    if (p->kind==ACCEPT && s_pong.mode==OFFERING) { s_pong.last_rx=now; start_host(now); return; }
    if (p->kind==OFFER && s_pong.mode==CONNECTING) { send_packet(ACCEPT); return; }
    if (p->kind!=STATE || p->side!=1-s_pong.side ||
        (s_pong.mode!=HUMAN && s_pong.mode!=CONNECTING)) return;
    s_pong.last_rx=now;
    if (p->echo_us>0 && p->echo_us<=rx->received_us) {
        int64_t rtt=rx->received_us-p->echo_us-p->echo_delay_us;
        if (rtt>=0 && rtt<=500000 && (!s_pong.rtt_us || rtt<s_pong.rtt_us)) s_pong.rtt_us=(uint32_t)rtt;
    }
    if (p->sent_us>s_pong.peer_sent_us) {
        s_pong.peer_sent_us=p->sent_us; s_pong.peer_received_us=rx->received_us;
        s_pong.paddle[1-s_pong.side]=p->paddle;
    }
    const pong_event_t *e=&p->event;
    if (!valid_event(e)) return;
    if (s_pong.mode==CONNECTING) {
        if (e->sequence!=0 || !(e->flags & EVENT_SERVE) || e->ball.vx<=0) return;
        s_pong.mode=HUMAN; accept_event(e,rx->received_us,now); send_packet(STATE);
    } else if (e->sequence==s_pong.sequence+1 && (e->sequence & 1)==p->side &&
               ((p->side==0 && e->ball.vx>0) || (p->side==1 && e->ball.vx<0))) {
        accept_event(e,rx->received_us,now);
    }
}

void pong_handle_link_frame(link_msg_type_t type,const uint8_t *mac,const uint8_t *data,size_t len)
{
    if (!s_active || !s_rx || type!=LINK_MSG_GAME || !mac || !data || len!=sizeof(pong_packet_t)) return;
    pong_rx_t rx={.received_us=esp_timer_get_time()}; memcpy(&rx.packet,data,len);
    pong_packet_t *p=&rx.packet;
    if (p->game!=1 || p->version!=PONG_VERSION || p->kind<HELLO || p->kind>LEAVE ||
        p->side>1 || !p->nonce || !isfinite(p->paddle) || p->paddle<TOP || p->paddle>BOTTOM-PADDLE_H) return;
    memcpy(rx.mac,mac,6); (void)xQueueSend(s_rx,&rx,0);
}

static void simulate(float dt,int64_t start)
{
    if (s_pong.stalled) return;
    for (unsigned steps=0;dt>0 && steps<4;++steps) {
        unsigned side=s_pong.ball.vx<0 ? 0 : 1;
        float crossing=(plane(side)-s_pong.ball.x)/s_pong.ball.vx;
        if (crossing>dt) { pong_advance(&s_pong.ball,dt,TOP,BOTTOM); return; }
        crossing=fmaxf(0,crossing);
        pong_advance(&s_pong.ball,crossing,TOP,BOTTOM); s_pong.ball.x=plane(side);
        start+=(int64_t)(crossing*1000000); dt-=crossing;
        bool owner=s_pong.mode==AI || side==s_pong.side;
        if (owner && s_pong.mode==HUMAN && ((s_pong.sequence+1)&1)!=side) {
            s_pong.stalled=true; return; // missing preceding remote impact
        }
        bool hit=pong_paddle_hit(&s_pong.ball,s_pong.paddle[side],PADDLE_H,2);
        if (hit) pong_bounce(&s_pong.ball,side,s_pong.paddle[side],PADDLE_H);
        else if (!owner) { s_pong.stalled=true; return; }
        if (owner) {
            ++s_pong.sequence;
            if (!hit) {
                if (s_pong.score[1-side]<999) ++s_pong.score[1-side];
                reset_ball(1-side,start);
            }
            if (s_pong.mode==HUMAN) record_event(!hit,start);
            if (!hit) return;
        }
    }
}

void pong_service_init(void)
{
    s_active=false;
    if (!s_rx) s_rx=xQueueCreate(8,sizeof(pong_rx_t));
    if (s_rx) xQueueReset(s_rx);
    memset(&s_pong,0,sizeof(s_pong));
    (void)esp_read_mac(s_pong.mac,ESP_MAC_WIFI_STA);
    s_pong.paddle[0]=s_pong.paddle[1]=CENTER_Y-PADDLE_H*.5f;
    s_active=true;
}

void pong_service_deinit(void)
{
    if (s_pong.mode==HUMAN || s_pong.mode==CONNECTING || s_pong.mode==OFFERING) send_packet(LEAVE);
    s_active=false; s_pong.mode=MENU;
}

void pong_service_reset(void)
{
    pong_service_deinit();
    pong_service_init();
}

bool pong_service_human_available(void)
{
    return s_rx != NULL;
}

void pong_service_start_ai(void)
{
    if (!s_active || s_pong.mode != MENU) return;
    int64_t now=esp_timer_get_time();
    s_pong.mode=AI; s_pong.last_tick=now; reset_ball(1,now);
}

void pong_service_start_human(void)
{
    if (!s_active || !s_rx || s_pong.mode != MENU) return;
    enter_waiting(esp_timer_get_time());
}

void pong_service_move_paddle(int direction)
{
    if (s_pong.mode != AI && s_pong.mode != HUMAN) return;
    if (direction < -1) direction = -1;
    if (direction > 1) direction = 1;
    if (!direction) return;
    unsigned side=s_pong.side;
    s_pong.paddle[side]=clamp(s_pong.paddle[side]+direction*4.0f,TOP,BOTTOM-PADDLE_H);
    if (s_pong.mode==HUMAN) send_packet(STATE);
}

void pong_service_toggle_pause(void)
{
    if (s_pong.mode == AI) s_pong.paused=!s_pong.paused;
}

void pong_service_tick(void)
{
    if (!s_active) return;
    int64_t now=esp_timer_get_time();
    pong_rx_t rx;
    for (unsigned i=0;s_rx && i<8 && xQueueReceive(s_rx,&rx,0)==pdTRUE;++i) handle_packet(&rx,now);
    if (s_pong.mode==MENU) return;
    if (s_pong.mode!=AI) {
        if (s_pong.mode!=WAITING && now-s_pong.last_rx>TIMEOUT_US) { enter_waiting(now); return; }
        if (now-s_pong.last_tx>=HEARTBEAT_US)
            send_packet(s_pong.mode==WAITING ? HELLO : s_pong.mode==OFFERING ? OFFER :
                s_pong.mode==CONNECTING ? ACCEPT : STATE);
    }
    int64_t start=s_pong.last_tick; s_pong.last_tick=now;
    if (s_pong.mode!=AI && s_pong.mode!=HUMAN) return;
    if (s_pong.paused || now<=s_pong.serve_until) return;
    if (start<s_pong.serve_until) start=s_pong.serve_until;
    // Bound long shell stalls rather than doing an unbounded catch-up burst.
    float dt=clamp((now-start)*1e-6f,0,.1f);
    if (s_pong.mode==AI) {
        float target=s_pong.ball.y-PADDLE_H*.5f;
        s_pong.paddle[1]=clamp(s_pong.paddle[1]+clamp(target-s_pong.paddle[1],-36*dt,36*dt),TOP,BOTTOM-PADDLE_H);
    }
    simulate(dt,now-(int64_t)(dt*1000000));
}

void pong_service_snapshot(pong_view_t *out)
{
    if (!out) return;
    memset(out,0,sizeof(*out));
    switch (s_pong.mode) {
        case WAITING: out->mode=PONG_VIEW_WAITING; break;
        case OFFERING:
        case CONNECTING: out->mode=PONG_VIEW_CONNECTING; break;
        case HUMAN: out->mode=PONG_VIEW_HUMAN; break;
        case AI: out->mode=PONG_VIEW_AI; break;
        case MENU:
        default: out->mode=PONG_VIEW_MENU; break;
    }
    out->human_available=s_rx!=NULL;
    out->paused=s_pong.paused;
    out->side=(uint8_t)s_pong.side;
    out->paddle[0]=s_pong.paddle[0]; out->paddle[1]=s_pong.paddle[1];
    out->score[0]=s_pong.score[0]; out->score[1]=s_pong.score[1];
    visual_position(esp_timer_get_time(),&out->ball_x,&out->ball_y);
}
