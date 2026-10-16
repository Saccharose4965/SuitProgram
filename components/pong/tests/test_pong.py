"""Compile real Pong against two clocks, queued radio delivery and loss."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
STUBS = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define PANEL_W 128
#define PANEL_H 64
#define ESP_MAC_WIFI_STA 0
#define LINK_MSG_GAME 3
#define pdTRUE 1
typedef int link_msg_type_t;
typedef int TickType_t;
typedef void *QueueHandle_t;
int64_t esp_timer_get_time(void);
uint32_t esp_random(void);
int esp_read_mac(uint8_t *,int);
int link_send_frame_to(const uint8_t *,int,const uint8_t *,size_t,bool);
QueueHandle_t xQueueCreate(unsigned,size_t);
int xQueueReset(QueueHandle_t);
int xQueueSend(QueueHandle_t,const void *,int);
int xQueueReceive(QueueHandle_t,void *,int);
void oled_draw_text3x5(uint8_t *,int,int,const char *);
'''
TEST = r'''
#include <assert.h>
#include "pong.c"

static int current;
static int64_t wall=1000000, offsets[2]={0,9000000};
static uint32_t random_id=100;
static bool online[2], lossy;
static unsigned sent, dropped, delivered;
static struct { unsigned count; size_t size; uint8_t bytes[8][128]; } queues[2];
static struct { pong_state_t state; bool active; } nodes[2];
static struct { bool used; int target; int64_t due; uint8_t mac[6]; pong_packet_t packet; } wire[128];

int64_t esp_timer_get_time(void) { return wall+offsets[current]; }
uint32_t esp_random(void) { return ++random_id; }
int esp_read_mac(uint8_t *mac,int kind)
{ (void)kind; memset(mac,0,6); mac[0]=2; mac[5]=2-current; return 0; }
QueueHandle_t xQueueCreate(unsigned n,size_t size)
{ assert(n==8 && size<=128); queues[current].size=size; return &queues[current]; }
int xQueueReset(QueueHandle_t q)
{ for(int i=0;i<2;++i) if(q==&queues[i]) queues[i].count=0; return 1; }
int xQueueSend(QueueHandle_t q,const void *p,int ticks)
{
    assert(ticks==0);
    for(int i=0;i<2;++i) if(q==&queues[i]) {
        if(queues[i].count==8) return 0;
        memcpy(queues[i].bytes[queues[i].count++],p,queues[i].size); return 1;
    }
    assert(false); return 0;
}
int xQueueReceive(QueueHandle_t q,void *p,int ticks)
{
    assert(ticks==0);
    for(int i=0;i<2;++i) if(q==&queues[i]) {
        if(!queues[i].count) return 0;
        memcpy(p,queues[i].bytes[0],queues[i].size);
        --queues[i].count;
        memmove(queues[i].bytes,queues[i].bytes+1,queues[i].count*128); return 1;
    }
    assert(false); return 0;
}
int link_send_frame_to(const uint8_t *mac,int type,const uint8_t *data,size_t len,bool ack)
{
    assert(type==LINK_MSG_GAME && len==sizeof(pong_packet_t) && len<=200 && !ack);
    ++sent;
    if(lossy && sent%7==0) { ++dropped; return 0; }
    int dst=1-current;
    if(!online[dst] || (!same_mac(mac,broadcast) && !same_mac(mac,nodes[dst].state.mac))) return 0;
    for(unsigned i=0;i<128;++i) if(!wire[i].used) {
        wire[i].used=true; wire[i].target=dst;
        wire[i].due=wall+30000+(lossy ? (sent%5)*10000 : 0);
        memcpy(wire[i].mac,s_pong.mac,6); memcpy(&wire[i].packet,data,len); return 0;
    }
    assert(false); return 0;
}
void oled_draw_text3x5(uint8_t *fb,int x,int y,const char *text)
{ (void)fb; assert(x>=0 && y>=0 && x+(int)strlen(text)*4<=PANEL_W && y+5<=PANEL_H); }
static void select_node(int i)
{ current=i; s_pong=nodes[i].state; s_rx=&queues[i]; s_active=nodes[i].active; }
static void save_node(int i) { nodes[i].state=s_pong; nodes[i].active=s_active; }
static void press(unsigned button)
{ input_event_t e={.button=button,.type=INPUT_EVENT_PRESS}; pong_app_handle_input(&e); }
static void open_node(int i)
{
    current=i; s_rx=NULL; pong_app_init(); online[i]=true; save_node(i);
    assert(s_pong.mode==MENU);
    press(INPUT_BTN_C); press(INPUT_BTN_D); save_node(i);
    assert(s_pong.mode==WAITING);
}
static void step(bool auto_paddles)
{
    wall+=10000;
    for(int i=0;i<2;++i) if(online[i]) {
        select_node(i);
        for(unsigned j=0;j<128;++j) if(wire[j].used && wire[j].target==i && wire[j].due<=wall) {
            pong_handle_link_frame(LINK_MSG_GAME,wire[j].mac,(const uint8_t *)&wire[j].packet,sizeof(pong_packet_t));
            wire[j].used=false; ++delivered;
        }
        if(auto_paddles && s_pong.mode==HUMAN)
            s_pong.paddle[s_pong.side]=clamp(s_pong.ball.y-PADDLE_H*.5f,TOP,BOTTOM-PADDLE_H);
        pong_app_tick(.01f);
        uint8_t fb[PANEL_W*PANEL_H/8]={0}; pong_app_draw(fb,0,8,128,49);
        save_node(i);
    }
}

int main(void)
{
    // Reflection is timestep-independent, including multiple wall crossings.
    pong_ball_t a={30,25,60,-70}, b=a;
    pong_advance(&a,3,TOP,BOTTOM);
    for(int i=0;i<300;++i) pong_advance(&b,.01f,TOP,BOTTOM);
    assert(fabsf(a.x-b.x)<.01f && fabsf(a.y-b.y)<.01f && a.vy==b.vy);
    assert(pong_correction_weight(0,1)==1 && pong_correction_weight(1,1)==0);

    open_node(0); for(int i=0;i<40;++i) step(false); open_node(1);
    lossy=true;
    for(int i=0;i<300;++i) step(true);
    assert(nodes[0].state.mode==HUMAN && nodes[1].state.mode==HUMAN);
    assert(nodes[0].state.side==0 && nodes[1].state.side==1); // first waiter, not lowest MAC
    assert(nodes[0].state.session==nodes[1].state.session);
    select_node(1);
    float left=s_pong.paddle[0]; s_pong.paddle[1]=CENTER_Y;
    press(INPUT_BTN_B); assert(s_pong.paddle[0]==left && s_pong.paddle[1]==CENTER_Y-4);
    save_node(1);
    for(int i=0;i<6000;++i) {
        step(true);
        assert(nodes[0].state.mode==HUMAN && nodes[1].state.mode==HUMAN);
        int gap=(int)nodes[0].state.sequence-(int)nodes[1].state.sequence;
        assert(abs(gap)<=1);
    }
    assert(nodes[0].state.sequence>20 && nodes[1].state.sequence>20 && dropped>0);
    assert(nodes[0].state.rtt_us>0 && nodes[1].state.rtt_us>0);

    // Delayed authoritative impact preserves the visible position, then aligns by midfield.
    select_node(0); s_pong.side=0;
    s_pong.ball=(pong_ball_t){RIGHT_X-4,20,-60,0}; s_pong.correction_seconds=0;
    pong_event_t impact={.sequence=101,.flags=EVENT_VALID,.age_us=150000,
        .ball={RIGHT_X,35,-60,10},.paddle=28};
    int64_t now=esp_timer_get_time();
    accept_event(&impact,now,now);
    float vx,vy; visual_position(now,&vx,&vy);
    assert(fabsf(vx-(RIGHT_X-4))<.001f && fabsf(vy-20)<.001f);
    float seconds=(CENTER_X-s_pong.ball.x)/s_pong.ball.vx;
    pong_advance(&s_pong.ball,seconds,TOP,BOTTOM);
    visual_position(now+(int64_t)(seconds*1000000)+1,&vx,&vy);
    assert(fabsf(vx-s_pong.ball.x)<.001f && fabsf(vy-s_pong.ball.y)<.001f);

    // Local swept collision cannot tunnel through a paddle at a large dt.
    s_pong.mode=HUMAN; s_pong.sequence=101; s_pong.stalled=false;
    s_pong.ball=(pong_ball_t){LEFT_X+1,25,-60,0}; s_pong.paddle[0]=20;
    simulate(.1f,now); assert(s_pong.sequence==102 && s_pong.ball.vx>0 && s_pong.ball.x>LEFT_X);

    // Authoritative miss scores exactly once despite repeated collision snapshots.
    memset(wire,0,sizeof(wire)); lossy=false;
    for(int i=0;i<2;++i) {
        select_node(i); xQueueReset(s_rx);
        s_pong.sequence=1; s_pong.score[0]=s_pong.score[1]=0;
        s_pong.ball=(pong_ball_t){LEFT_X+.2f,BOTTOM,-60,0};
        s_pong.paddle[0]=TOP; s_pong.local_event.flags=0; s_pong.stalled=false;
        s_pong.correction_seconds=0; s_pong.serve_until=0;
        s_pong.last_tick=s_pong.last_rx=esp_timer_get_time(); save_node(i);
    }
    for(int i=0;i<50;++i) step(false);
    assert(nodes[0].state.sequence==2 && nodes[1].state.sequence==2);
    assert(nodes[0].state.score[1]==1 && nodes[1].state.score[1]==1);

    // Wrong peer/session, malformed data and stale rounds cannot change the match.
    select_node(1); uint32_t sequence=s_pong.sequence;
    pong_rx_t bad={.packet={.kind=STATE,.nonce=s_pong.peer_nonce,.target_nonce=s_pong.nonce,
        .session=s_pong.session,.side=0,.paddle=20},.received_us=esp_timer_get_time()};
    memset(bad.mac,9,6); bad.packet.event=nodes[0].state.local_event;
    handle_packet(&bad,esp_timer_get_time()); assert(s_pong.sequence==sequence);
    unsigned queued=queues[1].count;
    bad.packet.game=1; bad.packet.version=PONG_VERSION; bad.packet.paddle=NAN;
    pong_handle_link_frame(LINK_MSG_GAME,bad.mac,(uint8_t *)&bad.packet,sizeof(bad.packet));
    assert(queues[1].count==queued);

    // Exit stops reception; the peer returns to waiting rather than playing a ghost.
    pong_app_deinit(); save_node(1); online[1]=false;
    for(int i=0;i<350;++i) step(false);
    assert(nodes[0].state.mode==WAITING);
    current=1; s_rx=&queues[1]; pong_app_init(); press(INPUT_BTN_D);
    assert(s_pong.mode==AI); now=esp_timer_get_time();
    wall+=500000; pong_app_tick(.5f); assert(s_pong.ball.x>CENTER_X);
    press(INPUT_BTN_D); assert(s_pong.paused); float x=s_pong.ball.x;
    wall+=20000; pong_app_tick(.02f); assert(s_pong.ball.x==x);
    press(INPUT_BTN_A); assert(s_pong.mode==MENU);
    puts("Pong: two clocks, handshake, independent simulation, packet loss, collision authority, smoothing, score and exit passed");
    return 0;
}
'''


class PongTests(unittest.TestCase):
    def test_multiplayer(self):
        with tempfile.TemporaryDirectory(prefix="suit-pong-") as tmp:
            path=Path(tmp)
            (path / "stubs.h").write_text(STUBS)
            for name in ("link.h", "oled.h", "esp_mac.h", "esp_timer.h", "esp_random.h",
                         "freertos/FreeRTOS.h", "freertos/queue.h"):
                target=path/name
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_text('#include "stubs.h"\n')
            (path / "test.c").write_text(TEST)
            subprocess.run(["gcc","-std=c11","-O1","-Wall","-Wextra","-Werror",
                "-fsanitize=address,undefined","-I",str(path),
                "-I",str(ROOT / "components/pong"),"-I",str(ROOT / "components/pong/include"),
                "-I",str(ROOT / "components/input/include"),str(path / "test.c"),
                str(ROOT / "components/pong/pong_physics.c"),"-lm","-o",str(path / "test")],check=True)
            subprocess.run([str(path / "test")],check=True)


if __name__=="__main__":
    unittest.main()
