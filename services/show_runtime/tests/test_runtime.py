"""Compile the actual transport runtime against deterministic host devices."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

STUBS = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_INVALID_ARG 2
#define ESP_ERR_INVALID_STATE 3
#define ESP_ERR_NOT_FOUND 4
#define ESP_MAC_WIFI_STA 0
#define LED_STRIP_LENGTH 8
#define LED_OUTPUT_OWNER_SHOW 1
#define pdTRUE 1
static void mock_log(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; }
#define ESP_LOGI(...) mock_log(__VA_ARGS__)
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
static size_t heap_caps_get_free_size(int cap) { (void)cap; return 16000; }
static size_t heap_caps_get_largest_free_block(int cap) { (void)cap; return 8000; }
typedef int link_msg_type_t;
#define LINK_MSG_CONTROL 1
typedef void *QueueHandle_t;
typedef struct { uint64_t frames; uint32_t rate; int64_t updated_us; bool bluetooth; } audio_progress_t;
static int64_t now = 10000000;
static bool playing, paused;
static unsigned stops, sends;
static audio_progress_t progress;
static uint8_t device_mac[6] = {2,0,0,0,0,2};
static int64_t esp_timer_get_time(void) { return now; }
static unsigned esp_random(void) { return 42; }
static int esp_read_mac(uint8_t *p, int kind) { (void)kind; memcpy(p,device_mac,6); return 0; }
static const char *esp_err_to_name(int e) { (void)e; return "error"; }
static QueueHandle_t xQueueCreate(unsigned n, size_t s) { (void)n; (void)s; return (void *)1; }
static int xQueueReceive(QueueHandle_t q, void *p, int t) { (void)q; (void)p; (void)t; return 0; }
static int xQueueSend(QueueHandle_t q, const void *p, int t) { (void)q; (void)p; (void)t; return 1; }
static int link_send_frame_to(const uint8_t *mac, int type, const uint8_t *p, size_t len, bool ack)
{ (void)mac; (void)type; (void)p; (void)len; (void)ack; ++sends; return 0; }
static int led_init(void) { return 0; }
static bool led_modes_enabled(void) { return false; }
static bool led_beat_enabled(void) { return false; }
static bool led_audio_brightness_enabled(void) { return false; }
static int led_output_claim(int owner) { (void)owner; return 0; }
static void led_output_release(int owner) { (void)owner; }
static void led_modes_enable(bool b) { (void)b; }
static void led_beat_enable(bool b) { (void)b; }
static void led_audio_brightness_enable(bool b) { (void)b; }
static unsigned led_submissions;
static int led_show_pixels_owned(int o, const uint8_t *p, size_t n) { (void)o; (void)p; (void)n; ++led_submissions; return 0; }
static size_t led_layout_count(void) { return 8; }
static uint8_t led_modes_get_brightness(void) { return 255; }
static void audio_player_stop(void) { playing = false; ++stops; }
static int audio_player_play_show(const char *path) { (void)path; playing = true; paused = false; progress = (audio_progress_t){0}; return 0; }
static bool audio_player_is_active(void) { return playing; }
static bool audio_player_is_paused(void) { return paused; }
static int audio_player_toggle_play_pause(void) { paused = !paused; return 0; }
static audio_progress_t audio_progress_get(void) { return progress; }
'''

TEST = r'''
#include <assert.h>
#include "show_runtime.c"
struct show_renderer { int unused; };
static struct show_renderer renderer;
static unsigned render_calls;
static int64_t render_cost_us;
static uint32_t last_render_playhead;
static unsigned package_loads;
static int64_t load_cost_us;
show_renderer_t *show_renderer_create(void) { return &renderer; }
void show_renderer_destroy(show_renderer_t *r) { (void)r; }
int show_renderer_configure(show_renderer_t *r, const show_package_program_t *p, uint8_t role)
{ (void)r; (void)p; (void)role; return 0; }
int show_renderer_render(show_renderer_t *r, uint32_t ms, uint8_t b, uint8_t *f, size_t n, size_t *out)
{ (void)r; (void)b; (void)f; *out = n; ++render_calls;
  last_render_playhead=ms; now+=render_cost_us; return 0; }
int show_package_build_path(const char *slug, char *out, size_t n)
{ (void)slug; snprintf(out,n,"./show.bin"); return 0; }
int show_package_load_program(const char *s, uint8_t role, show_package_program_t *p)
{ (void)s; ++package_loads; now+=load_cost_us; memset(p,0,sizeof(*p)); p->role.role_id=role; p->info.show_uid=7; p->info.duration_ms=60000; p->info.role_count=3; return 0; }
void show_package_program_free(show_package_program_t *p) { (void)p; }

int main(void) {
    s_diagnostic_no_leds = true;
    assert(submit_show_frame(8) == ESP_OK && led_submissions == 0);
    s_diagnostic_no_leds = false;
    assert(submit_show_frame(8) == ESP_OK && led_submissions == 1);
    assert(show_runtime_init() == 0);
    // Closed menus cannot be commandeered by a remote show.
    uint8_t other[6] = {2,0,0,0,0,1};
    assert(!master_packet_allowed(other,1,now));
    show_runtime_set_menu_open(true);
    assert(master_packet_allowed(other,1,now));
    show_peer_t *peer = find_peer(other,true);
    peer->seen_us = now; peer->role = 0; peer->role_ready=true; peer->boot=123;
    show_runtime_service_tick(0);
    assert(!s_runtime.role_ready); // discovery asks before claiming a role
    now+=700000;
    show_wire_roles_t roles={.protocol=SHOW_SYNC_PROTOCOL,.kind=SHOW_WIRE_ROLES,
        .bytes=sizeof(roles),.boot=123,.revision=1,.count=2};
    memcpy(roles.entries[0].mac,other,6); roles.entries[0].role=0; roles.entries[0].boot=123;
    memcpy(roles.entries[1].mac,device_mac,6); roles.entries[1].role=1; roles.entries[1].boot=s_runtime.boot;
    show_rx_event_t assignment={.len=sizeof(roles),.received_us=now};
    memcpy(assignment.src_mac,other,6); memcpy(assignment.data,&roles,sizeof(roles));
    handle_roles(&assignment,now);
    assert(s_runtime.role_ready && s_runtime.role_id == 1);
    assert(show_runtime_get_status().present_roles == 3);
    assert(!show_runtime_get_status().role_conflict);
    assert(show_runtime_load("test",1) == 0);
    assert(show_runtime_start_master() == 0);
    assert(playing && s_runtime.state == SHOW_RUNTIME_PAUSED);
    assert(master_packet_allowed(other,s_runtime.leadership_term,now)); // simultaneous start tie-break
    // No LED free-running while opening/prebuffering audio.
    now += 1000000;
    show_runtime_service_tick(0);
    assert(current_playhead_ms(now) == 0);
    progress = (audio_progress_t){44100,44100,now,true};
    show_runtime_service_tick(0);
    assert(s_runtime.audio_started && s_runtime.state == SHOW_RUNTIME_PLAYING);
    assert(!master_packet_allowed(other,0,now)); // retired/invalid terms never win
    assert(master_packet_allowed(other,s_runtime.leadership_term+1,now)); // explicit new start wins
    assert(current_playhead_ms(now) == 900); // 100 ms default BT compensation
    now += 500000;
    show_runtime_service_tick(0);
    assert(s_runtime.state == SHOW_RUNTIME_PAUSED); // stalled PCM freezes show
    assert(current_playhead_ms(now) == 1000);
    progress.frames += 4410; progress.updated_us = now;
    show_runtime_service_tick(0);
    assert(s_runtime.state == SHOW_RUNTIME_PLAYING);
    assert(current_playhead_ms(now) >= 1000); // no backwards motion on resume
    assert(show_runtime_toggle_master_pause() == 0);
    assert(paused);
    assert(show_runtime_toggle_master_pause() == 0);
    assert(!paused);
    show_runtime_stop(true);
    assert(!playing && stops);

    // A coarse broadcast cannot replace the corrected clock anchor.
    s_runtime.have_master=true; memcpy(s_runtime.master_mac,other,6);
    s_runtime.leadership_term=2; memcpy(s_runtime.leadership_mac,other,6);
    s_runtime.synced=true; s_runtime.session_id=5; s_runtime.generation=2;
    s_runtime.state=SHOW_RUNTIME_PLAYING; s_runtime.anchor_local_us=1234;
    show_wire_transport_t wire = {.protocol=SHOW_SYNC_PROTOCOL,.kind=SHOW_WIRE_TRANSPORT,
        .bytes=sizeof(wire),.session_id=5,.generation=2,.show_uid=7,.leadership_term=2,
        .duration_ms=60000,.role_count=3,.state=SHOW_RUNTIME_PLAYING};
    strcpy(wire.slug,"test");
    show_rx_event_t event={.len=sizeof(wire),.received_us=now};
    memcpy(event.src_mac,other,6); memcpy(event.data,&wire,sizeof(wire));
    handle_transport(&event,now);
    assert(s_runtime.anchor_local_us==1234 && s_runtime.synced);
    wire.generation=3; wire.state=SHOW_RUNTIME_PAUSED; wire.playhead_ms=900;
    memcpy(event.data,&wire,sizeof(wire)); handle_transport(&event,now);
    assert(s_runtime.state==SHOW_RUNTIME_PAUSED && current_playhead_ms(now)==900);
    // Expired peers disappear, and leaving an idle menu opts out.
    show_runtime_stop(false); show_runtime_set_menu_open(false);
    now += SHOW_SYNC_STALE_US + 1;
    assert(show_runtime_get_status().peer_count==0);
    assert(!master_packet_allowed(other,5,now));
    assert(show_runtime_load("test",1)==0);
    assert(remove("./track.wav")==0);
    assert(show_runtime_start_master()==ESP_ERR_NOT_FOUND);
    assert(!show_runtime_owns_leds());
    // Older 30 Hz package hints still render at the 60 Hz runtime target.
    s_runtime.output_owned=true; s_runtime.state=SHOW_RUNTIME_PLAYING;
    s_runtime.anchor_local_us=now; s_runtime.anchor_playhead_ms=0;
    s_runtime.program.info.fps_hint=30; s_runtime.last_render_us=0;
    render_calls=0;
    for (unsigned i=0;i<600;++i) { now+=16660; show_runtime_service_tick(0); }
    assert(render_calls>=598 && render_calls<=600);
    // Slow frames back off immediately, but sample current song time, not a queue.
    s_runtime.program.info.duration_ms=1000000;
    render_cost_us=20000;
    now+=40000; show_runtime_service_tick(0);
    assert(s_runtime.render_budget_us==20000);
    unsigned slow_calls=render_calls;
    show_runtime_service_tick(0);
    assert(render_calls==slow_calls);
    now+=50000;
    uint32_t expected_playhead=current_playhead_ms(now);
    show_runtime_service_tick(0);
    assert(last_render_playhead==expected_playhead && render_calls==slow_calls+1);
    // Cheap sections gradually regain the 60 Hz budget.
    render_cost_us=1000;
    for (unsigned i=0;i<120;++i) { now+=50000; show_runtime_service_tick(0); }
    assert(s_runtime.render_budget_us<2000);
    render_cost_us=0;
    s_runtime.state=SHOW_RUNTIME_PAUSED;
    s_runtime.paused_playhead_ms=1000;
    now+=40000; show_runtime_service_tick(0);
    unsigned held_calls=render_calls;
    for (unsigned i=0;i<100;++i) { now+=40000; show_runtime_service_tick(0); }
    assert(render_calls==held_calls); // identical paused frame is not recomputed
    unsigned before=sends;
    broadcast_transport(now); // no live peers: don't consume radio airtime
    assert(sends==before);
    peer->seen_us=now;
    broadcast_transport(now);
    assert(sends>before); // discovery re-enables transport
    puts("Show runtime: discovery, roles, audio clock, stalls, pause and sync passed");
}
'''


class RuntimeTest(unittest.TestCase):
    def test_navigation_has_no_package_io(self):
        ui = (ROOT / "apps/show_player/show_player.c").read_text()
        navigation = ui[ui.index("void show_player_app_handle_input"):
                        ui.index("if (ev->button == INPUT_BTN_C)")]
        self.assertNotIn("show_package_load_info", ui)
        for operation in ("storage_mount_sd", "show_player_scan", "show_runtime_load", "fopen"):
            self.assertNotIn(operation, navigation)
        self.assertEqual(ui.count("show_runtime_load("), 1)

    def test_runtime(self):
        with tempfile.TemporaryDirectory(prefix="suit-show-test-") as tmp:
            path = Path(tmp)
            (path / "stubs.h").write_text(STUBS)
            for name in ("esp_err.h", "esp_mac.h", "esp_random.h", "esp_timer.h", "esp_log.h", "esp_heap_caps.h", "link.h",
                         "freertos/FreeRTOS.h", "freertos/queue.h", "led.h", "led_layout.h",
                         "led_modes.h", "audio_player.h", "audio.h"):
                target = path / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "stubs.h"\n')
            ui = (ROOT / "apps/show_player/show_player.c").read_text()
            def function(start, end):
                return ui[ui.index(start):ui.index(end, ui.index(start))]
            draw = ui[ui.index("void show_player_app_draw("):]
            helpers = function("static const char *show_role_name", "static bool show_player_make_child_path")
            helpers += function("static void show_player_copy_row_label", "static void show_player_set_status")
            display_test = r'''
typedef struct { int unused; } shell_app_context_t;
static struct { bool mounted; uint8_t role_id; int count; size_t selected;
                char shows[32][28]; } s_show;
static int bound_x, bound_y, bound_w, bound_h;
static void oled_draw_text3x5(uint8_t *fb, int x, int y, const char *text) {
    (void)fb;
    assert(x >= bound_x && y >= bound_y);
    assert(y + 5 <= bound_y + bound_h);
    assert(x + (int)strlen(text)*4 <= bound_x + bound_w);
}
'''
            checks = r'''
    uint8_t fb[1024];
    for (int h=0; h<=64; ++h) for (int w=0; w<=128; ++w) {
        bound_x=3; bound_y=7; bound_w=w; bound_h=h;
        for (int n=0; n<=32; ++n) {
            s_show.count=n; s_show.mounted=true;
            for (int row=0; row<n+3; ++row) {
                s_show.selected=row;
                show_player_app_draw(NULL,fb,bound_x,bound_y,w,h);
            }
        }
    }
    puts("Show UI: every list selection stays inside content bounds");
'''
            source = TEST.replace("int main(void) {", display_test + helpers + draw + "\nint main(void) {" + checks)
            (path / "test.c").write_text(source)
            (path / "track.wav").touch()
            subprocess.run(["gcc", "-std=c11", "-D_POSIX_C_SOURCE=200809L", "-Wall", "-Wextra", "-Werror", "-O1",
                            "-fsanitize=address,undefined", "-I", str(path),
                            "-I", str(ROOT / "services/show_package/include"),
                            "-I", str(ROOT / "services/show_renderer/include"),
                            "-I", str(ROOT / "services/show_runtime/include"),
                            "-I", str(ROOT / "services/suit_geometry/include"),
                            "-I", str(ROOT / "services/show_runtime"),
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], cwd=path, check=True)


if __name__ == "__main__":
    unittest.main()
