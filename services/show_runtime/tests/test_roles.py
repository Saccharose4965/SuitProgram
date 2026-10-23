"""Three runtime instances: discovery, retryable swaps and live joining."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_runtime import ROOT, STUBS, TEST


SCENARIOS = r'''
static show_runtime_t nodes[3];
static bool online[3];

static void select_node(unsigned i) { s_runtime=nodes[i]; memcpy(device_mac,s_runtime.own_mac,6); }
static void save_node(unsigned i) { nodes[i]=s_runtime; }
static void open_node(unsigned i) {
    memset(&s_runtime,0,sizeof(s_runtime));
    device_mac[5]=i+1;
    assert(show_runtime_init()==0);
    s_runtime.boot=100+i;
    show_runtime_set_menu_open(true);
    online[i]=true; save_node(i);
}
static void round_network(int drop_target) {
    now+=100000;
    for (unsigned src=0;src<3;++src) if (online[src]) {
        show_runtime_t *n=&nodes[src];
        show_wire_hello_t hello={SHOW_SYNC_PROTOCOL,SHOW_WIRE_HELLO,sizeof(hello),
            n->role_id,n->authority,n->menu_open||n->output_owned,n->role_ready,
            n->wanted_role,n->boot,n->role_request};
        show_rx_event_t event={.len=sizeof(hello),.received_us=now};
        memcpy(event.src_mac,n->own_mac,6); memcpy(event.data,&hello,sizeof(hello));
        for (unsigned dst=0;dst<3;++dst) if (online[dst] && src!=dst) {
            select_node(dst); handle_hello(&event,now); save_node(dst);
        }
    }
    for (unsigned i=0;i<3;++i) if (online[i]) {
        select_node(i); service_roles(now); save_node(i);
    }
    for (unsigned src=0;src<3;++src) if (online[src]) {
        select_node(src);
        if (!same_mac(role_coordinator(now),s_runtime.own_mac) || !s_runtime.roles.count) continue;
        show_rx_event_t event={.len=sizeof(s_runtime.roles),.received_us=now};
        memcpy(event.src_mac,s_runtime.own_mac,6); memcpy(event.data,&s_runtime.roles,event.len);
        for (unsigned dst=0;dst<3;++dst) if (online[dst] && dst!=src && (int)dst!=drop_target) {
            select_node(dst); handle_roles(&event,now); save_node(dst);
        }
    }
}
static void settle(void) { for (unsigned i=0;i<12;++i) round_network(-1); }
static void unique(void) {
    unsigned mask=0;
    for (unsigned i=0;i<3;++i) if (online[i]) {
        assert(nodes[i].role_ready && nodes[i].wanted_role==UINT8_MAX);
        assert(!(mask & (1u<<nodes[i].role_id))); mask|=1u<<nodes[i].role_id;
    }
}

int main(void) {
    // Lower-MAC newcomers may become coordinator, but cannot steal an existing role.
    open_node(2); assert(!nodes[2].role_ready); settle(); assert(nodes[2].role_id==0);
    open_node(1); settle(); assert(nodes[2].role_id==0 && nodes[1].role_id==1);
    open_node(0); settle(); unique(); assert(nodes[0].role_id==2);
    assert(package_loads==0); // discovery and idle role work never touches the SD
    show_wire_roles_t old_map=nodes[0].roles;

    select_node(2); assert(show_runtime_load("test",s_runtime.role_id)==0); save_node(2);
    unsigned loads=package_loads;
    select_node(2); assert(show_runtime_set_role(1)==0); save_node(2);
    for (unsigned i=0;i<10;++i) round_network(2); // requester misses acknowledgements
    settle(); unique(); assert(nodes[2].role_id==1 && nodes[1].role_id==0);
    assert(package_loads==loads); // even a previously loaded idle show is not reloaded
    settle(); assert(nodes[2].role_id==1 && nodes[1].role_id==0); // retries do not swap back
    select_node(2);
    show_rx_event_t stale={.len=sizeof(old_map),.received_us=now};
    memcpy(stale.src_mac,nodes[0].own_mac,6); memcpy(stale.data,&old_map,sizeof(old_map));
    handle_roles(&stale,now); assert(s_runtime.role_id==1); // delayed snapshot cannot undo a swap
    old_map.revision=s_runtime.roles.revision+10;
    old_map.entries[1].role=old_map.entries[0].role;
    memcpy(stale.data,&old_map,sizeof(old_map));
    handle_roles(&stale,now); assert(s_runtime.roles.revision!=old_map.revision);
    save_node(2);

    // Concurrent requests are serialized; eventually everybody has a unique role.
    select_node(1); assert(show_runtime_set_role(2)==0); save_node(1);
    select_node(2); assert(show_runtime_set_role(2)==0); save_node(2);
    settle(); unique(); assert(package_loads==loads);

    // Playback leader is the starter, not the role coordinator; live swap retains audio/session.
    select_node(2); assert(show_runtime_start_master()==0); save_node(2);
    settle(); assert(nodes[2].authority && !nodes[0].authority && !nodes[1].authority);
    uint32_t session=nodes[2].session_id;
    unsigned before_stops=stops;
    uint8_t wanted=nodes[1].role_id, old=nodes[2].role_id;
    select_node(2); assert(show_runtime_set_role(wanted)==0); save_node(2);
    settle(); unique(); assert(nodes[2].role_id==wanted && nodes[1].role_id==old);
    select_node(2);
    assert(s_runtime.authority && s_runtime.audio_owned && s_runtime.session_id==session);
    loads=package_loads;
    // Keep the mocked audio in its startup window while testing the one necessary role load.
    s_runtime.audio_request_us=now;
    show_runtime_service_tick(0);
    assert(package_loads==loads+1 && stops==before_stops && s_runtime.authority);
    assert(s_runtime.session_id==session && s_runtime.program.role.role_id==wanted);
    save_node(2);
    settle();
    for (unsigned i=0;i<3;++i) {
        select_node(i); assert(show_runtime_get_status().master_roles==(1u<<wanted));
    }

    // A late follower accounts for the entire 1.2 seconds spent loading the role.
    select_node(1);
    s_runtime.program_loaded=false; s_runtime.state=SHOW_RUNTIME_STOPPED;
    s_runtime.have_master=false; s_runtime.synced=false;
    show_wire_transport_t wire={.protocol=SHOW_SYNC_PROTOCOL,.kind=SHOW_WIRE_TRANSPORT,
        .bytes=sizeof(wire),.session_id=session,.generation=10,.show_uid=7,.leadership_term=1,
        .duration_ms=60000,.playhead_ms=10000,.role_count=3,.state=SHOW_RUNTIME_PLAYING};
    strcpy(wire.slug,"test");
    show_rx_event_t event={.len=sizeof(wire),.received_us=now};
    memcpy(event.src_mac,nodes[2].own_mac,6); memcpy(event.data,&wire,sizeof(wire));
    load_cost_us=1200000; loads=package_loads;
    handle_transport(&event,now);
    assert(current_playhead_ms(now)==11200 && package_loads==loads+1);
    assert(!s_runtime.authority && s_runtime.have_master && s_runtime.output_owned);
    handle_transport(&event,now); assert(package_loads==loads+1);
    load_cost_us=0; save_node(1);

    // Coordinator departure preserves the surviving suits' roles and playback leader.
    uint8_t a=nodes[1].role_id,b=nodes[2].role_id;
    online[0]=false;
    for (unsigned i=0;i<35;++i) round_network(-1);
    unique(); assert(nodes[1].role_id==a && nodes[2].role_id==b && nodes[2].authority);
    open_node(0); settle(); unique(); assert(nodes[1].role_id==a && nodes[2].role_id==b);

    // A follower explicitly starts a new song, even while the old leader plays.
    select_node(1);
    assert(show_runtime_load("test",s_runtime.role_id)==0);
    assert(show_runtime_start_master()==0);
    assert(s_runtime.leadership_term==2 && s_runtime.authority);
    save_node(1);
    wire.leadership_term=2; wire.session_id=s_runtime.session_id;
    wire.generation=s_runtime.generation; wire.state=SHOW_RUNTIME_PAUSED;
    wire.playhead_ms=0;
    memcpy(event.src_mac,nodes[1].own_mac,6); memcpy(event.data,&wire,sizeof(wire));
    event.received_us=now;
    select_node(2); s_runtime.audio_started=true;
    before_stops=stops; loads=package_loads;
    handle_transport(&event,now);
    assert(!s_runtime.authority && !s_runtime.audio_owned && stops==before_stops+1);
    assert(s_runtime.leadership_term==2 && package_loads==loads);
    handle_transport(&event,now); assert(stops==before_stops+1); // idempotent repeat
    save_node(2);
    select_node(1);
    wire.leadership_term=1;
    memcpy(event.src_mac,nodes[2].own_mac,6); memcpy(event.data,&wire,sizeof(wire));
    handle_transport(&event,now);
    assert(s_runtime.authority && s_runtime.leadership_term==2);
    // Same-term simultaneous starts resolve by MAC, independently of audio timing.
    wire.leadership_term=2; memcpy(event.data,&wire,sizeof(wire));
    memcpy(event.src_mac,nodes[0].own_mac,6);
    handle_transport(&event,now);
    assert(!s_runtime.authority && same_mac(s_runtime.master_mac,nodes[0].own_mac));
    show_wire_release_t release={.protocol=SHOW_SYNC_PROTOCOL,.kind=SHOW_WIRE_RELEASE,
        .bytes=sizeof(release),.session_id=s_runtime.session_id,
        .generation=UINT32_MAX,.leadership_term=1};
    show_rx_event_t stop_event={.len=sizeof(release),.received_us=now};
    memcpy(stop_event.src_mac,nodes[0].own_mac,6);
    memcpy(stop_event.data,&release,sizeof(release));
    handle_release(&stop_event,now);
    assert(s_runtime.have_master && s_runtime.output_owned); // old stop cannot stop new song
    assert(!master_packet_allowed(nodes[2].own_mac,2,now));
    assert(master_packet_allowed(nodes[2].own_mac,3,now));

    // Simultaneous cold startup also converges without duplicate roles.
    for(unsigned i=0;i<3;++i) open_node(i);
    settle(); unique();
    puts("Three suits: discovery, swaps, loss/retry, concurrent requests, leader, late join, rejoin passed");
}
'''


class RolesTest(unittest.TestCase):
    def test_three_suits(self):
        with tempfile.TemporaryDirectory(prefix="suit-roles-") as tmp:
            path = Path(tmp)
            (path / "stubs.h").write_text(STUBS)
            for name in ("esp_err.h", "esp_mac.h", "esp_random.h", "esp_timer.h", "esp_log.h",
                         "esp_heap_caps.h", "link.h", "freertos/FreeRTOS.h", "freertos/queue.h",
                         "led.h", "led_layout.h", "led_modes.h", "audio_player.h", "audio.h"):
                target = path / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "stubs.h"\n')
            (path / "test.c").write_text(TEST.split("int main(void)")[0] + SCENARIOS)
            (path / "track.wav").touch()
            subprocess.run(["gcc", "-std=c11", "-D_POSIX_C_SOURCE=200809L", "-Wall", "-Wextra",
                            "-Werror", "-O1", "-fsanitize=address,undefined", "-I", str(path),
                            "-I", str(ROOT / "services/show_package/include"),
                            "-I", str(ROOT / "services/show_renderer/include"),
                            "-I", str(ROOT / "services/show_runtime/include"),
                            "-I", str(ROOT / "services/show_runtime"),
                            "-I", str(ROOT / "services/suit_geometry/include"),
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], cwd=path, check=True)


if __name__ == "__main__":
    unittest.main()
