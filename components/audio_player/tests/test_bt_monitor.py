"""Exercise the production playback monitor with scripted BT status snapshots."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class MonitorTest(unittest.TestCase):
    def test_stall_disconnect_and_drain(self):
        source = (Path(__file__).resolve().parents[1] / "audio_player.c").read_text()
        start = source.index("                bool drain_stop_requested")
        end = source.index("            } else if (bt_audio_can_stream()", start)
        monitor = source[start:end]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define pdMS_TO_TICKS(ms) (ms)
#define BT_PROGRESS_STALL_MS 1200
#define BT_AUDIO_STATE_CONNECTED 1
#define BT_AUDIO_STATE_STREAMING 2
#define AUDIO_PLAYER_MODE_BT 1
typedef struct { int state; bool streaming; size_t queued_frames; } bt_audio_status_t;
static bool s_abort, s_paused;
static int s_play_mode = AUDIO_PLAYER_MODE_BT;
static int scenario, stops, diagnostics;
static int64_t now;
static bool drained;
static int64_t esp_timer_get_time(void) { return now; }
static void vTaskDelay(int ms) { now += ms * 1000; assert(now < 10000000); }
static bool bt_audio_can_stream(void) { return true; }
static bool bt_audio_media_cmd_pending(void) { return false; }
static void bt_audio_log_stream_health(void) { ++diagnostics; }
static void playback_diagnostics_sample(bool stalled) { (void)stalled; }
static int bt_audio_stop_stream(void) { ++stops; drained = true; return 0; }
static void bt_audio_get_status(bt_audio_status_t *st) {
    *st = (bt_audio_status_t){BT_AUDIO_STATE_STREAMING, true, 1000};
    if (scenario == 0) { // stall, recover, stall again, then disconnect
        if (now >= 2000000) st->queued_frames = 900;
        if (now >= 4000000) st->state = 0;
    } else if (scenario == 1) { // disconnect while user-paused
        if (now >= 200000) st->state = 0;
    } else if (scenario == 2) { // natural EOF: exactly one stop
        st->queued_frames = 0;
        if (drained) st->streaming = false;
    } else if (scenario == 3) { // explicit stop while stalled
        if (now >= 2000000) s_abort = true;
    }
}
static void run_monitor(void) {
MONITOR
}
int main(void) {
    for (scenario=0; scenario<4; ++scenario) {
        now=0; stops=diagnostics=0; s_abort=drained=false; s_paused=scenario==1;
        run_monitor();
        assert(stops==1);
        assert(diagnostics == (scenario==0 ? 2 : scenario==3 ? 1 : 0));
    }
}
'''.replace("MONITOR", monitor)
        # No Start/Pause mocks: automatic restart commands fail compilation.
        with tempfile.TemporaryDirectory(prefix="suit-bt-monitor-") as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(harness)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", str(path / "test.c"),
                            "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
