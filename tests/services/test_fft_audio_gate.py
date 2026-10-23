"""Exercise the actual FFT request function with competing audio owners."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class AudioGateTest(unittest.TestCase):
    def test_wait_and_resume(self):
        source = (Path(__file__).resolve().parents[2] / "services/audio_policy/audio_policy.c").read_text()
        function = source[source.index("void audio_policy_fft_request("):]
        function += source[source.index("void audio_policy_fft_set_paint_app("):source.index("bool audio_policy_fft_blocked_by_bluetooth(")]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_LOGW(...) ((void)0)
static uint32_t s_audio_policy_fft_need_mask;
static bool s_audio_policy_fft_owned, s_show_app_active, s_call_app_active, s_paint_app_active;
static bool s_paint_threedee_active, s_paint_armpose_active;
static bool s_synth_app_active;
static bool recording, playback, output, running, beat, bt;
static int starts, inits;
static bool show_runtime_owns_leds(void) { return false; }
static bool audio_policy_fft_blocked_by_bluetooth(void) { return bt; }
static bool fft_visualizer_running(void) { return running; }
static void fft_visualizer_stop(void) { running=false; }
static bool led_beat_enabled(void) { return beat; }
static bool led_audio_brightness_enabled(void) { return false; }
static bool mic_rec_is_recording(void) { return recording; }
static bool audio_player_is_active(void) { return playback; }
static bool audio_output_active(void) { return output; }
static bool audio_bus_init_if_needed(void) { ++inits; return true; }
static int fft_visualizer_start(void) { if (!running) ++starts; running=true; return 0; }
static int64_t esp_timer_get_time(void) { return 0; }
FUNCTION
int main(void) {
    recording=true;
    for (int i=0;i<100;i++) audio_policy_fft_request(1,true);
    assert(!starts && !inits && s_audio_policy_fft_need_mask==1);
    recording=false; playback=true; audio_policy_fft_request(0,false);
    assert(!starts && !inits);
    playback=false; output=true; audio_policy_fft_request(0,false);
    assert(!starts && !inits);
    output=false; audio_policy_fft_request(0,false);
    assert(starts==1 && inits==1 && running);
    audio_policy_fft_request(1,false);
    assert(!running && !s_audio_policy_fft_need_mask);
    beat=true; recording=true; audio_policy_fft_request(0,false);
    assert(starts==1);
    recording=false; audio_policy_fft_request(0,false);
    assert(starts==2 && running);
    audio_policy_fft_request(1,true);
    audio_policy_fft_set_paint_app(true);
    assert(!running && s_audio_policy_fft_need_mask==1);
    for(int i=0;i<100;i++) audio_policy_fft_request(1,true);
    assert(!running && starts==2);
    audio_policy_fft_set_paint_app(false);
    assert(!running);
    audio_policy_fft_request(0,false);
    assert(running && starts==3);
    bt=true; audio_policy_fft_request(0,false);
    assert(!running && starts==3);
    audio_policy_fft_set_paint_app(true);
    audio_policy_fft_set_paint_app(false);
    audio_policy_fft_request(0,false);
    assert(!running && starts==3); // exiting paint must not bypass BT ownership
}
'''.replace("FUNCTION", function)
        with tempfile.TemporaryDirectory(prefix="suit-audio-gate-") as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(harness)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
