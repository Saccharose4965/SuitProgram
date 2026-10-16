"""Exercise the actual FFT request function with competing audio owners."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class AudioGateTest(unittest.TestCase):
    def test_wait_and_resume(self):
        source = (Path(__file__).resolve().parents[1] / "app_led_support.c").read_text()
        function = source[source.index("void led_ui_fft_request("):]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_LOGW(...) ((void)0)
static uint32_t s_led_ui_fft_need_mask;
static bool s_led_ui_fft_owned, s_show_app_active, s_call_app_active;
static bool recording, playback, output, running, beat, bt;
static int starts, inits;
static bool show_runtime_owns_leds(void) { return false; }
static bool led_ui_fft_blocked_by_bluetooth(void) { return bt; }
static bool fft_visualizer_running(void) { return running; }
static void fft_set_display_enabled(bool b) { (void)b; }
static void fft_visualizer_stop(void) { running=false; }
static bool led_beat_enabled(void) { return beat; }
static bool led_audio_brightness_enabled(void) { return false; }
static bool mic_rec_is_recording(void) { return recording; }
static bool audio_player_is_active(void) { return playback; }
static bool audio_output_active(void) { return output; }
static bool shell_audio_init_if_needed(void) { ++inits; return true; }
static int fft_visualizer_start(void) { ++starts; running=true; return 0; }
static int64_t esp_timer_get_time(void) { return 0; }
FUNCTION
int main(void) {
    recording=true;
    for (int i=0;i<100;i++) led_ui_fft_request(1,true);
    assert(!starts && !inits && s_led_ui_fft_need_mask==1);
    recording=false; playback=true; led_ui_fft_request(0,false);
    assert(!starts && !inits);
    playback=false; output=true; led_ui_fft_request(0,false);
    assert(!starts && !inits);
    output=false; led_ui_fft_request(0,false);
    assert(starts==1 && inits==1 && running);
    led_ui_fft_request(1,false);
    assert(!running && !s_led_ui_fft_need_mask);
    beat=true; recording=true; led_ui_fft_request(0,false);
    assert(starts==1);
    recording=false; led_ui_fft_request(0,false);
    assert(starts==2 && running);
    bt=true; led_ui_fft_request(0,false);
    assert(!running && starts==2);
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
