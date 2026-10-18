#include "app_led_support.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fft.h"
#include "led.h"
#include "shell_audio.h"
#include "bt_audio.h"
#include "audio.h"
#include "audio_player.h"
#include "microphone.h"
#include "esp_heap_caps.h"
#include "show_runtime.h"

static const char *TAG = "led_ui_fft";
static uint32_t s_led_ui_fft_need_mask = 0;
static bool s_led_ui_fft_owned = false;
static bool s_bluetooth_app_active;
static bool s_show_app_active;
static bool s_paint_app_active;
static bool s_synth_app_active;
void led_ui_fft_set_synth_app(bool active)
{
    s_synth_app_active=active;
    if(active) led_ui_fft_request(0,false);
}
static bool s_paint_threedee_active;
static bool s_paint_armpose_active;
static volatile bool s_call_app_active;
void led_ui_fft_set_call_app(bool active) { s_call_app_active = active; }

void led_ui_fft_set_show_app(bool active)
{
    s_show_app_active = active;
    if (!active) return;
    fft_set_display_enabled(false);
    fft_visualizer_stop();
    s_led_ui_fft_owned = false;
    if (!mic_rec_is_recording() && !audio_player_is_active() && !audio_output_active())
        (void)audio_release_idle_bus();
}

void led_ui_fft_set_paint_app(bool active)
{
    s_paint_app_active = active;
    // Preserve pending clients; on exit the normal service tick resumes FFT.
    if (active) led_ui_fft_request(0, false);
}

void led_ui_fft_set_threedee_app(bool active)
{
    s_paint_threedee_active = active;
}

void led_ui_fft_set_armpose_app(bool active)
{
    s_paint_armpose_active = active;
}

bool led_ui_fft_blocked_by_bluetooth(void)
{
    bt_audio_status_t status = {0};
    bt_audio_get_status(&status);
    return s_bluetooth_app_active || status.state == BT_AUDIO_STATE_STARTING ||
        status.state == BT_AUDIO_STATE_DISCOVERING || status.state == BT_AUDIO_STATE_CONNECTING ||
        status.state == BT_AUDIO_STATE_CONNECTED || status.state == BT_AUDIO_STATE_STREAMING;
}

void led_ui_fft_set_bluetooth_app(bool active)
{
    s_bluetooth_app_active = active;
    if (active) {
        fft_set_display_enabled(false);
        fft_visualizer_stop();
        s_led_ui_fft_owned = false;
        // FFT tasks are gone; their I2S DMA buffers are still allocated until
        // explicitly released. Keep the bus if another audio user owns it.
        if (!mic_rec_is_recording() && !audio_player_is_active() && !audio_output_active()) {
            size_t before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            esp_err_t err = audio_release_idle_bus();
            size_t after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            ESP_LOGI(TAG, "BT I2S release: %s; internal heap %u -> %u",
                     esp_err_to_name(err), (unsigned)before, (unsigned)after);
        }
    }
    // On exit the ordinary service tick retries pending requests when BT is idle.
}

void led_ui_fft_request(uint32_t client_mask, bool needed)
{
    if (needed) {
        s_led_ui_fft_need_mask |= client_mask;
    } else {
        s_led_ui_fft_need_mask &= ~client_mask;
    }

    if (s_synth_app_active || s_paint_app_active || s_call_app_active || s_show_app_active || s_paint_threedee_active || s_paint_armpose_active || show_runtime_owns_leds() || led_ui_fft_blocked_by_bluetooth()) {
        if (fft_visualizer_running()) {
            fft_set_display_enabled(false);
            fft_visualizer_stop();
        }
        s_led_ui_fft_owned = false;
        return;
    }

    bool want_fft = (s_led_ui_fft_need_mask != 0) ||
                    led_beat_enabled() ||
                    led_audio_brightness_enabled();
    if (want_fft) {
        // Recording/playback owning I2S is expected, not a failed FFT start.
        // Keep requests pending; importantly, do not reinitialize/reclock the
        // audio bus while another consumer owns it. The service tick resumes
        // FFT when that consumer finishes.
        if (mic_rec_is_recording() || audio_player_is_active() || audio_output_active())
            return;
        if (!shell_audio_init_if_needed()) {
            ESP_LOGW(TAG, "audio init unavailable; FFT request pending");
            return;
        }

        bool was_running = fft_visualizer_running();
        if (!was_running) fft_set_display_enabled(false);
        esp_err_t err = fft_visualizer_start();
        if (err != ESP_OK) {
            static int64_t next_warning;
            if (esp_timer_get_time() >= next_warning) {
                ESP_LOGW(TAG, "FFT request pending (audio busy or unavailable): %s", esp_err_to_name(err));
                next_warning = esp_timer_get_time() + 5000000;
            }
            return;
        }
        if (!was_running) {
            s_led_ui_fft_owned = true;
        }
        return;
    }

    if (s_led_ui_fft_owned && fft_visualizer_running()) {
        fft_visualizer_stop();
    }
    s_led_ui_fft_owned = false;
}
