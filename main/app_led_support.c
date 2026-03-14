#include "app_led_support.h"

#include "esp_err.h"
#include "esp_log.h"
#include "fft.h"
#include "led.h"
#include "shell_audio.h"

static const char *TAG = "led_ui_fft";
static uint32_t s_led_ui_fft_need_mask = 0;
static bool s_led_ui_fft_owned = false;

void led_ui_fft_request(uint32_t client_mask, bool needed)
{
    if (needed) {
        s_led_ui_fft_need_mask |= client_mask;
    } else {
        s_led_ui_fft_need_mask &= ~client_mask;
    }

    bool want_fft = (s_led_ui_fft_need_mask != 0) ||
                    led_beat_enabled() ||
                    led_audio_brightness_enabled();
    if (want_fft) {
        if (!shell_audio_init_if_needed()) {
            ESP_LOGW(TAG, "audio init unavailable; FFT request pending");
            return;
        }

        bool was_running = fft_visualizer_running();
        fft_set_display_enabled(false);
        esp_err_t err = fft_visualizer_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "fft_visualizer_start failed: %s", esp_err_to_name(err));
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
