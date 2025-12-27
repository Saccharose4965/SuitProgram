// main/app_main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "hw.h"
#include "oled_spi.h"
#include "twenty48.h"

void app_main(void)
{
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init_fixed();
    ESP_ERROR_CHECK(hw_adc1_init_default());

    oled_init();
    oled_clear();

    t48_game_init();

    while (1) {
        t48_game_tick();
        vTaskDelay(pdMS_TO_TICKS(12));
    }
}

/*
// main/app_main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "audio.h"
#include "fft.h"
#include "hw.h"
#include "oled_spi.h"
#include "led.h"

static void beat_led_task(void *arg){
    (void)arg;
    fft_beat_event_t evt;
    while (1){
        if (fft_receive_beat(&evt, portMAX_DELAY)){
            sendpulse(0,255,0);
        }
    }
}

void app_main(void)
{
    // Bring up SPI + GPIO defaults
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init_fixed();
    ESP_ERROR_CHECK(hw_adc1_init_default());

    // Initialize OLED
    oled_init();
    oled_clear();

    // Configure shared I²S bus for mic RX (DIN) + amp TX (DOUT)
    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = PIN_I2S_DIN,
        .sd_en_pin = -1,
    };
    ESP_ERROR_CHECK(audio_init(&bus));

    // Start FFT visualizer (mic → FFT → OLED bars)
    ESP_ERROR_CHECK(fft_visualizer_start());
    xTaskCreate(beat_led_task, "beat_led", 2048, NULL, 4, NULL);

    // Cycle through views on button A press (single-step on rising edge)
    fft_view_t view = FFT_VIEW_BPM_TEXT;
    hw_button_id_t last_btn = HW_BTN_NONE;

    while (1) {
        hw_button_id_t b = HW_BTN_NONE;
        if (hw_buttons_read(&b) == ESP_OK) {
            if (b == HW_BTN_A && last_btn != HW_BTN_A) {
                view = (fft_view_t)((view + 1) % (FFT_VIEW_PHASE_COMB + 1));
                fft_visualizer_set_view(view);
                ESP_LOGI("app", "View -> %d", (int)view);
            }
            last_btn = b;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}*/
/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "hw.h"
#include "storage_sd.h"
#include "audio.h"

static const char* TAG = "app_main";

void app_main(void)
{
    // Bring up shared SPI bus (OLED/IMUs/SD share it)
    ESP_ERROR_CHECK(hw_spi2_init_once());

    // Mount SD and bump counter at the actual mount point
    ESP_ERROR_CHECK(storage_mount_sd());
    ESP_ERROR_CHECK_WITHOUT_ABORT(storage_sd_increment_counter("boot_count.txt"));

    // Optional: list root so we see files present
    (void)storage_sd_list_dir(NULL);
    // Probe a subfolder but don't treat it as fatal if missing
    if (storage_sd_list_dir("subfolder") != ESP_OK) {
        ESP_LOGW(TAG, "No %s/subfolder (that's fine)", storage_sd_mount_point());
    }

    // ---- AUDIO INIT (REQUIRED) ----
    const audio_i2s_config_t acfg = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .sd_en_pin = -1,           // set to your amp enable GPIO if you have one
    };
    ESP_ERROR_CHECK(audio_init(&acfg));   // creates/enables I2S TX

    // Quick sanity beeps (200 ms each @ 48 kHz)
    (void)audio_play_tone(440,  200);
    (void)audio_play_tone(880,  200);
    (void)audio_play_tone(1320, 200);

    // Play /<mount>/test.wav (PCM 8/16-bit, mono/stereo; I2S rate auto-matches file)
    char wav_path[128];
    ESP_ERROR_CHECK(storage_sd_make_path(wav_path, sizeof(wav_path), "test.wav"));
    esp_err_t e = audio_play_wav_file(wav_path);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "WAV playback failed from %s: %s", wav_path, esp_err_to_name(e));
    }

    // Idle
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




void app_main(void)
{
    // Bring up SPI bus and keep other devices off the line
    ESP_ERROR_CHECK(hw_spi2_init_once());
    // Mount SD
    ESP_ERROR_CHECK(storage_mount_sd());
    ESP_LOGI(TAG, "SD mounted at: %s", storage_sd_mount_point());

    // Counter file lives on SD root (no folders)
    ESP_ERROR_CHECK_WITHOUT_ABORT(storage_sd_increment_counter("boot_count.txt"));

    // Audio init (MAX98357A)
    const audio_i2s_config_t acfg = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .sd_en_pin = -1,   // set your amp enable pin if you add one
    };
    ESP_ERROR_CHECK(audio_init(&acfg));

    // Beeps (root-level sanity check)
    audio_play_tone(440,  1000);
    audio_play_tone(880,  1000);
    audio_play_tone(220, 1000);

    // Build absolute path to a root-level file "test.wav" (no subfolders)
    char wav_path[128];
    ESP_ERROR_CHECK(storage_sd_make_path(wav_path, sizeof(wav_path), "test.wav"));

    // Optional: confirm file exists before playing (root file drop case)
    FILE* f = fopen(wav_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "File not found on SD root: %s", wav_path);
    } else {
        fclose(f);
        esp_err_t e = audio_play_wav_file(wav_path);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "WAV playback failed from %s: %s", wav_path, esp_err_to_name(e));
        } else {
            ESP_LOGI(TAG, "WAV playback completed");
        }
    }

    // Idle forever
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
*/

/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw.h"
#include "logo.h"
#include "mpu6500.h"
#include "telemetry.h"

void app_main(void){
    hw_spi2_init_once();

    // Play once; comment out while debugging if you want faster boot
    anim_logo();
}
    // --- IMU on SPI ---
    spi_device_handle_t imu_dev;
    if (hw_spi2_add_device(PIN_IMU_CS, &imu_dev) != ESP_OK) {
        // stay alive; don't reboot
        for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    mpu6500_config_t imu_cfg = { .spi = imu_dev };
    if (mpu6500_init(&imu_cfg) != ESP_OK) {
        for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    mpu6500_start_sampler(100); // ignore failure; we'll just not send

    // --- Wi-Fi + UDP to your PC's **LAN** IP ---
    telemetry_cfg_t tcfg = {
        .ssid   = "TERRIER_24",
        .pass   = "L@P3st3#",
        .pc_ip  = "192.168.1.255", // <-- change to your PC’s IP
        .pc_port= 9000
    };
    while (telemetry_start(&tcfg) != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(2000)); }

    while (1) {
        mpu6500_sample_t s = mpu6500_latest();
        telemetry_send_mpu_sample(s.ax_g, s.ay_g, s.az_g,
                                  s.gx_dps, s.gy_dps, s.gz_dps,
                                  s.temp_c, s.t_us);
        vTaskDelay(pdMS_TO_TICKS(50)); // ~20 Hz
    }
}*//*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry.h"

void app_main(void){
    // Use your PC's LAN IP here
    telemetry_cfg_t tcfg = {
        .ssid    = "TERRIER_24",
        .pass    = "L@P3st3#",
        .pc_ip   = "192.168.0.205",   // <= YOUR PC'S LAN IP
        .pc_port = 9000
    };

    while (telemetry_start(&tcfg) != ESP_OK) {
        printf("net: start failed, retrying...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    printf("net: started\n");

    while (1) {
        telemetry_send_line("hello\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*//*
#include "audio.h"

void app_main(void) {
    audio_i2s_config_t cfg = {
        .i2s_id = 0,
        .bclk_gpio = 4,
        .lrck_gpio = 22,
        .dout_gpio = 25,
        .sd_en_pin = -1   // or your amp's shutdown pin, if you wired it
    };
    audio_init(&cfg);
    audio_play_tone(440, 1000); // 1s A4 beep
    audio_play_tone(880, 1000); // 1s A5 beep
    audio_play_tone(1200, 1000); // 1s X5 beep
    audio_play_embedded();
}
*/
