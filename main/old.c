// main/app_main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "audio.h"
#include "fft.h"
#include "hw.h"
#include "oled_spi.h"

void app_main(void)
{
    // Bring up SPI + GPIO defaults
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init();

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

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/*// main/app_main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "storage_sd.h"
#include "audio.h"
#include "oled_spi.h"
#include "microphone.h"
#include "hw.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"

#include "hw.h"
#include "oled_spi.h"
#include "flappy.h"

void app_main(void)
{
    // Bring up SPI + GPIO defaults
    hw_spi2_init_once();
    hw_gpio_init();

    // Initialize OLED
    oled_init();
    oled_clear();

    // Run the Flappy minigame (blocking)
    flappy_run();

    // After exiting game
    oled_clear();
    ESP_LOGI("MAIN", "Flappy exited, main continues…");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/
/*
void app_main(void)
{
    storage_mount_sd();
    hw_spi2_init_once(); // for SD/OLED if needed
    oled_init();               // Your existing OLED init

    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = PIN_I2S_DIN,
        .sd_en_pin = -1,         // set to your AMP EN pin if wired (e.g., 27)
    };
    ESP_ERROR_CHECK(audio_init(&bus));

    mic_rec_cfg_t mic = { .sample_rate_hz = 16000 };
    ESP_ERROR_CHECK(mic_rec_init(&mic));

    oled_render_three_lines("AUDIO INPUT", "sample_rate_hz = 16000", "RECORDING...");
    // Record 6 seconds
    ESP_ERROR_CHECK(mic_rec_start(NULL)); // auto path /sdcard/rec_XXXXX.wav
    vTaskDelay(pdMS_TO_TICKS(6000));
    ESP_ERROR_CHECK(mic_rec_stop());
    oled_render_three_lines("AUDIO INPUT", "sample_rate_hz = 16000", "STOPPED...");

    ESP_ERROR_CHECK(audio_set_rate(16000));   // same family as your recordings
    // Play it back
    const char *last = mic_rec_last_path();
    if (last) {
        ESP_LOGI("MAIN", "Playback: %s", last);
        esp_err_t e = audio_play_wav_file(last);
        if (e != ESP_OK) {
            ESP_LOGE("MAIN", "Playback error: %s", esp_err_to_name(e));
            // optional: show on OLED, etc.
        }

    } else {
        ESP_LOGE("MAIN", "No last recording path.");
    }
}*/

/*
// main/app_main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "storage_sd.h"
#include "audio.h"
#include "oled_spi.h"
#include "microphone.h"
#include "hw.h"

void app_main(void)
{
    ESP_ERROR_CHECK(hw_spi2_init_once());       // SPI for OLED/SD
    storage_mount_sd();
    oled_init();               // Your existing OLED init

    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .sd_en_pin = -1,
        .din_gpio  = PIN_I2S_DIN,
    };
    audio_init(&bus);

    mic_rec_cfg_t mic = { .sample_rate_hz = 16000 };
    oled_render_three_lines("AUDIO INPUT", "sample_rate_hz = 16000", "RECORDING...");
    mic_rec_init(&mic);

    // Record 10s
    mic_rec_start(NULL);
    vTaskDelay(pdMS_TO_TICKS(10000));
    oled_render_three_lines("AUDIO INPUT", "sample_rate_hz = 16000", "STOPPED...");
    mic_rec_stop();

    vTaskDelay(pdMS_TO_TICKS(500));

    // Play it back
    const char *last = mic_rec_last_path();
    //if (last) audio_play_wav_file(last);
}
*/

/*
#include "hw.h"
#include "logo.h"
#include "oled_spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    // Init hardware we need
    ESP_ERROR_CHECK(hw_spi2_init_once());       // SPI for OLED/SD
    oled_init();               // Your existing OLED init
    anim_logo();
    ESP_ERROR_CHECK(hw_adc1_init_default());    // ADC for buttons ladder

    // First splash
    oled_render_three_lines("Buttons ADC", "GPIO39 (VN)", "Reading...");

    char l1[32], l2[32], l3[32];

    while (1) {
        int raw = 0;
        int mv  = 0;

        esp_err_t er = hw_adc1_read_raw(HW_BUTTONS_ADC_CH, &raw);
        esp_err_t ev = hw_adc1_read_mv(HW_BUTTONS_ADC_CH, &mv);

        if (er == ESP_OK && ev == ESP_OK) {
            // Line 1: short title
            snprintf(l1, sizeof(l1), "BTN @ GPIO39");

            // Line 2: raw code
            // 4 digits width keeps the text steady on screen
            snprintf(l2, sizeof(l2), "RAW: %4d", raw);

            // Line 3: calibrated millivolts
            snprintf(l3, sizeof(l3), "MV : %4d", mv);
        } else {
            snprintf(l1, sizeof(l1), "Read error");
            snprintf(l2, sizeof(l2), "raw:%d mv:%d", (int)er, (int)ev);
            snprintf(l3, sizeof(l3), " ");
            ESP_LOGE(TAG, "ADC read failed er=%d ev=%d", er, ev);
        }

        oled_render_three_lines(l1, l2, l3);

        vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz updates
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

