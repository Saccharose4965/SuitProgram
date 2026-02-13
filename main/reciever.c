// Receiver-only firmware: listen for UDP PCM packets and play them on the amp
// Set WIFI_SSID/PASS and PEER_IP before flashing.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "audio.h"
#include "hw.h"
#include "telemetry.h"
#include "oled_spi.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "receiver";
#define SAMPLE_RATE_HZ 16000
#define CALL_PORT      9100

typedef struct {
    uint16_t port;
} rx_cfg_t;

static void call_rx_task(void *arg){
    rx_cfg_t cfg = *(rx_cfg_t*)arg;
    free(arg);

    i2s_chan_handle_t tx = audio_tx_handle();
    if (!tx) {
        ESP_LOGE(TAG, "no TX handle");
        vTaskDelete(NULL);
    }
    (void)audio_enable_tx();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cfg.port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(sock);
        vTaskDelete(NULL);
    }

    const size_t FRAMES = 512; // max mono frames per packet we will accept
    uint8_t  *pkt   = (uint8_t*)heap_caps_malloc(1500, MALLOC_CAP_8BIT);
    int32_t  *out32 = (int32_t*)heap_caps_malloc(FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
    if (!pkt || !out32) {
        ESP_LOGE(TAG, "OOM rx buffers");
        free(pkt); free(out32);
        close(sock);
        vTaskDelete(NULL);
    }

    while (1) {
        ssize_t n = recv(sock, pkt, 1500, 0);
        if (n < 14) continue;
        if (memcmp(pkt, "CALL", 4) != 0) continue;

        uint16_t nb = 0;
        memcpy(&nb, pkt + 8, 2);
        if ((size_t)n < 12 + nb) continue;

        size_t samples = nb / sizeof(int16_t);
        if (samples > FRAMES) samples = FRAMES;

        for (size_t i = 0; i < samples; ++i) {
            int16_t v16;
            memcpy(&v16, pkt + 12 + i * sizeof(int16_t), sizeof(int16_t));
            int32_t v32 = ((int32_t)v16) << 16;
            out32[2 * i]     = v32;
            out32[2 * i + 1] = v32;
        }

        size_t bytes = samples * 2 * sizeof(int32_t);
        size_t written = 0;
        esp_err_t er = i2s_channel_write(tx, out32, bytes, &written, 30);
        if (er != ESP_OK) {
            ESP_LOGW(TAG, "i2s write err %d", er);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    // not reached
    free(pkt);
    free(out32);
    close(sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // --- Configure your network + peer ---
    static const char *WIFI_SSID = "TERRIER_24";
    static const char *WIFI_PASS = "L@P3st3#";
    static const char *PEER_IP   = "192.168.1.255"; // set to sender ESP IP (used for hello)
    static const uint16_t PEER_PORT = 9000;          // telemetry hello port

    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init();
    ESP_ERROR_CHECK(hw_adc1_init_default());

    oled_init();
    oled_clear();

    // I2S bus for amp playback only
    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = -1,         // no mic on receiver
        .sd_en_pin = -1,
    };
    ESP_ERROR_CHECK(audio_init(&bus));
    ESP_ERROR_CHECK(audio_set_rate(SAMPLE_RATE_HZ));
    ESP_ERROR_CHECK(audio_enable_tx());

    // Bring up Wi-Fi/UDP (uses telemetry helper)
    telemetry_cfg_t net = {
        .ssid    = WIFI_SSID,
        .pass    = WIFI_PASS,
        .pc_ip   = PEER_IP,   // reuse peer IP for the hello packet
        .pc_port = PEER_PORT,
    };
    ESP_ERROR_CHECK(telemetry_start(&net));

    // Launch UDP -> speaker receiver
    rx_cfg_t *cfg = (rx_cfg_t*)malloc(sizeof(rx_cfg_t));
    if (cfg) {
        cfg->port = CALL_PORT;
        xTaskCreatePinnedToCore(call_rx_task, "call_rx", 4096, cfg, 5, NULL, tskNO_AFFINITY);
    } else {
        ESP_LOGE(TAG, "alloc rx cfg failed");
    }

    char l1[32], l2[32], l3[32];
    snprintf(l1, sizeof(l1), "CALL: RECEIVER");
    snprintf(l2, sizeof(l2), "port %u", CALL_PORT);
    snprintf(l3, sizeof(l3), "fs=%d Hz", SAMPLE_RATE_HZ);
    oled_render_three_lines(l1, l2, l3);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
