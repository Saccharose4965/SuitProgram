// Sender-only firmware: capture mic and stream PCM over UDP to a peer
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

static const char *TAG = "sender";
#define SAMPLE_RATE_HZ 16000
#define CALL_PORT      9100

typedef struct {
    char     peer_ip[32];
    uint16_t port;
} tx_cfg_t;

static void mic_tx_task(void *arg){
    tx_cfg_t cfg = *(tx_cfg_t*)arg;
    free(arg);

    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx) {
        ESP_LOGE(TAG, "no RX handle");
        vTaskDelete(NULL);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
    }

    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port   = htons(cfg.port);
    inet_aton(cfg.peer_ip, &peer.sin_addr);

    const size_t FRAMES = 256; // mono frames per packet
    const size_t PAYLOAD_MAX = FRAMES * sizeof(int16_t);
    uint8_t  *pkt   = (uint8_t*)heap_caps_malloc(PAYLOAD_MAX + 12, MALLOC_CAP_8BIT);
    int32_t  *buf32 = (int32_t*)heap_caps_malloc(FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
    if (!pkt || !buf32) {
        ESP_LOGE(TAG, "OOM buffers");
        free(pkt); free(buf32);
        vTaskDelete(NULL);
    }

    uint32_t seq = 0;
    int err_count = 0;
    for (;;) {
        size_t nread = 0;
        esp_err_t er = i2s_channel_read(rx, buf32,
                                        FRAMES * 2 * sizeof(int32_t),
                                        &nread, 200);
        if (er != ESP_OK) {
            if (++err_count % 8 == 0) ESP_LOGW(TAG, "mic read err %d", er);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        err_count = 0;

        size_t frames  = nread / (sizeof(int32_t) * 2);
        size_t payload = frames * sizeof(int16_t);
        if (payload > PAYLOAD_MAX) payload = PAYLOAD_MAX;
        size_t samples = payload / sizeof(int16_t);

        for (size_t i = 0; i < samples; ++i) {
            int16_t v = (int16_t)(buf32[2 * i] >> 14); // use left mic channel
            memcpy(pkt + 12 + i * sizeof(int16_t), &v, sizeof(int16_t));
        }

        uint16_t nb = (uint16_t)payload;
        memcpy(pkt + 0,  "CALL", 4);
        memcpy(pkt + 4,  &seq, 4);
        memcpy(pkt + 8,  &nb,  2);
        sendto(sock, pkt, 12 + payload, 0, (struct sockaddr*)&peer, sizeof(peer));
        seq++;
    }
}

void app_main(void)
{
    // --- Configure your network + peer ---
    static const char *WIFI_SSID = "TERRIER_24";
    static const char *WIFI_PASS = "L@P3st3#";
    static const char *PEER_IP   = "192.168.1.255"; // set to receiver ESP IP
    static const uint16_t PEER_PORT = 9000;          // telemetry hello port

    // Base hardware bring-up
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init();
    ESP_ERROR_CHECK(hw_adc1_init_default());

    oled_init();
    oled_clear();

    // I2S bus for mic capture only
    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = PIN_I2S_DIN,
        .sd_en_pin = -1,
    };
    ESP_ERROR_CHECK(audio_init(&bus));
    ESP_ERROR_CHECK(audio_set_rate(SAMPLE_RATE_HZ));
    ESP_ERROR_CHECK(audio_enable_rx());

    // Bring up Wi-Fi/UDP (uses telemetry helper)
    telemetry_cfg_t net = {
        .ssid    = WIFI_SSID,
        .pass    = WIFI_PASS,
        .pc_ip   = PEER_IP,   // we reuse peer IP for the hello packet
        .pc_port = PEER_PORT,
    };
    ESP_ERROR_CHECK(telemetry_start(&net));

    // Launch mic -> UDP sender
    tx_cfg_t *cfg = (tx_cfg_t*)malloc(sizeof(tx_cfg_t));
    if (cfg) {
        strncpy(cfg->peer_ip, PEER_IP, sizeof(cfg->peer_ip) - 1);
        cfg->peer_ip[sizeof(cfg->peer_ip) - 1] = '';
        cfg->port = CALL_PORT;
        xTaskCreatePinnedToCore(mic_tx_task, "mic_tx", 4096, cfg, 5, NULL, tskNO_AFFINITY);
    } else {
        ESP_LOGE(TAG, "alloc tx cfg failed");
    }

    char l1[32], l2[32], l3[32];
    snprintf(l1, sizeof(l1), "CALL: SENDER");
    snprintf(l2, sizeof(l2), "-> %s:%u", PEER_IP, CALL_PORT);
    snprintf(l3, sizeof(l3), "fs=%d Hz", SAMPLE_RATE_HZ);
    oled_render_three_lines(l1, l2, l3);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
