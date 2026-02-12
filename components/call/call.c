#include "call.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"

#include "audio.h"
#include "hw.h"
#include "telemetry.h"
#include "oled.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "call";
#define SAMPLE_RATE_HZ 16000
#define CALL_FRAMES_PER_PKT 160
#define CALL_PLAY_CHUNK     CALL_FRAMES_PER_PKT
#define CALL_RING_SAMPLES   65536 // mono int16 samples in ring buffer

#if configNUMBER_OF_CORES > 1
#define BG_TASK_CORE 0
#else
#define BG_TASK_CORE 0
#endif

typedef struct {
    uint32_t pkts;
    uint32_t last_seq;
    uint16_t last_nb;
    uint32_t write_errs;
    uint32_t short_writes;
    uint32_t drops;
    uint32_t underruns;
    uint32_t overruns;
} rx_stats_t;

static rx_stats_t g_rx_stats = {0};

typedef struct {
    uint16_t port;
} rx_cfg_t;

// -------- Ring buffer for RX PCM (mono int16) --------
static int16_t g_ring[CALL_RING_SAMPLES];
static size_t  g_ring_head = 0, g_ring_tail = 0, g_ring_count = 0;
static portMUX_TYPE g_ring_mux = portMUX_INITIALIZER_UNLOCKED;

static size_t ring_push(const int16_t *src, size_t n){
    size_t pushed = 0;
    portENTER_CRITICAL(&g_ring_mux);
    size_t space = CALL_RING_SAMPLES - g_ring_count;
    if (n > space) {
        size_t to_drop = n - space;
        if (to_drop > g_ring_count) to_drop = g_ring_count;
        g_ring_tail = (g_ring_tail + to_drop) % CALL_RING_SAMPLES;
        g_ring_count -= to_drop;
        g_rx_stats.overruns++;
    }
    for (size_t i = 0; i < n; ++i) {
        g_ring[g_ring_head] = src[i];
        g_ring_head = (g_ring_head + 1) % CALL_RING_SAMPLES;
    }
    g_ring_count += n;
    pushed = n;
    portEXIT_CRITICAL(&g_ring_mux);
    return pushed;
}

static size_t ring_pop(int16_t *dst, size_t n){
    size_t got = 0;
    portENTER_CRITICAL(&g_ring_mux);
    size_t avail = g_ring_count;
    size_t take = (n < avail) ? n : avail;
    for (size_t i = 0; i < take; ++i) {
        dst[i] = g_ring[g_ring_tail];
        g_ring_tail = (g_ring_tail + 1) % CALL_RING_SAMPLES;
    }
    g_ring_count -= take;
    got = take;
    portEXIT_CRITICAL(&g_ring_mux);
    return got;
}

static void call_rx_play_task(void *arg){
    (void)arg;
    const size_t CHUNK = CALL_PLAY_CHUNK;
    int16_t mono[CALL_PLAY_CHUNK];
    int32_t stereo[CALL_PLAY_CHUNK * 2];

    i2s_chan_handle_t tx = audio_tx_handle();
    if (!tx || audio_enable_tx() != ESP_OK) {
        ESP_LOGE(TAG, "play: no TX");
        vTaskDelete(NULL);
    }

    const size_t WARMUP_SAMPLES = (SAMPLE_RATE_HZ / 1000) * 300; // ~300 ms
    while (1) {
        size_t count;
        portENTER_CRITICAL(&g_ring_mux);
        count = g_ring_count;
        portEXIT_CRITICAL(&g_ring_mux);
        if (count >= WARMUP_SAMPLES) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1) {
        size_t got = ring_pop(mono, CHUNK);
        if (got < CHUNK) {
            memset(mono + got, 0, (CHUNK - got) * sizeof(int16_t));
            g_rx_stats.underruns++;
        }
        for (size_t i = 0; i < CHUNK; ++i) {
            int32_t v32 = ((int32_t)mono[i]) << 16;
            stereo[2 * i]     = v32;
            stereo[2 * i + 1] = v32;
        }
        size_t bytes = CHUNK * 2 * sizeof(int32_t);
        size_t written = 0;
        esp_err_t er = i2s_channel_write(tx, stereo, bytes, &written, pdMS_TO_TICKS(400));
        if (er != ESP_OK) {
            ESP_LOGW(TAG, "i2s write err %d", er);
            g_rx_stats.write_errs++;
            vTaskDelay(pdMS_TO_TICKS(5));
        } else if (written != bytes) {
            g_rx_stats.short_writes++;
        }
    }
}

static void call_rx_task(void *arg){
    rx_cfg_t cfg = *(rx_cfg_t*)arg;
    free(arg);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
    }
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int bcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    int rcvbuf = 32768;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cfg.port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(sock);
        vTaskDelete(NULL);
    }

    const size_t FRAMES = CALL_FRAMES_PER_PKT;
    uint8_t  *pkt   = (uint8_t*)heap_caps_malloc(1500, MALLOC_CAP_8BIT);
    int16_t  *mono  = (int16_t*)heap_caps_malloc(FRAMES * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!pkt || !mono) {
        ESP_LOGE(TAG, "OOM rx buffers");
        free(pkt); free(mono);
        close(sock);
        vTaskDelete(NULL);
    }

    uint32_t pkt_count = 0;
    uint32_t prev_seq = 0;
    bool     have_seq = false;
    while (1) {
        ssize_t n = recv(sock, pkt, 1500, 0);
        if (n < 14) continue;
        if (memcmp(pkt, "CALL", 4) != 0) continue;

        uint32_t seq = 0;
        memcpy(&seq, pkt + 4, 4);
        uint16_t nb = 0;
        memcpy(&nb, pkt + 8, 2);
        if ((size_t)n < 12 + nb) continue;

        if (have_seq && seq != prev_seq + 1) {
            g_rx_stats.drops++;
        }
        prev_seq = seq;
        have_seq = true;

        size_t samples = nb / sizeof(int16_t);
        if (samples > FRAMES) samples = FRAMES;
        memcpy(mono, pkt + 12, samples * sizeof(int16_t));
        size_t pushed = ring_push(mono, samples);
        if (pushed != samples) {
            g_rx_stats.overruns++;
        }

        g_rx_stats.pkts      = ++pkt_count;
        g_rx_stats.last_seq  = seq;
        g_rx_stats.last_nb   = nb;

        if ((pkt_count % 100) == 0) {
            ESP_LOGI(TAG, "rx %u pkts, last %d bytes", (unsigned)pkt_count, (int)n);
        }
    }
}

// ---------------- Sender ----------------

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
    int bcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port   = htons(cfg.port);
    inet_aton(cfg.peer_ip, &peer.sin_addr);

    const size_t FRAMES = CALL_FRAMES_PER_PKT; // mono frames per packet
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
    uint32_t send_errs = 0;
    for (;;) {
        size_t nread = 0;
        esp_err_t er = i2s_channel_read(rx, buf32,
                                        FRAMES * 2 * sizeof(int32_t),
                                        &nread, pdMS_TO_TICKS(200));
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
        ssize_t sent = sendto(sock, pkt, 12 + payload, 0, (struct sockaddr*)&peer, sizeof(peer));
        if (sent < 0 && (++send_errs % 8) == 0) {
            ESP_LOGW(TAG, "sendto fail errno=%d", errno);
        }
        seq++;
    }
}

// ---------------- Public entry ----------------

void call_start(const call_cfg_t *cfg){
    if (!cfg || !cfg->ssid || !cfg->pass || !cfg->peer_ip || cfg->peer_port == 0 || cfg->call_port == 0) return;

    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init();
    ESP_ERROR_CHECK(hw_adc1_init_default());

    oled_init();
    oled_clear();

    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = (cfg->role == CALL_ROLE_TX) ? PIN_I2S_DIN : -1,
        .sd_en_pin = -1,
    };
    ESP_ERROR_CHECK(audio_init(&bus));
    ESP_ERROR_CHECK(audio_set_rate(SAMPLE_RATE_HZ));
    if (cfg->role == CALL_ROLE_RX) {
        ESP_ERROR_CHECK(audio_enable_tx());
        (void)audio_play_tone(1000, 250);
    } else {
        ESP_ERROR_CHECK(audio_enable_rx());
    }

    telemetry_cfg_t net = {
        .ssid    = cfg->ssid,
        .pass    = cfg->pass,
        .pc_ip   = cfg->peer_ip,
        .pc_port = cfg->peer_port,
    };
    ESP_ERROR_CHECK(telemetry_start(&net));
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    if (cfg->role == CALL_ROLE_RX) {
        rx_cfg_t *rcfg = (rx_cfg_t*)malloc(sizeof(rx_cfg_t));
        if (rcfg) {
            rcfg->port = cfg->call_port;
            xTaskCreatePinnedToCore(call_rx_task, "call_rx", 4096, rcfg, 5, NULL, BG_TASK_CORE);
            xTaskCreatePinnedToCore(call_rx_play_task, "call_play", 4096, NULL, 6, NULL, BG_TASK_CORE);
        } else {
            ESP_LOGE(TAG, "alloc rx cfg failed");
        }

        for (;;) {
            rx_stats_t s = g_rx_stats;
            char l1[32], l2[32], l3[32];
            snprintf(l1, sizeof(l1), "CALL RX fs=%d", SAMPLE_RATE_HZ);
            snprintf(l2, sizeof(l2), "pk:%lu d:%lu o:%lu", (unsigned long)s.pkts, (unsigned long)s.drops, (unsigned long)s.overruns);
            snprintf(l3, sizeof(l3), "nb:%u e:%lu u:%lu", s.last_nb, (unsigned long)s.write_errs, (unsigned long)s.underruns);
            oled_render_three_lines(l1, l2, l3);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } else {
        tx_cfg_t *tcfg = (tx_cfg_t*)malloc(sizeof(tx_cfg_t));
        if (tcfg) {
            strncpy(tcfg->peer_ip, cfg->peer_ip, sizeof(tcfg->peer_ip) - 1);
            tcfg->peer_ip[sizeof(tcfg->peer_ip) - 1] = 0;
            tcfg->port = cfg->call_port;
            xTaskCreatePinnedToCore(mic_tx_task, "mic_tx", 4096, tcfg, 5, NULL, BG_TASK_CORE);
        } else {
            ESP_LOGE(TAG, "alloc tx cfg failed");
        }

        char l1[32], l2[32], l3[32];
        snprintf(l1, sizeof(l1), "CALL: SENDER");
        snprintf(l2, sizeof(l2), "-> %s:%u", cfg->peer_ip, cfg->call_port);
        snprintf(l3, sizeof(l3), "fs=%d Hz", SAMPLE_RATE_HZ);
        oled_render_three_lines(l1, l2, l3);

        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
