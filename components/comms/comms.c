#include "comms.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "audio.h"
#include "hw.h"
#include "microphone.h"
#include "telemetry.h"
#include "storage_sd.h"
#include "oled.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "comms";

#if configNUMBER_OF_CORES > 1
#define BG_TASK_CORE 0
#else
#define BG_TASK_CORE 0
#endif

typedef struct {
    char     peer_ip[32];
    uint16_t port;
    bool     save_local;
} call_cfg_t;

// ---------------- Call TX ----------------
static void call_sender_task(void *arg){
    call_cfg_t cfg = *(call_cfg_t*)arg;
    free(arg);

    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx) { ESP_LOGE(TAG, "call_tx: no RX handle"); vTaskDelete(NULL); }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelete(NULL); }
    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port   = htons(cfg.port);
    inet_aton(cfg.peer_ip, &peer.sin_addr);

    const size_t FRAMES = 256;
    const size_t PAYLOAD_MAX = FRAMES * sizeof(int16_t);
    const size_t PKT_MAX = 12 + PAYLOAD_MAX;
    int32_t *buf32 = (int32_t*)heap_caps_malloc(FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
    uint8_t *pkt   = (uint8_t*)heap_caps_malloc(PKT_MAX, MALLOC_CAP_8BIT);
    if (!buf32 || !pkt) { free(buf32); free(pkt); vTaskDelete(NULL); }
    FILE *f = NULL;
    if (cfg.save_local) {
        f = fopen("/sdcard/call_tx.pcm", "wb");
        if (!f) cfg.save_local = false;
    }

    uint32_t seq = 0;
    int slot = 0; bool decided = false;
    int err_count = 0;

    while (1){
        size_t nread = 0;
        esp_err_t er = i2s_channel_read(rx, buf32, sizeof(buf32), &nread, pdMS_TO_TICKS(200));
        if (er != ESP_OK) {
            if (++err_count > 5) { ESP_LOGE(TAG, "call_tx: RX read failing, stopping"); break; }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        err_count = 0;

        size_t frames = nread / (sizeof(int32_t)*2);
        if (!decided && frames >= 8){
            int64_t sumL=0,sumR=0;
            for (size_t i=0;i<frames;i++){ sumL += llabs((long long)buf32[2*i]); sumR += llabs((long long)buf32[2*i+1]); }
            slot = (sumR > sumL) ? 1 : 0;
            decided = true;
        }

        uint16_t nbytes = (uint16_t)(frames * sizeof(int16_t));
        if (nbytes > PAYLOAD_MAX) nbytes = (uint16_t)PAYLOAD_MAX;
        for (size_t i=0;i<frames && (i*sizeof(int16_t))<nbytes; i++){
            int32_t v32 = buf32[2*i + slot];
            int16_t v16 = (int16_t)(v32 >> 14);
            memcpy(pkt + 12 + i*sizeof(int16_t), &v16, sizeof(int16_t));
        }
        memcpy(pkt+0, "CALL", 4);
        memcpy(pkt+4, &seq, 4);
        memcpy(pkt+8, &nbytes, 2);
        sendto(sock, pkt, 12 + nbytes, 0, (struct sockaddr*)&peer, sizeof(peer));
        if (cfg.save_local && f) fwrite(pkt + 12, 1, nbytes, f);
        seq++;
    }
    if (f) fclose(f);
    free(buf32);
    free(pkt);
    vTaskDelete(NULL);
}

// ---------------- Call RX ----------------
static void call_receiver_task(void *arg){
    call_cfg_t cfg = *(call_cfg_t*)arg;
    free(arg);

    bool can_save = false; // per spec, we do not save RX side

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelete(NULL); }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        close(sock);
        vTaskDelete(NULL);
    }

    uint8_t pkt[1500];
    static int32_t out32[512*2];
    while (1){
        ssize_t n = recv(sock, pkt, sizeof(pkt), 0);
        if (n < 14) continue;
        if (memcmp(pkt, "CALL", 4) != 0) continue;
        uint16_t sz = 0;
        memcpy(&sz, pkt + 8, 2);
        if (12 + sz > (size_t)n) continue;
        size_t samples = sz / sizeof(int16_t);
        if (samples > 512) samples = 512;
        for (size_t i=0;i<samples;i++){
            int16_t v16;
            memcpy(&v16, pkt + 12 + i*sizeof(int16_t), sizeof(int16_t));
            int32_t v32 = ((int32_t)v16) << 16;
            out32[2*i]   = v32;
            out32[2*i+1] = v32;
        }
        size_t bytes = samples * 2 * sizeof(int32_t);
        size_t written = 0;
        i2s_channel_write(audio_tx_handle(), out32, bytes, &written, pdMS_TO_TICKS(20));
        (void)can_save;
    }
    vTaskDelete(NULL);
}

// ---------------- Message send (button) ----------------
static void send_last_recording(const telemetry_cfg_t *tel){
    const char *path = mic_rec_last_path();
    if (!path) {
        oled_render_three_lines("No recording", "Nothing to send", "");
        return;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        oled_render_three_lines("Cannot open", path, "");
        return;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 200000) {
        fclose(f);
        oled_render_three_lines("File too big", path, "");
        return;
    }
    uint8_t *buf = (uint8_t*)malloc(sz);
    if (!buf) {
        fclose(f);
        oled_render_three_lines("OOM buffer", "", "");
        return;
    }
    fread(buf, 1, sz, f);
    fclose(f);
    telemetry_send_buffer_udp(buf, (size_t)sz, tel->pc_ip, tel->pc_port, NULL, NULL);
    oled_render_three_lines("Send done", path, "");
    free(buf);
}

// ---------------- Control task ----------------
typedef struct {
    telemetry_cfg_t tel;
    uint16_t call_port;
    bool has_mic;
    bool has_amp;
    bool has_sd;
} comms_ctx_t;

static void comms_task(void *arg){
    comms_ctx_t *ctx = (comms_ctx_t*)arg;

    // Start call tasks based on hardware
    if (ctx->has_mic) {
        call_cfg_t *cfg_tx = (call_cfg_t*)malloc(sizeof(call_cfg_t));
        if (cfg_tx) {
            strncpy(cfg_tx->peer_ip, ctx->tel.pc_ip, sizeof(cfg_tx->peer_ip)-1);
            cfg_tx->peer_ip[sizeof(cfg_tx->peer_ip)-1] = 0;
            cfg_tx->port = ctx->call_port;
            cfg_tx->save_local = ctx->has_sd;
            xTaskCreatePinnedToCore(call_sender_task, "call_tx", 5120, cfg_tx, 5, NULL, BG_TASK_CORE);
        }
    }
    if (ctx->has_amp) {
        call_cfg_t *cfg_rx = (call_cfg_t*)malloc(sizeof(call_cfg_t));
        if (cfg_rx) {
            strncpy(cfg_rx->peer_ip, ctx->tel.pc_ip, sizeof(cfg_rx->peer_ip)-1);
            cfg_rx->peer_ip[sizeof(cfg_rx->peer_ip)-1] = 0;
            cfg_rx->port = ctx->call_port;
            cfg_rx->save_local = false;
            xTaskCreatePinnedToCore(call_receiver_task, "call_rx", 4096, cfg_rx, 5, NULL, BG_TASK_CORE);
        }
    }

    oled_render_three_lines("CALL DUPLEX",
                            ctx->has_mic ? "TX mic on" : "TX mic off",
                            ctx->has_amp ? "RX amp on" : "RX amp off");

    // Button loop for message send
    for (;;) {
        hw_button_id_t b = HW_BTN_NONE;
        if (hw_buttons_read(&b) != ESP_OK) b = HW_BTN_NONE;
        if (b == HW_BTN_A) {
            send_last_recording(&ctx->tel);
            vTaskDelay(pdMS_TO_TICKS(500)); // debounce / avoid repeats
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------- Public API ----------------
void comms_start(const comms_cfg_t *cfg){
    if (!cfg || !cfg->ssid || !cfg->peer_ip || cfg->peer_port == 0 || cfg->call_port == 0) return;

    telemetry_cfg_t tel = {
        .ssid    = cfg->ssid,
        .pass    = cfg->pass,
        .pc_ip   = cfg->peer_ip,
        .pc_port = cfg->peer_port,
    };
    ESP_ERROR_CHECK(telemetry_start(&tel));
    telemetry_start_file_receiver(cfg->peer_port, "/sdcard");

    bool has_sd = storage_sd_is_mounted();
    bool has_mic = (audio_enable_rx() == ESP_OK);
    bool has_amp = (audio_enable_tx() == ESP_OK);
    audio_set_rate(16000);

    comms_ctx_t *ctx = (comms_ctx_t*)malloc(sizeof(comms_ctx_t));
    if (!ctx) return;
    ctx->tel = tel;
    ctx->call_port = cfg->call_port;
    ctx->has_mic = has_mic;
    ctx->has_amp = has_amp;
    ctx->has_sd  = has_sd;

    xTaskCreatePinnedToCore(comms_task, "comms_ctrl", 4096, ctx, 5, NULL, BG_TASK_CORE);
}
