#include "link.h"

#include <string.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_check.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "telemetry.h"
#include "system_state.h"

static const char *TAG = "link";

static link_path_t s_active = LINK_PATH_NONE;
static telemetry_cfg_t s_wifi = {0};
static bool s_wifi_ready = false;
static bool s_espnow_ready = false;
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
static link_rx_cb_t s_rx_cb = NULL;
static void *s_rx_ctx = NULL;
static TaskHandle_t s_udp_rx_task = NULL;
static link_frame_cb_t s_frame_cb = NULL;
static void *s_frame_ctx = NULL;
static SemaphoreHandle_t s_send_lock = NULL;
static EventGroupHandle_t s_ack_eg = NULL;
static uint16_t s_seq = 1;
#define LINK_ACK_BIT (1<<0)
#define LINK_HDR_LEN 4
#define LINK_MAX_PAYLOAD 200
static TaskHandle_t s_info_task = NULL;
static uint32_t s_info_period_ms = 0;

// ---- Common Wi-Fi bring-up (for ESP-NOW and/or STA) ----
// link.c
static esp_err_t ensure_wifi_base(void){
    static bool base = false;
    if (base) return ESP_OK;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs");

    ESP_ERROR_CHECK(esp_netif_init());
    esp_event_loop_create_default(); // ignore "already created"

    // DO create the default STA here – this should be **the only place**:
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    base = true;
    return ESP_OK;
}

// ---- ESP-NOW ----
static esp_err_t link_send_espnow(const uint8_t *data, size_t len){
    if (!s_espnow_ready || !data || len == 0) return ESP_ERR_INVALID_STATE;
    const uint8_t *dest = s_peer_mac;
    return esp_now_send(dest, data, len);
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len){
    (void)info;
    if (!data || len <= 0) return;

    // Frame-aware path
    if (len >= LINK_HDR_LEN){
        uint8_t type = data[0];
        uint8_t flags = data[1];
        uint16_t seq = 0;
        memcpy(&seq, data + 2, 2);
        const uint8_t *payload = data + LINK_HDR_LEN;
        size_t plen = (size_t)len - LINK_HDR_LEN;

        // ACK handling
        if (flags & 0x80){ // ACK frame
            if (s_ack_eg){
                xEventGroupSetBits(s_ack_eg, LINK_ACK_BIT);
            }
        } else if (flags & 0x40){ // ACK requested
            uint8_t ack[LINK_HDR_LEN] = { type, 0x80, 0, 0 };
            memcpy(ack + 2, &seq, 2);
            link_send_espnow(ack, sizeof(ack));
        }

        if (s_frame_cb){
            s_frame_cb((link_msg_type_t)type, payload, plen, s_frame_ctx);
            return;
        }
    }

    if (s_rx_cb){
        s_rx_cb(data, (size_t)len, s_rx_ctx);
    }
}

static esp_err_t link_init_espnow(const link_config_t *cfg){
    esp_err_t err = ensure_wifi_base();
    if (err != ESP_OK) return err;

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "espnow init");

    if (cfg && cfg->espnow_peer_mac){
        memcpy(s_peer_mac, cfg->espnow_peer_mac, ESP_NOW_ETH_ALEN);
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        esp_now_del_peer(peer.peer_addr); // ignore failure
        err = esp_now_add_peer(&peer);
        if (err != ESP_OK){
            ESP_LOGE(TAG, "add peer failed: %d", err);
            return err;
        }
    } else {
        memset(s_peer_mac, 0xff, sizeof(s_peer_mac)); // broadcast
    }

    s_espnow_ready = true;
    esp_now_register_recv_cb(espnow_recv_cb);
    return ESP_OK;
}

// ---- Wi-Fi / UDP (reuses telemetry) ----
static esp_err_t link_init_wifi(const link_config_t *cfg){
    if (!cfg || !cfg->ssid || !cfg->pc_ip || cfg->pc_port == 0) return ESP_ERR_INVALID_ARG;
    // ensure base stack for STA
    ensure_wifi_base();
    s_wifi.ssid = cfg->ssid;
    s_wifi.pass = cfg->pass;
    s_wifi.pc_ip = cfg->pc_ip;
    s_wifi.pc_port = cfg->pc_port;
    esp_err_t err = telemetry_start(&s_wifi);
    if (err == ESP_OK){
        s_wifi_ready = true;
    }
    return err;
}

// UDP receive helper
static void udp_rx_task(void *arg){
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { s_udp_rx_task = NULL; vTaskDelete(NULL); }

    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(s_wifi.pc_port);
    if (bind(sock, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) != 0){
        close(sock);
        s_udp_rx_task = NULL;
        vTaskDelete(NULL);
    }

    uint8_t buf[1500];
    while (1){
        ssize_t n = recv(sock, buf, sizeof(buf), 0);
        if (n <= 0) continue;
        if (s_rx_cb){
            s_rx_cb(buf, (size_t)n, s_rx_ctx);
        }
    }
}

static void maybe_start_udp_rx(void){
    if (s_udp_rx_task || !s_rx_cb || !s_wifi_ready) return;
    xTaskCreate(udp_rx_task, "link_udp_rx", 3072, NULL, 4, &s_udp_rx_task);
}

// Info broadcast task (system_state snapshot)
static void info_task(void *arg){
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(s_info_period_ms ? s_info_period_ms : 1000);
    while (1){
        system_state_t st = system_state_get();
        uint8_t buf[16] = {0};
        buf[0] = 1; // version
        buf[1] = (uint8_t)st.current_led_mode;
        buf[2] = st.battery_pct[0];
        buf[3] = st.battery_pct[1];
        buf[4] = st.battery_pct[2];
        buf[5] = (uint8_t)st.connection;
        buf[6] = st.time_valid ? 1 : 0;
        buf[7] = st.hours;
        buf[8] = st.minutes;
        size_t len = 9;
        link_send_frame(LINK_MSG_INFO, buf, len, false);
        vTaskDelay(period);
    }
}

esp_err_t link_start_info_broadcast(uint32_t period_ms){
    s_info_period_ms = period_ms;
    if (s_info_task) return ESP_OK;
    if (!s_active || s_active == LINK_PATH_NONE) return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreate(info_task, "link_info", 4096, NULL, 4, &s_info_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

// ------------------------------------------------------------------

esp_err_t link_init(const link_config_t *cfg){
    if (!cfg) return ESP_ERR_INVALID_ARG;

    if (!s_send_lock){
        s_send_lock = xSemaphoreCreateMutex();
    }
    if (!s_ack_eg){
        s_ack_eg = xEventGroupCreate();
    }

    // 1) Try ESP-NOW
    esp_err_t err_espnow = link_init_espnow(cfg);
    if (err_espnow == ESP_OK) {
        s_active = LINK_PATH_ESPNOW;   // "default" path for framed link_send_frame
        system_state_set_connection(SYS_CONN_CONNECTED);
        ESP_LOGI(TAG, "link: ESP-NOW enabled");
    }

    // 2) Try Wi-Fi / UDP as well (for telemetry + audio)
    esp_err_t err_wifi = ESP_OK;
    if (cfg->ssid && cfg->pc_ip && cfg->pc_port) {
        err_wifi = link_init_wifi(cfg);
        if (err_wifi == ESP_OK) {
            ESP_LOGI(TAG, "link: Wi-Fi telemetry enabled");
            // we don't have to change s_active here; audio_rx uses raw UDP
        } else {
            ESP_LOGW(TAG, "link: wifi init failed: %s", esp_err_to_name(err_wifi));
        }
    }

    if (err_espnow == ESP_OK || err_wifi == ESP_OK) {
        if (err_espnow != ESP_OK && err_wifi == ESP_OK) {
            // Only Wi-Fi worked → reflect in state / name
            s_active = LINK_PATH_WIFI;
            system_state_set_connection(SYS_CONN_CONNECTED);
        }
        return ESP_OK;
    }

    s_active = LINK_PATH_NONE;
    system_state_set_connection(SYS_CONN_DISCONNECTED);
    return ESP_FAIL;
}

link_path_t link_active_path(void){
    return s_active;
}

const char *link_path_name(link_path_t p){
    switch (p){
        case LINK_PATH_ESPNOW: return "espnow";
        case LINK_PATH_WIFI:   return "wifi";
        default:               return "none";
    }
}

esp_err_t link_send(const uint8_t *data, size_t len){
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_FAIL;

    if (s_active == LINK_PATH_ESPNOW){
        err = link_send_espnow(data, len);
        if (err == ESP_OK) return ESP_OK;
    }
    if (s_wifi_ready){
        err = telemetry_send_buffer_udp(data, len, s_wifi.pc_ip, s_wifi.pc_port, NULL, NULL);
        if (err == ESP_OK) return ESP_OK;
    }
    return err;
}

esp_err_t link_set_rx(link_rx_cb_t cb, void *user_ctx){
    s_rx_cb = cb;
    s_rx_ctx = user_ctx;
    maybe_start_udp_rx();
    return ESP_OK;
}

esp_err_t link_set_frame_rx(link_frame_cb_t cb, void *user_ctx){
    s_frame_cb = cb;
    s_frame_ctx = user_ctx;
    maybe_start_udp_rx();
    return ESP_OK;
}

static esp_err_t send_with_ack(const uint8_t *frame, size_t len){
    if (!s_send_lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_send_lock, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3; ++attempt){
        err = link_send_espnow(frame, len);
        if (err != ESP_OK) continue;
        xEventGroupClearBits(s_ack_eg, LINK_ACK_BIT);
        EventBits_t bits = xEventGroupWaitBits(s_ack_eg, LINK_ACK_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(60));
        if (bits & LINK_ACK_BIT){
            err = ESP_OK;
            break;
        } else {
            err = ESP_ERR_TIMEOUT;
        }
    }
    xSemaphoreGive(s_send_lock);
    return err;
}

esp_err_t link_send_frame(link_msg_type_t type, const uint8_t *payload, size_t len, bool require_ack){
    if (!payload && len > 0) return ESP_ERR_INVALID_ARG;
    if (len > LINK_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[LINK_HDR_LEN + LINK_MAX_PAYLOAD];
    uint16_t seq = s_seq++;
    buf[0] = (uint8_t)type;
    buf[1] = require_ack ? 0x40 : 0x00;
    memcpy(buf + 2, &seq, 2);
    if (len) memcpy(buf + LINK_HDR_LEN, payload, len);

    if (s_active == LINK_PATH_ESPNOW && require_ack){
        return send_with_ack(buf, LINK_HDR_LEN + len);
    }
    return link_send(buf, LINK_HDR_LEN + len);
}
