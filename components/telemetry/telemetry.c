#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "sys/stat.h"
#include "esp_mac.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>

#define TEL_DEBUG 1   // set 0 to silence the breadcrumbs
#define TEL_MAGIC 0x46494C45u  // 'FILE'

static int  s_sock = -1;
static bool s_wifi_stack_up = false;
static struct sockaddr_in s_dest = {0};
static struct sockaddr_in s_bcast = {0};
static bool s_have_dest = false;
static bool s_have_bcast = false;
static TaskHandle_t s_rx_task = NULL;
static uint8_t s_mac[6] = {0};

static EventGroupHandle_t s_net_eg;
static const int BIT_GOT_IP = (1<<0);

#if configNUMBER_OF_CORES > 1
#define BG_TASK_CORE 0
#else
#define BG_TASK_CORE 0
#endif

// Known MACs (from readme) for identity tagging (currently unused; keep for reference)
static const uint8_t MAC_A[6] __attribute__((unused)) = {0x24,0xD7,0xEB,0x6B,0x26,0x40};
static const uint8_t MAC_B[6] __attribute__((unused)) = {0x24,0xD7,0xEB,0x6B,0x26,0xC8};

static void on_got_ip(void* arg, esp_event_base_t base, int32_t id, void* data){
    ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
#if TEL_DEBUG
    printf("net: ip=%s\n", ip4addr_ntoa((const ip4_addr_t*)&e->ip_info.ip));
    fflush(stdout);
#endif
    xEventGroupSetBits(s_net_eg, BIT_GOT_IP);
}

// telemetry.c
static esp_err_t wifi_stack_init_once(void){
    if (s_wifi_stack_up) return ESP_OK;

    // No NVS, no esp_netif_init, no wifi_init, no create_default_wifi_sta here.
    // That all lives in link.c: ensure_wifi_base().

    s_net_eg = xEventGroupCreate();
    if (!s_net_eg) return ESP_ERR_NO_MEM;

    esp_read_mac(s_mac, ESP_MAC_WIFI_STA);

    esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL, NULL);

    s_wifi_stack_up = true;
    return ESP_OK;
}


static uint32_t ip4_broadcast(uint32_t ip, uint32_t netmask){
    return (ip & netmask) | (~netmask);
}

static void telemetry_try_recv_ack(uint32_t expected_total)
{
    if (s_sock < 0) return;
    uint8_t ack[16];
    struct sockaddr_in src;
    socklen_t sl = sizeof(src);
    ssize_t r = recvfrom(s_sock, ack, sizeof(ack), MSG_DONTWAIT,
                         (struct sockaddr*)&src, &sl);
    if (r >= 8) {
        uint32_t m = 0;
        uint32_t ok = 0;
        memcpy(&m, ack, 4);
        memcpy(&ok, ack + 4, 4);
        if (m == TEL_MAGIC && ok == expected_total) {
#if TEL_DEBUG
            printf("file ack received (%" PRIu32 " bytes)\n", ok);
#endif
        }
    } else {
        (void)errno;
    }
}

esp_err_t telemetry_start(const telemetry_cfg_t* cfg){
    if (!cfg || !cfg->ssid || !cfg->pc_ip || !cfg->pc_port) return ESP_ERR_INVALID_ARG;

    // Make sure base Wi-Fi stack is up:
    // (call ensure_wifi_base() somewhere before this – see next point)
    esp_err_t err = wifi_stack_init_once();
    if (err != ESP_OK) return err;

    // Wi-Fi STA config & connect
    wifi_config_t wcfg = {0};
    strncpy((char*)wcfg.sta.ssid, cfg->ssid, sizeof(wcfg.sta.ssid)-1);
    if (cfg->pass) strncpy((char*)wcfg.sta.password, cfg->pass, sizeof(wcfg.sta.password)-1);
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    esp_wifi_start();   // ok if already started
    esp_wifi_connect(); // ok if already connecting

    // Wait for IP (uses s_net_eg, now guaranteed created)
    EventBits_t bits = xEventGroupWaitBits(
        s_net_eg, BIT_GOT_IP, pdTRUE, pdTRUE, pdMS_TO_TICKS(10000));
    if (!(bits & BIT_GOT_IP)) return ESP_ERR_TIMEOUT;

#if TEL_DEBUG
    printf("net: link up\n"); fflush(stdout);
#endif

    // (Re)make socket
    if (s_sock >= 0) { close(s_sock); s_sock = -1; }
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) return ESP_FAIL;

    int yes = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    // Unicast dest (your PC)
    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family = AF_INET;
    s_dest.sin_port   = htons(cfg->pc_port);
    s_have_dest = inet_aton(cfg->pc_ip, &s_dest.sin_addr) != 0;

    // Broadcast dest (your subnet)
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif){
        esp_netif_ip_info_t ipi = {0};
        if (esp_netif_get_ip_info(netif, &ipi) == ESP_OK){
            memset(&s_bcast, 0, sizeof(s_bcast));
            s_bcast.sin_family = AF_INET;
            s_bcast.sin_port   = htons(cfg->pc_port);
            struct in_addr bc; bc.s_addr = ip4_broadcast(ipi.ip.addr, ipi.netmask.addr);
            s_bcast.sin_addr = bc;
            s_have_bcast = true;
        }
    }

    // Hello probes (unicast + broadcast)
    const char* hello = "hello\n";
    if (s_have_dest)  sendto(s_sock, hello, 6, 0, (struct sockaddr*)&s_dest,  sizeof(s_dest));
    if (s_have_bcast) sendto(s_sock, hello, 6, 0, (struct sockaddr*)&s_bcast, sizeof(s_bcast));
#if TEL_DEBUG
    printf("net: udp ready\n"); fflush(stdout);
#endif
    return ESP_OK;
}

esp_err_t telemetry_send_line(const char* line){
    if (s_sock < 0 || !line) return ESP_ERR_INVALID_STATE;
    size_t n = strlen(line);
    int ok = 0, sent;

    if (s_have_dest)  { sent = sendto(s_sock, line, n, 0, (struct sockaddr*)&s_dest,  sizeof(s_dest));  if (sent == (int)n) ok = 1; }
    if (s_have_bcast) { sent = sendto(s_sock, line, n, 0, (struct sockaddr*)&s_bcast, sizeof(s_bcast)); if (sent == (int)n) ok = 1; }

    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t telemetry_send_mpu_sample(float ax,float ay,float az,
                                    float gx,float gy,float gz,
                                    float temp_c, uint64_t t_us){
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "t_us=%llu,ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f,temp=%f\n",
        (unsigned long long)t_us, ax, ay, az, gx, gy, gz, temp_c);
    if (n <= 0 || n >= (int)sizeof(buf)) return ESP_FAIL;
    return telemetry_send_line(buf);
}

// Simple UDP file sender (best-effort). Sends header + chunks.
esp_err_t telemetry_send_file_udp_ex(const char *path,
                                     const char *peer_ip,
                                     uint16_t peer_port,
                                     telemetry_progress_cb_t cb,
                                     void *user_ctx){
    if (!path || !peer_ip || peer_port == 0 || s_sock < 0) return ESP_ERR_INVALID_ARG;

    struct stat st;
    if (stat(path, &st) != 0 || st.st_size <= 0) return ESP_ERR_NOT_FOUND;

    FILE *f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;

    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port   = htons(peer_port);
    if (!inet_aton(peer_ip, &peer.sin_addr)) {
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t magic = 0x46494C45; // 'FILE'
    const size_t CHUNK = 1024;
    uint8_t buf[CHUNK + 16];
    uint32_t seq = 0;
    size_t remaining = (size_t)st.st_size;

    // Header packet: magic + size (u32) + name length + name
    const char *fname = strrchr(path, '/');
    fname = fname ? fname + 1 : path;
    uint8_t hdr[256];
    size_t fname_len = strnlen(fname, 200);
    if (fname_len > 200) fname_len = 200;
    uint32_t total = (uint32_t)st.st_size;
    size_t hdr_len = 0;
    memcpy(hdr + hdr_len, &magic, 4); hdr_len += 4;
    memcpy(hdr + hdr_len, &total, 4); hdr_len += 4;
    // attach sender MAC
    memcpy(hdr + hdr_len, s_mac, 6); hdr_len += 6;
    hdr[hdr_len++] = (uint8_t)fname_len;
    memcpy(hdr + hdr_len, fname, fname_len); hdr_len += fname_len;
    sendto(s_sock, hdr, hdr_len, 0, (struct sockaddr*)&peer, sizeof(peer));

    while (remaining > 0){
        size_t n = remaining > CHUNK ? CHUNK : remaining;
        size_t rd = fread(buf + 12, 1, n, f);
        if (rd == 0) break;
        // Chunk header: magic, seq, size
        memcpy(buf + 0, &magic, 4);
        memcpy(buf + 4, &seq, 4);
        uint32_t sz = (uint32_t)rd;
        memcpy(buf + 8, &sz, 4);
        sendto(s_sock, buf, rd + 12, 0, (struct sockaddr*)&peer, sizeof(peer));
        remaining -= rd;
        seq++;
        if (cb) cb(total - remaining, total, user_ctx);
    }

    fclose(f);
    // Best-effort ACK (non-blocking)
    telemetry_try_recv_ack(total);
    return ESP_OK;
}

// Send in-memory buffer (call-style) with the same header/chunk/ack framing.
esp_err_t telemetry_send_buffer_udp(const uint8_t *data, size_t len,
                                    const char *peer_ip, uint16_t peer_port,
                                    telemetry_progress_cb_t cb,
                                    void *user_ctx){
    if (!data || len == 0 || !peer_ip || peer_port == 0 || s_sock < 0) return ESP_ERR_INVALID_ARG;

    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port   = htons(peer_port);
    if (!inet_aton(peer_ip, &peer.sin_addr)) return ESP_ERR_INVALID_ARG;

    const uint32_t magic = TEL_MAGIC;
    const size_t CHUNK = 1024;
    uint8_t buf[CHUNK + 12];
    uint32_t seq = 0;
    size_t remaining = len;

    // Header: magic + total + sender MAC + name len 0 (anonymous)
    uint8_t hdr[16];
    size_t hdr_len = 0;
    memcpy(hdr + hdr_len, &magic, 4); hdr_len += 4;
    uint32_t total = (uint32_t)len;
    memcpy(hdr + hdr_len, &total, 4); hdr_len += 4;
    memcpy(hdr + hdr_len, s_mac, 6); hdr_len += 6;
    hdr[hdr_len++] = 0; // no filename
    sendto(s_sock, hdr, hdr_len, 0, (struct sockaddr*)&peer, sizeof(peer));

    const uint8_t *p = data;
    while (remaining > 0){
        size_t n = remaining > CHUNK ? CHUNK : remaining;
        memcpy(buf + 12, p, n);
        uint32_t sz = (uint32_t)n;
        memcpy(buf + 0, &magic, 4);
        memcpy(buf + 4, &seq, 4);
        memcpy(buf + 8, &sz, 4);
        sendto(s_sock, buf, n + 12, 0, (struct sockaddr*)&peer, sizeof(peer));
        remaining -= n;
        p += n;
        seq++;
        if (cb) cb(len - remaining, len, user_ctx);
    }

    // Best-effort ACK (non-blocking)
    telemetry_try_recv_ack((uint32_t)len);
    return ESP_OK;
}

typedef struct {
    uint16_t port;
    char dir[96];
} tel_rx_cfg_t;

static void telemetry_rx_task(void *arg){
    tel_rx_cfg_t cfg = *(tel_rx_cfg_t*)arg;
    free(arg);

    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(cfg.port);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelete(NULL); return; }
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (bind(sock, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) != 0){
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    uint8_t buf[1500];
    for(;;){
        struct sockaddr_in src = {0};
        socklen_t slen = sizeof(src);
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&src, &slen);
        if (n <= 0 || n < 9) continue;
        uint32_t magic = 0;
        memcpy(&magic, buf, 4);
        if (magic != TEL_MAGIC) continue;

        uint32_t total = 0;
        memcpy(&total, buf + 4, 4);
        uint8_t name_len = buf[10]; // magic+total+MAC(6)+len
        if (11 + name_len > (size_t)n) continue;
        char name[64] = {0};
        if (name_len >= sizeof(name)) name_len = sizeof(name) - 1;
        memcpy(name, buf + 11, name_len);

        char path[160];
        snprintf(path, sizeof(path), "%s/%s", cfg.dir, name);
        FILE *f = NULL;
        bool save_to_sd = (name_len > 0 && cfg.dir[0]);
        if (save_to_sd) {
            f = fopen(path, "wb");
        }

        size_t written = 0;
        while (written < total){
            ssize_t r = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&src, &slen);
            if (r < 12) continue;
            uint32_t mag2=0, seq=0, sz=0;
            memcpy(&mag2, buf, 4);
            memcpy(&seq,  buf+4, 4);
            memcpy(&sz,   buf+8, 4);
            if (mag2 != TEL_MAGIC || sz > (uint32_t)(r - 12)) continue;
            size_t to_write = sz;
            if (written + to_write > total) to_write = total - written;
            if (f) fwrite(buf + 12, 1, to_write, f);
            // If no SD/file, could route to an in-memory buffer / stream callback.
            written += to_write;
            (void)seq;
        }
        if (f) { fflush(f); fclose(f); }
#if TEL_DEBUG
        if (save_to_sd) printf("recv file: %s (%u bytes)\n", path, (unsigned)written);
        else printf("recv %u bytes (no save)\n", (unsigned)written);
#endif
    }
}

esp_err_t telemetry_start_file_receiver(uint16_t listen_port, const char *save_dir){
    if (listen_port == 0 || !save_dir) return ESP_ERR_INVALID_ARG;
    if (s_rx_task) return ESP_OK;
    tel_rx_cfg_t *cfg = (tel_rx_cfg_t*)malloc(sizeof(tel_rx_cfg_t));
    if (!cfg) return ESP_ERR_NO_MEM;
    cfg->port = listen_port;
    strncpy(cfg->dir, save_dir, sizeof(cfg->dir)-1);
    cfg->dir[sizeof(cfg->dir)-1] = 0;
    BaseType_t ok = xTaskCreatePinnedToCore(telemetry_rx_task, "tel_rx", 4096, cfg, 4, &s_rx_task, BG_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
