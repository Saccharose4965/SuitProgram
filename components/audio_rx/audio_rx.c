#include "audio_rx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <sys/unistd.h>   // fsync
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

static const char *TAG = "audio_rx";

// -----------------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------------

// Size of TCP read buffer
#define AUDIO_RX_CHUNK_SIZE    4096
// PSRAM ring buffer size (decouples socket reads from SD writes)
#define AUDIO_RX_RING_BYTES    (2 * 1024 * 1024)
// Ring buffer chunk size (smaller chunks reduce blocking on near-full ring)
#define AUDIO_RX_RING_ITEM_BYTES 2048
// Minimum free space to attempt sending a partial ring chunk
#define AUDIO_RX_RING_MIN_BYTES 256
// Keep some headroom to avoid filling the ring to 0 bytes
#define AUDIO_RX_RING_GUARD_BYTES (8 * 1024)
// Pause reads when ring is too full, resume when it drains
#define AUDIO_RX_RING_LOW_WATER_BYTES (64 * 1024)
#define AUDIO_RX_RING_RESUME_BYTES    (256 * 1024)
// Writer chunk size (limits SD write bursts)
#define AUDIO_RX_WRITE_CHUNK   (16 * 1024)
// File buffering to reduce SD write overhead (DMA-capable, keep small)
#define AUDIO_RX_FILEBUF_BYTES (16 * 1024)
// Slice SD writes so other SPI devices can interleave
#define AUDIO_RX_SD_SLICE_BYTES (4 * 1024)
// Progress log cadence
#define AUDIO_RX_LOG_BYTES     (128 * 1024)
// RX rate limit (bytes/sec). Set to 0 to disable.
#define AUDIO_RX_RATE_LIMIT_BPS (100 * 1024)  // throttle to ~100 KB/s
// Optional metadata line prefix sent before file bytes.
// Format: "SUITRX1 <filename>\n" followed by raw audio bytes.
#define AUDIO_RX_META_PREFIX "SUITRX1 "
#define AUDIO_RX_META_MAX_BYTES 256
#define AUDIO_RX_NAME_MAX 96

// Task priority (keep below the UI / main loop)
#define AUDIO_RX_TASK_PRIO     1
#define AUDIO_RX_TASK_STACK    8192
#define AUDIO_RX_WRITER_PRIO   4
#define AUDIO_RX_WRITER_STACK  6144
#define AUDIO_RX_WRITER_WAIT_MIN_MS        10000
#define AUDIO_RX_WRITER_WAIT_MAX_MS        60000
#define AUDIO_RX_WRITER_IDLE_STALL_MS      15000
#define AUDIO_RX_WRITER_IDLE_POST_DRAIN_MS 30000

// Yield occasionally to keep UI responsive during sustained transfers
#define AUDIO_RX_YIELD_EVERY    4
#define AUDIO_RX_WRITER_YIELD_EVERY    16

#if CONFIG_FREERTOS_UNICORE
#define AUDIO_RX_TASK_CORE      0
#define AUDIO_RX_WRITER_CORE    0
#else
// Keep RX + writer on core1 so SD writes are not starved by BT/Wi-Fi work
// that heavily occupies core0 under load.
#define AUDIO_RX_TASK_CORE      1
#define AUDIO_RX_WRITER_CORE    1
#endif

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static TaskHandle_t  s_rx_task     = NULL;
static int           s_listen_sock = -1;
static bool          s_stop_flag   = false;
static uint8_t       s_rx_chunk_buf[AUDIO_RX_CHUNK_SIZE];

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static int create_listen_socket(int port)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return -1;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
        close(sock);
        return -1;
    }

    if (listen(sock, 2) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(sock);
        return -1;
    }

    return sock;
}

// Persist last received file path so other parts of the app can pick it up.
static void audio_rx_store_last(const char *path)
{
    if (!path) return;
    FILE *f = fopen("/sdcard/rx_rec_last.txt", "w");
    if (!f) {
        ESP_LOGW(TAG, "audio_rx: failed to write last path");
        return;
    }
    fprintf(f, "%s\n", path);
    fclose(f);
}

static bool audio_rx_path_exists(const char *path)
{
    if (!path || !path[0]) return false;
    struct stat st;
    return stat(path, &st) == 0;
}

static bool audio_rx_has_extension(const char *name)
{
    if (!name || !name[0]) return false;
    const char *dot = strrchr(name, '.');
    return dot && dot != name && dot[1] != '\0';
}

static void audio_rx_sanitize_filename(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!in || !in[0]) return;

    // Drop any path prefix from the sender.
    const char *base = in;
    for (const char *p = in; *p; ++p) {
        if (*p == '/' || *p == '\\') base = p + 1;
    }

    size_t j = 0;
    for (const char *p = base; *p && j + 1 < out_len; ++p) {
        unsigned char ch = (unsigned char)*p;
        if (isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
            out[j++] = (char)ch;
        } else if (ch == ' ') {
            out[j++] = '_';
        } else {
            out[j++] = '_';
        }
    }
    while (j > 0 && (out[j - 1] == '_' || out[j - 1] == '.')) {
        --j;
    }
    out[j] = '\0';

    if (j == 0) {
        snprintf(out, out_len, "track.wav");
        return;
    }

    if (!audio_rx_has_extension(out)) {
        size_t n = strlen(out);
        if (n + 4 < out_len) {
            memcpy(out + n, ".wav", 5); // includes trailing '\0'
        } else {
            while (n > 0 && n + 4 >= out_len) {
                out[--n] = '\0';
            }
            if (n == 0) {
                snprintf(out, out_len, "track.wav");
            } else {
                memcpy(out + n, ".wav", 5);
            }
        }
    }
}

static void audio_rx_split_ext(const char *name,
                               char *stem, size_t stem_len,
                               char *ext, size_t ext_len)
{
    if (!stem || stem_len == 0 || !ext || ext_len == 0) return;
    stem[0] = '\0';
    ext[0] = '\0';
    if (!name || !name[0]) return;

    const char *dot = strrchr(name, '.');
    if (!dot || dot == name || dot[1] == '\0') {
        snprintf(stem, stem_len, "%s", name);
        return;
    }

    size_t n_stem = (size_t)(dot - name);
    if (n_stem >= stem_len) {
        n_stem = stem_len - 1;
    }
    memcpy(stem, name, n_stem);
    stem[n_stem] = '\0';

    snprintf(ext, ext_len, "%s", dot);
}

// Build output and temp paths for a recording.
// If `name_hint` collides with an existing file, use suffixes "(1)", "(2)", ...
static void audio_rx_make_paths(char *path_final, size_t final_len,
                                char *path_tmp, size_t tmp_len,
                                const char *name_hint)
{
    if (!path_final || final_len == 0 || !path_tmp || tmp_len == 0) return;
    path_final[0] = '\0';
    path_tmp[0] = '\0';

    char safe_name[AUDIO_RX_NAME_MAX];
    audio_rx_sanitize_filename(name_hint, safe_name, sizeof(safe_name));
    if (!safe_name[0]) {
        snprintf(safe_name, sizeof(safe_name), "rx_rec.wav");
    }

    char stem[AUDIO_RX_NAME_MAX];
    char ext[AUDIO_RX_NAME_MAX];
    audio_rx_split_ext(safe_name, stem, sizeof(stem), ext, sizeof(ext));
    if (!stem[0]) {
        snprintf(stem, sizeof(stem), "rx_rec");
    }
    if (!ext[0]) {
        snprintf(ext, sizeof(ext), ".wav");
    }

    uint32_t suffix = 0;
    while (1) {
        if (suffix == 0) {
            snprintf(path_final, final_len, "/sdcard/%s%s", stem, ext);
        } else {
            snprintf(path_final, final_len, "/sdcard/%s(%u)%s",
                     stem, (unsigned)suffix, ext);
        }
        if (!audio_rx_path_exists(path_final)) {
            break;
        }
        if (suffix == UINT32_MAX) {
            break;
        }
        ++suffix;
    }

    snprintf(path_tmp, tmp_len, "%s.part", path_final);
}

static bool audio_rx_parse_meta_header(const uint8_t *buf, size_t buf_len,
                                       char *out_name, size_t out_name_len,
                                       size_t *out_data_offset)
{
    if (!buf || !out_name || out_name_len == 0 || !out_data_offset) return false;
    out_name[0] = '\0';
    *out_data_offset = 0;

    const size_t prefix_len = strlen(AUDIO_RX_META_PREFIX);
    if (buf_len < prefix_len || memcmp(buf, AUDIO_RX_META_PREFIX, prefix_len) != 0) {
        return false;
    }

    size_t scan_limit = buf_len;
    if (scan_limit > AUDIO_RX_META_MAX_BYTES) {
        scan_limit = AUDIO_RX_META_MAX_BYTES;
    }
    for (size_t i = prefix_len; i < scan_limit; ++i) {
        if (buf[i] == '\n') {
            size_t raw_len = i - prefix_len;
            char raw[AUDIO_RX_NAME_MAX];
            if (raw_len >= sizeof(raw)) {
                raw_len = sizeof(raw) - 1;
            }
            memcpy(raw, buf + prefix_len, raw_len);
            raw[raw_len] = '\0';
            while (raw_len > 0 &&
                   (raw[raw_len - 1] == '\r' ||
                    raw[raw_len - 1] == ' ' ||
                    raw[raw_len - 1] == '\t')) {
                raw[--raw_len] = '\0';
            }
            audio_rx_sanitize_filename(raw, out_name, out_name_len);
            *out_data_offset = i + 1;
            return true;
        }
    }

    return false;
}

static esp_err_t audio_rx_recv_initial_payload(int sock,
                                               uint8_t *buf,
                                               size_t buf_cap,
                                               size_t *out_len,
                                               char *out_name,
                                               size_t out_name_len)
{
    if (!buf || buf_cap == 0 || !out_len) return ESP_ERR_INVALID_ARG;
    *out_len = 0;
    if (out_name && out_name_len > 0) {
        out_name[0] = '\0';
    }

    ssize_t r = recv(sock, buf, buf_cap, 0);
    if (r == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return ESP_ERR_TIMEOUT;
        }
        return ESP_FAIL;
    }

    size_t used = (size_t)r;
    const size_t prefix_len = strlen(AUDIO_RX_META_PREFIX);
    size_t payload_off = 0;
    bool parsed_meta = false;

    if (used >= prefix_len && memcmp(buf, AUDIO_RX_META_PREFIX, prefix_len) == 0) {
        int64_t header_wait_start_us = esp_timer_get_time();
        while (!parsed_meta) {
            parsed_meta = audio_rx_parse_meta_header(
                buf, used, out_name, out_name_len, &payload_off);
            if (parsed_meta) {
                break;
            }
            if (used >= AUDIO_RX_META_MAX_BYTES || used >= buf_cap) {
                ESP_LOGW(TAG, "RX: metadata header too long; treating as raw payload");
                payload_off = 0;
                break;
            }
            r = recv(sock, buf + used, buf_cap - used, 0);
            if (r == 0) {
                ESP_LOGW(TAG, "RX: connection closed while reading metadata header");
                payload_off = 0;
                break;
            }
            if (r < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (esp_timer_get_time() - header_wait_start_us > 5000000) {
                        ESP_LOGW(TAG, "RX: metadata header timeout; treating as raw payload");
                        payload_off = 0;
                        break;
                    }
                    continue;
                }
                ESP_LOGW(TAG, "RX: metadata recv error (%d); treating as raw payload", errno);
                payload_off = 0;
                break;
            }
            used += (size_t)r;
        }
    }

    if (parsed_meta && out_name && out_name[0]) {
        ESP_LOGI(TAG, "RX metadata: name='%s'", out_name);
    }

    if (payload_off > 0 && payload_off < used) {
        memmove(buf, buf + payload_off, used - payload_off);
        used -= payload_off;
    } else if (payload_off >= used) {
        used = 0;
    }

    *out_len = used;
    return ESP_OK;
}

typedef struct {
    RingbufHandle_t ring;
    size_t          ring_bytes;
    TaskHandle_t    task;
    FILE           *file;
    uint8_t        *filebuf;
    volatile bool   stop;
    volatile bool   done;
    volatile bool   error;
    size_t          total_written;
    size_t          since_flush;
} audio_rx_writer_t;

static uint8_t *audio_rx_alloc_filebuf(FILE *f, size_t size)
{
    if (!f || size == 0) return NULL;
    // Prefer DMA-capable internal buffer; fall back as needed.
    uint8_t *buf = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!buf) {
        buf = (uint8_t *)malloc(size);
    }
    if (!buf) {
        return NULL;
    }
    if (setvbuf(f, (char *)buf, _IOFBF, size) != 0) {
        free(buf);
        return NULL;
    }
    return buf;
}

static inline void audio_rx_yield_soft(void)
{
    taskYIELD();
}

static inline void audio_rx_yield_wait(void)
{
    vTaskDelay(1);
}

static inline void audio_rx_rate_limit(size_t bytes)
{
#if AUDIO_RX_RATE_LIMIT_BPS > 0
    static int64_t window_start_us = 0;
    static size_t  window_bytes = 0;
    int64_t now = esp_timer_get_time();
    if (window_start_us == 0) {
        window_start_us = now;
    }
    window_bytes += bytes;
    int64_t elapsed_us = now - window_start_us;
    int64_t target_us = (int64_t)window_bytes * 1000000 / AUDIO_RX_RATE_LIMIT_BPS;
    if (target_us > elapsed_us) {
        int64_t sleep_us = target_us - elapsed_us;
        uint32_t sleep_ms = (uint32_t)((sleep_us + 999) / 1000);
        if (sleep_ms > 0) {
            TickType_t ticks = pdMS_TO_TICKS(sleep_ms);
            if (ticks == 0) {
                ticks = 1;
            }
            vTaskDelay(ticks);
        } else {
            audio_rx_yield_soft();
        }
    }
    if (elapsed_us > 1000000) {
        window_start_us = now;
        window_bytes = 0;
    }
#else
    (void)bytes;
#endif
}

static void audio_rx_writer_task(void *arg)
{
    audio_rx_writer_t *w = (audio_rx_writer_t *)arg;
    const size_t log_bytes = AUDIO_RX_LOG_BYTES;
    unsigned int yield_count = 0;

    while (1) {
        // Exit ASAP once asked to stop and ring is empty.
        RingbufHandle_t ring_local = w->ring;
        if ((uintptr_t)ring_local < 0x3F000000u) {
            ESP_LOGE(TAG, "Writer: invalid ring handle %p", (void *)ring_local);
            w->error = true;
            break;
        }
        if (w->stop && xRingbufferGetCurFreeSize(ring_local) == w->ring_bytes) {
            ESP_LOGI(TAG, "Writer: ring drained, finishing");
            break;
        }

        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(
            ring_local, &item_size, pdMS_TO_TICKS(200), AUDIO_RX_WRITE_CHUNK);

        if (!item) {
            continue;
        }

        bool write_failed = false;
        size_t off = 0;
        while (off < item_size) {
            size_t chunk = item_size - off;
            if (chunk > AUDIO_RX_SD_SLICE_BYTES) {
                chunk = AUDIO_RX_SD_SLICE_BYTES;
            }
            size_t written = fwrite(item + off, 1, chunk, w->file);
            if (written != chunk) {
                ESP_LOGE(TAG, "writer fwrite error (wrote %u of %u, errno=%d)",
                         (unsigned)written, (unsigned)chunk, errno);
                write_failed = true;
                break;
            }
            off += written;
            w->total_written += written;
            w->since_flush += written;
            if (w->since_flush >= log_bytes) {
                ESP_LOGI(TAG, "Writer progress: %u bytes", (unsigned)w->total_written);
                w->since_flush = 0;
            }
            if (++yield_count >= AUDIO_RX_WRITER_YIELD_EVERY) {
                audio_rx_yield_soft();
                yield_count = 0;
            }
        }
        vRingbufferReturnItem(ring_local, item);

        if (write_failed) {
            w->error = true;
            break;
        }
    }

    fflush(w->file);
    int fd = fileno(w->file);
    if (fd >= 0) {
        fsync(fd);
    }
    fclose(w->file);
    w->file = NULL;
    free(w->filebuf);
    w->filebuf = NULL;
    w->task = NULL;
    w->done = true;
    vTaskDelete(NULL);
}

// -----------------------------------------------------------------------------
// RX task: accept client, stream directly to SD
// -----------------------------------------------------------------------------

static void audio_rx_task(void *arg)
{
    int port = *(int *)arg;
    free(arg);

    s_stop_flag = false;

    s_listen_sock = create_listen_socket(port);
    if (s_listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create listen socket on port %d", port);
        s_rx_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "audio_rx listening on port %d", port);

    while (!s_stop_flag) {
        struct sockaddr_in6 source_addr; // large enough for IPv4/IPv6
        socklen_t addr_len = sizeof(source_addr);

        int client_sock = accept(s_listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (client_sock < 0) {
            if (s_stop_flag) break;
            ESP_LOGW(TAG, "accept() failed: errno=%d", errno);
            continue;
        }

        char addr_str[64];
        inet_ntop(AF_INET, &((struct sockaddr_in *)&source_addr)->sin_addr,
                  addr_str, sizeof(addr_str));
        ESP_LOGI(TAG, "New client from %s", addr_str);

        // Optional recv timeout so we don't block forever.
        struct timeval tv = {
            .tv_sec  = 5,
            .tv_usec = 0
        };
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Read an optional metadata line + first payload bytes.
        char name_hint[AUDIO_RX_NAME_MAX] = {0};
        size_t preloaded_len = 0;
        esp_err_t intro_err = audio_rx_recv_initial_payload(
            client_sock,
            s_rx_chunk_buf,
            sizeof(s_rx_chunk_buf),
            &preloaded_len,
            name_hint,
            sizeof(name_hint));
        if (intro_err != ESP_OK) {
            ESP_LOGW(TAG, "RX: no payload from %s (err=%s), closing client",
                     addr_str, esp_err_to_name(intro_err));
            shutdown(client_sock, SHUT_RDWR);
            close(client_sock);
            continue;
        }

        // Build file name (write to .part then rename on success).
        char path_final[128];
        char path_tmp[160];
        audio_rx_make_paths(path_final, sizeof(path_final),
                            path_tmp, sizeof(path_tmp),
                            name_hint);
        ESP_LOGI(TAG, "Recording will be written to '%s' (tmp '%s')", path_final, path_tmp);

        FILE *f = fopen(path_tmp, "wb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open '%s' for writing (errno=%d)", path_tmp, errno);

            // Drain client and close
            uint8_t tmp[256];
            while (recv(client_sock, tmp, sizeof(tmp), 0) > 0 && !s_stop_flag) {
                ; // discard
            }
            shutdown(client_sock, SHUT_RDWR);
            close(client_sock);
            continue;
        }

        uint8_t *filebuf = audio_rx_alloc_filebuf(f, AUDIO_RX_FILEBUF_BYTES);
        if (filebuf) {
            ESP_LOGI(TAG, "audio_rx: using %u KB file buffer",
                     (unsigned)(AUDIO_RX_FILEBUF_BYTES / 1024));
        }

        ESP_LOGI(TAG, "Writer: receiving into '%s'", path_tmp);
#if AUDIO_RX_RATE_LIMIT_BPS > 0
        ESP_LOGI(TAG, "audio_rx: rate limit %u KB/s",
                 (unsigned)(AUDIO_RX_RATE_LIMIT_BPS / 1024));
#endif

        RingbufHandle_t ring = xRingbufferCreateWithCaps(
            AUDIO_RX_RING_BYTES, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        audio_rx_writer_t *writer = NULL;
        if (!ring) {
            ESP_LOGW(TAG, "PSRAM ring buffer alloc failed; falling back to direct write");
        } else {
            writer = (audio_rx_writer_t *)heap_caps_calloc(
                1, sizeof(*writer), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!writer) {
                writer = (audio_rx_writer_t *)calloc(1, sizeof(*writer));
            }
            if (!writer) {
                ESP_LOGW(TAG, "writer alloc failed; falling back to direct write");
                vRingbufferDeleteWithCaps(ring);
                ring = NULL;
            } else {
                writer->ring = ring;
                writer->ring_bytes = xRingbufferGetCurFreeSize(ring);
                writer->file = f;
                writer->filebuf = filebuf;
                BaseType_t wok = xTaskCreatePinnedToCore(
                    audio_rx_writer_task,
                    "audio_rx_wr",
                    AUDIO_RX_WRITER_STACK,
                    writer,
                    AUDIO_RX_WRITER_PRIO,
                    &writer->task,
                    AUDIO_RX_WRITER_CORE
                );
                if (wok != pdPASS) {
                    ESP_LOGW(TAG, "writer task create failed; falling back to direct write");
                    free(writer);
                    writer = NULL;
                    vRingbufferDeleteWithCaps(ring);
                    ring = NULL;
                }
            }
        }

        uint8_t *buf = s_rx_chunk_buf;
        size_t  total_written = 0;
        size_t  total_received = 0;
        size_t  since_flush   = 0;
        bool write_failed_direct = false;
        unsigned int yield_count = 0;
        bool rx_paused = false;
        TickType_t last_pause_log = xTaskGetTickCount();
        TickType_t last_stat_log  = xTaskGetTickCount();

        TickType_t last_backpressure_log = xTaskGetTickCount();
        size_t last_writer_written = 0;
        TickType_t last_writer_progress_tick = xTaskGetTickCount();
        while (!s_stop_flag) {
            TickType_t now = xTaskGetTickCount();

            // Periodic status to observe ring usage and writer state when stalled
            if ((now - last_stat_log) > pdMS_TO_TICKS(1000)) {
                size_t free_bytes = writer && ring ? xRingbufferGetCurFreeSize(ring) : 0;
                size_t writer_written = writer ? writer->total_written : total_written;
                if (writer_written != last_writer_written) {
                    last_writer_written = writer_written;
                    last_writer_progress_tick = now;
                } else if (writer && !writer->done &&
                           (now - last_writer_progress_tick) > pdMS_TO_TICKS(3000) &&
                           free_bytes <= AUDIO_RX_RING_LOW_WATER_BYTES) {
                    ESP_LOGW(TAG, "Writer stalled >3s (ring free %u). "
                                  "Likely SD/SPI contention from concurrent workloads.",
                             (unsigned)free_bytes);
                    last_writer_progress_tick = now;
                }
                ESP_LOGI(TAG, "RX stat: ring free %u, written %u, writer %s%s",
                         (unsigned)free_bytes,
                         (unsigned)writer_written,
                         (writer && writer->done) ? "done" : "active",
                         (writer && writer->error) ? " ERROR" : "");
                last_stat_log = now;
            }

            if (writer && ring) {
                size_t free_bytes = xRingbufferGetCurFreeSize(ring);
                if (rx_paused) {
                    if (free_bytes >= AUDIO_RX_RING_RESUME_BYTES) {
                        rx_paused = false;
                    } else {
                        TickType_t now = xTaskGetTickCount();
                        if ((now - last_pause_log) > pdMS_TO_TICKS(1000)) {
                            ESP_LOGW(TAG, "RX paused: ring free %u bytes", (unsigned)free_bytes);
                            last_pause_log = now;
                        }
                        audio_rx_yield_wait();
                        continue;
                    }
                } else if (free_bytes <= AUDIO_RX_RING_LOW_WATER_BYTES) {
                    rx_paused = true;
                    last_pause_log = xTaskGetTickCount();
                    audio_rx_yield_wait();
                    continue;
                }
            }

            ssize_t r = 0;
            if (preloaded_len > 0) {
                r = (ssize_t)preloaded_len;
                preloaded_len = 0;
            } else {
                r = recv(client_sock, buf, AUDIO_RX_CHUNK_SIZE, 0);
                if (r < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // timeout, keep waiting (yield so UI stays responsive)
                        audio_rx_yield_wait();
                        continue;
                    }
                    ESP_LOGE(TAG, "recv() error from %s: errno=%d", addr_str, errno);
                    break;
                } else if (r == 0) {
                    // client closed
                    ESP_LOGI(TAG, "Client finished sending '%s'", path_tmp);
                    break;
                }
            }
            total_received += (size_t)r;

            if (writer && ring) {
                size_t off = 0;
                size_t remaining = (size_t)r;
                while (off < remaining && !s_stop_flag) {
                    size_t chunk = remaining - off;
                    if (chunk > AUDIO_RX_RING_ITEM_BYTES) {
                        chunk = AUDIO_RX_RING_ITEM_BYTES;
                    }
                    while (!s_stop_flag) {
                        size_t free_bytes = xRingbufferGetCurFreeSize(ring);
                    if (free_bytes > AUDIO_RX_RING_GUARD_BYTES) {
                        size_t usable = free_bytes - AUDIO_RX_RING_GUARD_BYTES;
                        if (usable >= AUDIO_RX_RING_MIN_BYTES) {
                            size_t send_bytes = chunk;
                            if (send_bytes > usable) {
                                send_bytes = usable;
                            }
                            if (xRingbufferSend(ring, buf + off, send_bytes,
                                                pdMS_TO_TICKS(200)) == pdTRUE) {
                                off += send_bytes;
                                break;
                            }
                        }
                    }
                        TickType_t now = xTaskGetTickCount();
                        if ((now - last_backpressure_log) > pdMS_TO_TICKS(1000)) {
                            size_t free_bytes = xRingbufferGetCurFreeSize(ring);
                            ESP_LOGW(TAG, "RX backpressure: ring free %u bytes", (unsigned)free_bytes);
                            last_backpressure_log = now;
                        }
                        if (writer->error) {
                            ESP_LOGE(TAG, "Writer error; aborting client");
                            break;
                        }
                        audio_rx_yield_wait();
                    }
                    if (writer->error) {
                        break;
                    }
                }
                audio_rx_rate_limit((size_t)r);
                if (s_stop_flag || writer->error) {
                    break;
                }
            } else {
                bool write_failed = false;
                size_t off = 0;
                while (off < (size_t)r) {
                    size_t chunk = (size_t)r - off;
                    if (chunk > AUDIO_RX_SD_SLICE_BYTES) {
                        chunk = AUDIO_RX_SD_SLICE_BYTES;
                    }
                    size_t written = fwrite(buf + off, 1, chunk, f);
                    if (written != chunk) {
                        ESP_LOGE(TAG, "fwrite error on '%s' (wrote %u of %u, errno=%d)",
                                 path_tmp, (unsigned)written, (unsigned)chunk, errno);
                        write_failed = true;
                        break;
                    }
                    off += written;
                    total_written += written;
                    since_flush   += written;

                    if (since_flush >= AUDIO_RX_LOG_BYTES) {
                        ESP_LOGI(TAG, "Writer progress '%s': %u bytes",
                                 path_tmp, (unsigned)total_written);
                        since_flush = 0;
                    }
                    if (++yield_count >= AUDIO_RX_YIELD_EVERY) {
                        audio_rx_yield_soft();
                        yield_count = 0;
                    }
                }
                if (write_failed) {
                    write_failed_direct = true;
                    break;
                }
                audio_rx_rate_limit((size_t)r);
            }

            if (++yield_count >= AUDIO_RX_YIELD_EVERY) {
                audio_rx_yield_soft();
                yield_count = 0;
            }
        }

        bool had_error = false;

        if (writer && ring) {
            writer->stop = true;
            TickType_t wait_start = xTaskGetTickCount();
            TickType_t last_progress_tick = wait_start;
            TickType_t last_wait_log = wait_start;
            size_t last_written = writer->total_written;
            bool ring_drained_last = false;
            while (!writer->done) {
                vTaskDelay(pdMS_TO_TICKS(10));
                TickType_t now = xTaskGetTickCount();
                size_t written_now = writer->total_written;
                size_t free_now = xRingbufferGetCurFreeSize(ring);
                bool ring_drained = (free_now == writer->ring_bytes);
                ring_drained_last = ring_drained;

                if (written_now != last_written) {
                    last_written = written_now;
                    last_progress_tick = now;
                }

                TickType_t elapsed = now - wait_start;
                TickType_t idle = now - last_progress_tick;
                if (elapsed > pdMS_TO_TICKS(AUDIO_RX_WRITER_WAIT_MAX_MS)) {
                    ESP_LOGE(TAG,
                             "RX: writer hard-timeout at %u ms (free=%u, written=%u, drained=%d)",
                             (unsigned)(elapsed * portTICK_PERIOD_MS),
                             (unsigned)free_now,
                             (unsigned)written_now,
                             ring_drained ? 1 : 0);
                    break;
                }
                if (elapsed > pdMS_TO_TICKS(AUDIO_RX_WRITER_WAIT_MIN_MS)) {
                    TickType_t idle_limit = ring_drained
                        ? pdMS_TO_TICKS(AUDIO_RX_WRITER_IDLE_POST_DRAIN_MS)
                        : pdMS_TO_TICKS(AUDIO_RX_WRITER_IDLE_STALL_MS);
                    if (idle > idle_limit) {
                        ESP_LOGE(TAG,
                                 "RX: writer stalled %u ms after client finished "
                                 "(free=%u, written=%u, drained=%d)",
                                 (unsigned)(idle * portTICK_PERIOD_MS),
                                 (unsigned)free_now,
                                 (unsigned)written_now,
                                 ring_drained ? 1 : 0);
                        break;
                    }
                }

                if ((now - last_wait_log) > pdMS_TO_TICKS(2000)) {
                    ESP_LOGI(TAG, "RX wait writer: free %u, written %u%s",
                             (unsigned)free_now,
                             (unsigned)written_now,
                             ring_drained ? " (drained)" : "");
                    last_wait_log = now;
                }
            }
            if (!writer->done) {
                ESP_LOGE(TAG, "RX: writer did not finish; forcing shutdown of '%s'", path_tmp);
                bool forced_close_ok = true;
                if (writer->task) {
                    vTaskDelete(writer->task);
                    writer->task = NULL;
                }
                // Best effort: flush/close the writer file to keep FS consistent.
                if (writer->file) {
                    if (fflush(writer->file) != 0) {
                        forced_close_ok = false;
                    }
                    int fd = fileno(writer->file);
                    if (fd >= 0 && fsync(fd) != 0) {
                        forced_close_ok = false;
                    }
                    if (fclose(writer->file) != 0) {
                        forced_close_ok = false;
                    }
                    writer->file = NULL;
                }
                if (writer->filebuf) {
                    free(writer->filebuf);
                    writer->filebuf = NULL;
                }
                writer->done = true;
                bool recovered = ring_drained_last &&
                                 !writer->error &&
                                 (writer->total_written >= total_received) &&
                                 forced_close_ok;
                if (recovered) {
                    ESP_LOGW(TAG, "RX: writer recovered after timeout (recv=%u written=%u)",
                             (unsigned)total_received, (unsigned)writer->total_written);
                    had_error = false;
                } else {
                    had_error = true;
                }
            } else {
                had_error |= writer->error;

                ESP_LOGI(TAG, "Finished '%s' (%u bytes, recv=%u)%s",
                         path_tmp,
                         (unsigned)writer->total_written,
                         (unsigned)total_received,
                         writer->error ? " [WRITER ERROR]" : "");
            }

            vRingbufferDeleteWithCaps(ring);
            ring = NULL;
            if (writer) {
                free(writer);
                writer = NULL;
            }

            f = NULL;
            filebuf = NULL;
        } else {
            // Final flush and fsync
            fflush(f);
            int fd = fileno(f);
            if (fd >= 0) {
                fsync(fd);
            }
            fclose(f);
            ESP_LOGI(TAG, "Finished '%s' (%u bytes)",
                     path_tmp, (unsigned)total_written);
            free(filebuf);
            had_error = write_failed_direct;
        }

        shutdown(client_sock, SHUT_RDWR);
        close(client_sock);

        // If writer completed without error, rename .part -> final name.
        if (!had_error) {
            if (rename(path_tmp, path_final) != 0) {
                ESP_LOGE(TAG, "Failed to rename '%s' -> '%s' (errno=%d)",
                         path_tmp, path_final, errno);
                (void)unlink(path_tmp);
            } else {
                audio_rx_store_last(path_final);
                ESP_LOGI(TAG, "Saved recording '%s'", path_final);
            }
        } else {
            ESP_LOGW(TAG, "Transfer failed; keeping temp file '%s' for inspection", path_tmp);
        }
    }

    if (s_listen_sock >= 0) {
        shutdown(s_listen_sock, SHUT_RDWR);
        close(s_listen_sock);
        s_listen_sock = -1;
    }

    ESP_LOGI(TAG, "audio_rx task exiting");
    s_rx_task = NULL;
    vTaskDelete(NULL);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool audio_rx_is_running(void)
{
    return s_rx_task != NULL;
}

esp_err_t audio_rx_start(int port)
{
    if (s_rx_task) {
        ESP_LOGW(TAG, "audio_rx already running");
        return ESP_OK;
    }

    if (port <= 0 || port > 65535) {
        port = AUDIO_RX_DEFAULT_PORT;
    }

    int *port_arg = (int *)malloc(sizeof(int));
    if (!port_arg) {
        return ESP_ERR_NO_MEM;
    }
    *port_arg = port;

    BaseType_t ok = xTaskCreatePinnedToCore(
        audio_rx_task,
        "audio_rx",
        AUDIO_RX_TASK_STACK,
        port_arg,
        AUDIO_RX_TASK_PRIO,
        &s_rx_task,
        AUDIO_RX_TASK_CORE
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio_rx task");
        s_rx_task = NULL;
        free(port_arg);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "audio_rx server starting on port %d", port);
    return ESP_OK;
}

esp_err_t audio_rx_stop(void)
{
    s_stop_flag = true;

    // Closing listen socket will unblock accept()
    if (s_listen_sock >= 0) {
        shutdown(s_listen_sock, SHUT_RDWR);
        close(s_listen_sock);
        s_listen_sock = -1;
    }

    // Wait for task to exit
    for (int i = 0; i < 200 && s_rx_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_rx_task) {
        ESP_LOGW(TAG, "audio_rx task did not terminate in time");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "audio_rx server stopped");
    return ESP_OK;
}
