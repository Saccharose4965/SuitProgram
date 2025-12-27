#include "audio_rx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/unistd.h>   // fsync
#include <sys/stat.h>
#include <dirent.h>

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
#define AUDIO_RX_RATE_LIMIT_BPS (100 * 1024)  // throttle to keep UI responsive

// Task priority (keep below the UI / main loop)
#define AUDIO_RX_TASK_PRIO     1
#define AUDIO_RX_TASK_STACK    8192
#define AUDIO_RX_WRITER_PRIO   3
#define AUDIO_RX_WRITER_STACK  4096

// Yield occasionally to keep UI responsive during sustained transfers
#define AUDIO_RX_YIELD_EVERY    4
#define AUDIO_RX_WRITER_YIELD_EVERY    16

#if CONFIG_FREERTOS_UNICORE
#define AUDIO_RX_TASK_CORE      0
#define AUDIO_RX_WRITER_CORE    0
#else
// Keep RX on core1, let writer run on core0 to drain ring promptly
#define AUDIO_RX_TASK_CORE      1
#define AUDIO_RX_WRITER_CORE    0
#endif

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static TaskHandle_t  s_rx_task     = NULL;
static int           s_listen_sock = -1;
static bool          s_stop_flag   = false;
static int           s_file_index  = 0;
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

// Find the highest existing rx_rec_<idx>.wav/.part on the SD card so we don't
// overwrite prior recordings after reboot.
static void audio_rx_sync_file_index(void)
{
    DIR *d = opendir("/sdcard");
    if (!d) {
        ESP_LOGW(TAG, "opendir(/sdcard) failed; using in-memory file index");
        return;
    }

    int max_idx = s_file_index;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        int idx = 0;
        char ext[8] = {0};
        if (sscanf(ent->d_name, "rx_rec_%d.%7s", &idx, ext) == 2) {
            if (idx > max_idx) {
                max_idx = idx;
            }
            if (strcmp(ext, "part") == 0) {
                char wav_path[64];
                snprintf(wav_path, sizeof(wav_path), "/sdcard/rx_rec_%05d.wav", idx);
                struct stat st = {0};
                if (stat(wav_path, &st) == 0) {
                    // Finished file exists; drop stale part
                    char part_path[64];
                    snprintf(part_path, sizeof(part_path), "/sdcard/%s", ent->d_name);
                    if (unlink(part_path) == 0) {
                        ESP_LOGI(TAG, "Removed stale part file '%s'", part_path);
                    }
                }
            }
        }
    }
    closedir(d);
    if (max_idx != s_file_index) {
        s_file_index = max_idx;
        ESP_LOGI(TAG, "audio_rx: resumed file index at %d", s_file_index);
    }
}

typedef struct {
    RingbufHandle_t ring;
    size_t          ring_bytes;
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
        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(
            w->ring, &item_size, AUDIO_RX_WRITE_CHUNK, pdMS_TO_TICKS(200));

        if (item) {
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
            vRingbufferReturnItem(w->ring, item);

            if (write_failed) {
                w->error = true;
                w->stop = true;
                break;
            }
            continue;
        }

        if (w->stop) {
            if (xRingbufferGetCurFreeSize(w->ring) == w->ring_bytes) {
                break;
            }
        }
    }

    fflush(w->file);
    int fd = fileno(w->file);
    if (fd >= 0) {
        fsync(fd);
    }
    fclose(w->file);
    free(w->filebuf);
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

    audio_rx_sync_file_index();

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

        // Build file name (write to .part then rename on success)
        char path_final[64];
        char path_tmp[64];
        s_file_index++;
        snprintf(path_final, sizeof(path_final), "/sdcard/rx_rec_%05d.wav", s_file_index);
        snprintf(path_tmp, sizeof(path_tmp), "/sdcard/rx_rec_%05d.part", s_file_index);
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

        // Optional recv timeout so we don't block forever
        struct timeval tv = {
            .tv_sec  = 5,
            .tv_usec = 0
        };
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RingbufHandle_t ring = xRingbufferCreateWithCaps(
            AUDIO_RX_RING_BYTES, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        audio_rx_writer_t *writer = NULL;
        if (!ring) {
            ESP_LOGW(TAG, "PSRAM ring buffer alloc failed; falling back to direct write");
        } else {
            writer = (audio_rx_writer_t *)calloc(1, sizeof(*writer));
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
                    NULL,
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
        size_t  since_flush   = 0;
        unsigned int yield_count = 0;
        bool rx_paused = false;
        TickType_t last_pause_log = xTaskGetTickCount();
        TickType_t last_stat_log  = xTaskGetTickCount();

        TickType_t last_backpressure_log = xTaskGetTickCount();
        while (!s_stop_flag) {
            TickType_t now = xTaskGetTickCount();

            // Periodic status to observe ring usage and writer state when stalled
            if ((now - last_stat_log) > pdMS_TO_TICKS(1000)) {
                size_t free_bytes = writer && ring ? xRingbufferGetCurFreeSize(ring) : 0;
                size_t writer_written = writer ? writer->total_written : total_written;
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

            ssize_t r = recv(client_sock, buf, AUDIO_RX_CHUNK_SIZE, 0);
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
                ESP_LOGI(TAG, "Client finished sending '%s'", path_final);
                break;
            }

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
                    break;
                }
                audio_rx_rate_limit((size_t)r);
            }

            if (++yield_count >= AUDIO_RX_YIELD_EVERY) {
                audio_rx_yield_soft();
                yield_count = 0;
            }
        }

        if (writer && ring) {
            writer->stop = true;
            while (!writer->done) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            bool writer_had_error = writer->error;
            size_t writer_total    = writer->total_written;
            vRingbufferDeleteWithCaps(ring);
            free(writer);
            writer = NULL;

            ESP_LOGI(TAG, "Finished '%s' (%u bytes)",
                     path_tmp, (unsigned)writer_total);
            if (writer_had_error) {
                // Abort on writer error to avoid wedging RX
                (void)unlink(path_tmp);
                break;
            }
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
        }

        shutdown(client_sock, SHUT_RDWR);
        close(client_sock);

        // If writer completed without error, rename .part → .wav; otherwise remove temp.
        if (!writer || !writer->error) {
            if (rename(path_tmp, path_final) != 0) {
                ESP_LOGE(TAG, "Failed to rename '%s' -> '%s' (errno=%d)",
                         path_tmp, path_final, errno);
            }
        } else {
            (void)unlink(path_tmp);
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
