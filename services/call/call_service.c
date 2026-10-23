#include "call_service.h"

#include <string.h>

#include "audio.h"
#include "audio_bus.h"
#include "audio_i2s.h"
#include "audio_player.h"
#include "audio_policy.h"
#include "bt_audio.h"
#include "fft.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "microphone.h"

#define CALL_SAMPLES 92
#define CALL_RATE 16000
#define CALL_PACKET_BYTES 196

typedef struct {
    uint8_t mac[6];
    uint8_t data[CALL_PACKET_BYTES];
} call_packet_t;

static QueueHandle_t s_packets;
static TaskHandle_t s_task;
static volatile bool s_active;
static volatile bool s_talk;
static volatile esp_err_t s_error;
static volatile uint32_t s_received;
static volatile uint32_t s_sent;

void call_service_receive(link_msg_type_t type, const uint8_t *src,
                          const uint8_t *data, size_t len)
{
    if (type != LINK_MSG_CALL || !s_active || s_talk || !s_packets || !src ||
        !data || len != CALL_PACKET_BYTES || memcmp(data, "CALL", 4)) return;
    uint16_t bytes;
    memcpy(&bytes, data + 8, sizeof(bytes));
    if (bytes != CALL_SAMPLES * 2) return;
    call_packet_t packet;
    memcpy(packet.mac, src, sizeof(packet.mac));
    memcpy(packet.data, data, len);
    (void)xQueueSend(s_packets, &packet, 0);
}

static void call_worker(void *arg)
{
    (void)arg;
    bool mode = !s_talk;
    unsigned batches = 0;
    uint32_t seq = 0, last_seq = 0;
    uint8_t peer[6] = {0};
    bool have_peer = false;
    TickType_t last_rx = 0;
    int32_t samples[CALL_SAMPLES * 2];
    call_packet_t packet = {0};
    s_error = audio_i2s_set_rate(CALL_RATE);
    while (s_active && s_error == ESP_OK) {
        bool talk = s_talk;
        if (mode != talk) {
            audio_i2s_disable_all();
            s_error = talk ? audio_i2s_enable_rx() : audio_i2s_enable_tx();
            if (s_error != ESP_OK) break;
            xQueueReset(s_packets);
            mode = talk;
            have_peer = false;
        }
        if (talk) {
            size_t bytes = 0;
            esp_err_t err = i2s_channel_read(audio_i2s_rx_handle(), samples,
                                             sizeof(samples), &bytes, 50);
            if (err == ESP_ERR_TIMEOUT) continue;
            if (err != ESP_OK) { s_error = err; break; }
            if (bytes != sizeof(samples)) continue;
            memcpy(packet.data, "CALL", 4);
            memcpy(packet.data + 4, &seq, sizeof(seq));
            uint16_t size = CALL_SAMPLES * 2;
            memcpy(packet.data + 8, &size, sizeof(size));
            for (size_t i = 0; i < CALL_SAMPLES; ++i) {
                int32_t value = samples[2 * i] / 16384;
                int16_t pcm = value > 32767 ? 32767
                    : value < -32768 ? -32768 : (int16_t)value;
                memcpy(packet.data + 12 + i * 2, &pcm, sizeof(pcm));
            }
            if (link_active_path() != LINK_PATH_ESPNOW) {
                s_error = ESP_ERR_INVALID_STATE;
                break;
            }
            static const uint8_t broadcast[6] = {255,255,255,255,255,255};
            esp_err_t sent = link_send_frame_to(broadcast, LINK_MSG_CALL,
                                                packet.data, sizeof(packet.data), false);
            if (sent == ESP_OK) ++s_sent;
            ++seq;
        } else {
            if (xQueueReceive(s_packets, &packet, pdMS_TO_TICKS(50)) != pdTRUE) continue;
            TickType_t now = xTaskGetTickCount();
            uint32_t incoming;
            memcpy(&incoming, packet.data + 4, sizeof(incoming));
            if (have_peer && now - last_rx < pdMS_TO_TICKS(500)) {
                if (memcmp(peer, packet.mac, sizeof(peer)) ||
                    (int32_t)(incoming - last_seq) <= 0) continue;
            }
            memcpy(peer, packet.mac, sizeof(peer));
            have_peer = true;
            last_seq = incoming;
            last_rx = now;
            float volume = audio_get_volume();
            for (size_t i = 0; i < CALL_SAMPLES; ++i) {
                int16_t pcm;
                memcpy(&pcm, packet.data + 12 + i * 2, sizeof(pcm));
                int32_t scaled = (int32_t)(pcm * volume);
                if (scaled > 32767) scaled = 32767;
                if (scaled < -32768) scaled = -32768;
                int32_t value = scaled * 65536;
                samples[2 * i] = samples[2 * i + 1] = value;
            }
            size_t offset = 0;
            while (s_active && offset < sizeof(samples)) {
                size_t written = 0;
                esp_err_t err = i2s_channel_write(audio_i2s_tx_handle(),
                    (uint8_t *)samples + offset, sizeof(samples) - offset,
                    &written, 50);
                offset += written;
                if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                    s_error = err;
                    break;
                }
            }
            ++s_received;
        }
        if ((++batches & 15u) == 0) vTaskDelay(1);
    }
    audio_i2s_disable_all();
    s_active = false;
    audio_policy_fft_set_call_app(false);
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t call_service_start(void)
{
    if (s_task) return s_active ? ESP_OK : ESP_ERR_INVALID_STATE;
    s_error = ESP_OK;
    s_talk = false;
    s_sent = s_received = 0;
    if (!s_packets) s_packets = xQueueCreate(16, sizeof(call_packet_t));
    if (!s_packets) return s_error = ESP_ERR_NO_MEM;
    if (link_active_path() != LINK_PATH_ESPNOW)
        return s_error = ESP_ERR_INVALID_STATE;
    audio_policy_fft_set_call_app(true);
    fft_visualizer_stop();
    mic_rec_stop();
    audio_player_stop();
    bt_audio_stop_stream();
    bt_audio_disconnect();
    if (!audio_bus_init_if_needed()) {
        audio_policy_fft_set_call_app(false);
        return s_error = ESP_FAIL;
    }
    s_active = true;
    if (xTaskCreatePinnedToCore(call_worker, "suit_call", 4096, NULL, 4,
                                &s_task, 1) != pdPASS) {
        s_active = false;
        s_task = NULL;
        audio_policy_fft_set_call_app(false);
        return s_error = ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void call_service_stop(void)
{
    s_active = false;
    for (unsigned i = 0; s_task && i < 200; ++i)
        vTaskDelay(pdMS_TO_TICKS(10));
}

void call_service_toggle_talk(void)
{
    if (s_active) s_talk = !s_talk;
}

call_service_status_t call_service_get_status(void)
{
    return (call_service_status_t) {
        .active = s_active,
        .talking = s_talk,
        .error = s_error,
        .sent = s_sent,
        .received = s_received,
    };
}
