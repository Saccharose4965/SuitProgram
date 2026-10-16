#include "app_call.h"
#include "app_led_support.h"
#include "audio.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "microphone.h"
#include "fft.h"
#include "shell_audio.h"
#include "oled.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>

#define CALL_SAMPLES 92
#define CALL_RATE 16000
typedef struct { uint8_t mac[6]; uint8_t data[196]; } call_packet_t;
static QueueHandle_t s_packets;
static TaskHandle_t s_task;
static volatile bool s_active, s_talk;
static volatile esp_err_t s_error;
static volatile uint32_t s_received, s_sent;

void call_app_receive(link_msg_type_t type, const uint8_t *src, const uint8_t *data, size_t len)
{
    if (type != LINK_MSG_CALL || !s_active || s_talk || !s_packets || !src ||
        len != 196 || memcmp(data, "CALL", 4)) return;
    uint16_t bytes;
    memcpy(&bytes, data+8, 2);
    if (bytes != CALL_SAMPLES*2) return;
    call_packet_t p;
    memcpy(p.mac, src, 6); memcpy(p.data, data, len);
    (void)xQueueSend(s_packets, &p, 0);
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
    int32_t samples[CALL_SAMPLES*2];
    call_packet_t packet = {0};
    s_error = audio_set_rate(CALL_RATE);
    while (s_active && s_error == ESP_OK) {
        bool talk = s_talk;
        if (mode != talk) {
            audio_disable_all(); // flush the old direction before reusing I2S
            s_error = talk ? audio_enable_rx() : audio_enable_tx();
            if (s_error != ESP_OK) break;
            xQueueReset(s_packets);
            mode = talk; have_peer = false;
        }
        if (talk) {
            size_t bytes = 0;
            esp_err_t err = i2s_channel_read(audio_rx_handle(), samples, sizeof(samples), &bytes, 50);
            if (err == ESP_ERR_TIMEOUT) continue;
            if (err != ESP_OK) { s_error=err; break; }
            if (bytes != sizeof(samples)) continue;
            memcpy(packet.data, "CALL", 4);
            memcpy(packet.data+4, &seq, 4);
            uint16_t size = CALL_SAMPLES*2;
            memcpy(packet.data+8, &size, 2);
            for (size_t i=0;i<CALL_SAMPLES;i++) {
                int32_t v = samples[2*i] / 16384;
                int16_t pcm = v>32767 ? 32767 : v < -32768 ? -32768 : v;
                memcpy(packet.data+12+i*2, &pcm, 2);
            }
            if (link_active_path() != LINK_PATH_ESPNOW) { s_error=ESP_ERR_INVALID_STATE; break; }
            static const uint8_t broadcast[6]={255,255,255,255,255,255};
            esp_err_t sent=link_send_frame_to(broadcast,LINK_MSG_CALL, packet.data, sizeof(packet.data), false);
            if (sent == ESP_OK) ++s_sent;
            ++seq;
        } else {
            if (xQueueReceive(s_packets, &packet, pdMS_TO_TICKS(50)) != pdTRUE) continue;
            TickType_t now=xTaskGetTickCount();
            uint32_t incoming; memcpy(&incoming, packet.data+4, 4);
            if (have_peer && now-last_rx < pdMS_TO_TICKS(500)) {
                if (memcmp(peer, packet.mac, 6) || (int32_t)(incoming-last_seq)<=0) continue;
            }
            memcpy(peer, packet.mac, 6); have_peer=true; last_seq=incoming; last_rx=now;
            float volume=audio_get_volume();
            for (size_t i=0;i<CALL_SAMPLES;i++) {
                int16_t pcm; memcpy(&pcm, packet.data+12+i*2, 2);
                int32_t scaled=(int32_t)(pcm*volume);
                if(scaled>32767)scaled=32767;
                if(scaled<-32768)scaled=-32768;
                int32_t value=scaled*65536;
                samples[2*i]=samples[2*i+1]=value;
            }
            size_t offset=0;
            while (s_active && offset<sizeof(samples)) {
                size_t written=0;
                esp_err_t err=i2s_channel_write(audio_tx_handle(), (uint8_t *)samples+offset,
                                                sizeof(samples)-offset, &written, 50);
                offset+=written;
                if (err!=ESP_OK && err!=ESP_ERR_TIMEOUT) { s_error=err; break; }
            }
            ++s_received;
        }
        // Allow lower-priority file/audio tasks and Idle to run under radio load.
        if ((++batches & 15u) == 0) vTaskDelay(1);
    }
    audio_disable_all();
    s_active=false;
    led_ui_fft_set_call_app(false);
    s_task=NULL;
    vTaskDelete(NULL);
}

void call_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    if (s_task) return;
    s_error=ESP_OK; s_talk=false; s_sent=s_received=0;
    if (!s_packets) s_packets=xQueueCreate(16,sizeof(call_packet_t));
    if (!s_packets) { s_error=ESP_ERR_NO_MEM; return; }
    if (link_active_path()!=LINK_PATH_ESPNOW) { s_error=ESP_ERR_INVALID_STATE; return; }
    led_ui_fft_set_call_app(true);
    fft_set_display_enabled(false); fft_visualizer_stop();
    mic_rec_stop(); audio_player_stop();
    // Call uses the integrated speaker, not a high-latency A2DP sink.
    bt_audio_stop_stream(); bt_audio_disconnect();
    if (!shell_audio_init_if_needed()) { s_error=ESP_FAIL; led_ui_fft_set_call_app(false); return; }
    s_active=true;
    if (xTaskCreatePinnedToCore(call_worker,"suit_call",4096,NULL,4,&s_task,1)!=pdPASS) {
        s_active=false; s_task=NULL; s_error=ESP_ERR_NO_MEM;
        led_ui_fft_set_call_app(false);
    }
}

void call_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx; s_active=false;
    for (unsigned i=0;s_task && i<200;i++) vTaskDelay(pdMS_TO_TICKS(10));
    // Worker retains the FFT exclusion until I2S cleanup completes.
}

void call_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (ev && ev->type==INPUT_EVENT_PRESS && ev->button==INPUT_BTN_A && s_active) s_talk=!s_talk;
}

void call_app_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx;(void)w;(void)h;
    char line[48];
    oled_draw_text3x5(fb,x+2,y+2,"CALL - DIRECT SUIT");
    oled_draw_text3x5(fb,x+2,y+12,s_active ? (s_talk?"A: stop talking":"A: talk / listening") : "Call stopped");
    snprintf(line,sizeof(line),"tx:%lu rx:%lu",(unsigned long)s_sent,(unsigned long)s_received);
    oled_draw_text3x5(fb,x+2,y+22,line);
    oled_draw_text3x5(fb,x+2,y+32,esp_err_to_name(s_error));
}
