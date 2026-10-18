#include "app_shell.h"
#include "app_led_support.h"
#include "shell_audio.h"
#include "arm_pose.h"
#include "orientation_service.h"
#include "audio.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "microphone.h"
#include "oled.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "synth_voice.h"
#include <stdio.h>

static arm_pose_t s_pose;
static synth_voice_t s_voice;
static bool s_muted, s_worker, s_stop;
static bool s_bluetooth;
static float s_hz, s_gain;
static int64_t s_updated;
static esp_err_t s_error;
static portMUX_TYPE s_mux=portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_done_storage;
static SemaphoreHandle_t s_done;

static bool sample(imu_quat_t *q)
{
    uint64_t stamp;
    return orientation_service_snapshot(q,&stamp) &&
        (uint64_t)esp_timer_get_time()-stamp<=250000;
}
static size_t generate(int16_t *out,size_t n,void *user)
{
    (void)user;
    portENTER_CRITICAL(&s_mux);
    float hz=s_hz,gain=s_gain;
    bool stop=s_stop;
    int64_t updated=s_updated;
    portEXIT_CRITICAL(&s_mux);
    if(stop || esp_timer_get_time()-updated>250000) gain=0;
    if(stop && s_voice.gain<.0001f) return 0;
    synth_voice_render(&s_voice,out,n,hz,gain);
    return n;
}
static bool stopping(void)
{
    portENTER_CRITICAL(&s_mux);
    bool stop=s_stop;
    portEXIT_CRITICAL(&s_mux);
    return stop;
}
static size_t generate_bt(int16_t *out,size_t n,int rate,void *user)
{
    s_voice.rate=rate;
    return generate(out,n,user);
}
static void worker(void *unused)
{
    (void)unused;
    esp_err_t err=ESP_OK;
    if(bt_audio_can_stream()) {
        // Allow a previous suspend command to finish; no repeated start kicks.
        int64_t deadline=esp_timer_get_time()+2000000;
        while(bt_audio_media_cmd_pending() && !stopping() && esp_timer_get_time()<deadline)
            vTaskDelay(pdMS_TO_TICKS(10));
        bool attached=false,started=false;
        int rate=bt_audio_get_peer_sample_rate();
        if(!stopping() && !bt_audio_media_cmd_pending() && (rate==44100 || rate==48000)) {
            attached=bt_audio_set_generator(generate_bt,NULL)==ESP_OK;
            if(attached) started=bt_audio_start_stream()==ESP_OK;
        }
        if(started) {
            portENTER_CRITICAL(&s_mux); s_bluetooth=true; portEXIT_CRITICAL(&s_mux);
            while(!stopping() && bt_audio_can_stream()) {
                bt_audio_status_t status;
                bt_audio_get_status(&status);
                if(!status.streaming && !bt_audio_media_cmd_pending()) break;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if(attached) (void)bt_audio_set_generator(NULL,NULL);
        if(started) {
            deadline=esp_timer_get_time()+2000000;
            while(bt_audio_media_cmd_pending() && esp_timer_get_time()<deadline)
                vTaskDelay(pdMS_TO_TICKS(10));
            (void)bt_audio_stop_stream();
        }
    }
    // Startup failure/disconnection falls back once, without reconnect loops.
    portENTER_CRITICAL(&s_mux); s_bluetooth=false; portEXIT_CRITICAL(&s_mux);
    if(!stopping()) {
        s_voice.rate=SYNTH_RATE;
        s_voice.gain=0; // fade in on route change, never jump in amplitude
        if(shell_audio_init_if_needed()) err=audio_play_generated(generate,NULL,SYNTH_RATE);
        else err=ESP_FAIL;
    }
    portENTER_CRITICAL(&s_mux);
    s_error=err;
    portEXIT_CRITICAL(&s_mux);
    xSemaphoreGive(s_done);
    vTaskDelete(NULL);
}
void synth_init(shell_app_context_t *ctx)
{
    (void)ctx;
    led_ui_fft_set_synth_app(true);
    s_pose=(arm_pose_t){0}; s_muted=true; s_worker=false;
    s_stop=false; s_bluetooth=false; s_hz=220; s_gain=0; s_updated=esp_timer_get_time(); s_error=ESP_OK;
    bt_audio_status_t status;
    bt_audio_get_status(&status);
    if(mic_rec_is_recording() || audio_player_is_active() || audio_output_active() || status.streaming) {
        s_error=ESP_ERR_INVALID_STATE; return;
    }
    if(!s_done) s_done=xSemaphoreCreateBinaryStatic(&s_done_storage);
    while(xSemaphoreTake(s_done,0)==pdTRUE) {}
    synth_voice_init(&s_voice);
    s_worker=xTaskCreate(worker,"synth",6144,NULL,3,NULL)==pdPASS;
    if(!s_worker) s_error=ESP_ERR_NO_MEM;
}
void synth_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_mux); s_stop=true; portEXIT_CRITICAL(&s_mux);
    // Bounded writes and a short release envelope; never delete a task in I2S.
    if(s_worker) xSemaphoreTake(s_done,portMAX_DELAY);
    s_worker=false;
    led_ui_fft_set_synth_app(false);
}
void synth_input(shell_app_context_t *ctx,const input_event_t *ev)
{
    if(!ev || ev->type!=INPUT_EVENT_PRESS) return;
    if(ev->button==INPUT_BTN_A) ctx->request_switch("menu_sim",ctx->request_user_data);
    else if(ev->button==INPUT_BTN_B) {
        imu_quat_t q;
        if(sample(&q)) { arm_pose_calibrate(&s_pose,q); s_muted=true; }
    } else if(ev->button==INPUT_BTN_D && s_pose.calibrated) s_muted=!s_muted;
}
void synth_tick(shell_app_context_t *ctx,float dt)
{
    (void)ctx; (void)dt;
    imu_quat_t q; arm_vec3_t d;
    float hz=220,gain=0;
    if(sample(&q) && arm_pose_direction(&s_pose,q,true,&d)) {
        // Three octaves; horizontal arm = A3. Left/right aim controls loudness.
        hz=220*exp2f(1.5f*fmaxf(-1,fminf(1,d.y)));
        if(!s_muted) gain=.5f*(1+fmaxf(-1,fminf(1,d.x)));
    }
    portENTER_CRITICAL(&s_mux);
    s_hz=hz; s_gain=gain; s_updated=esp_timer_get_time();
    portEXIT_CRITICAL(&s_mux);
}
void synth_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx; (void)w; (void)h;
    portENTER_CRITICAL(&s_mux);
    float hz=s_hz,gain=s_gain; esp_err_t err=s_error; bool bluetooth=s_bluetooth;
    portEXIT_CRITICAL(&s_mux);
    char line[40];
    oled_draw_text3x5(fb,x+2,y+2,bluetooth?"Arm Synth / Bluetooth":"Arm Synth / speaker");
    if(err!=ESP_OK) {
        oled_draw_text3x5(fb,x+2,y+14,esp_err_to_name(err));
        oled_draw_text3x5(fb,x+2,y+24,"Audio busy? Exit/reopen");
    } else {
        snprintf(line,sizeof(line),"%.0f Hz / %.0f%%",hz,gain*100);
        oled_draw_text3x5(fb,x+2,y+14,s_pose.calibrated?line:"Fold arm; B calibrates");
        oled_draw_text3x5(fb,x+2,y+24,s_muted?"D: unmute":"D: mute");
        oled_draw_text3x5(fb,x+2,y+34,"Up: pitch / side: volume");
    }
    oled_draw_text3x5(fb,x+2,y+44,"A back / B calibrate");
}
