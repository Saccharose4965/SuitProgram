#include "app_shell.h"
#include "ui_draw.h"
#include "message_direct.h"
#include "message_notifications.h"

#include <errno.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_random.h"
#include "fft.h"
#include "microphone.h"
#include "oled.h"
#include "storage_sd.h"
#include "telemetry.h"
#include "audio_wav.h"
#include "esp_attr.h"

enum { MSG_MENU, MSG_NEW, MSG_OUTBOX, MSG_INBOX, MSG_FILE_LIMIT=96 };
static int s_page, s_choice;
static size_t s_file_count, s_file_selected;
static bool s_list_full;
static bool s_record_graph;
static EXT_RAM_BSS_ATTR char s_files[MSG_FILE_LIMIT][64];
static char s_playing_path[128];

static bool message_valid_wav(const char *path)
{
    FILE *f=fopen(path,"rb");
    if(!f)return false;
    uint8_t header[12];
    bool ok=fread(header,1,sizeof(header),f)==sizeof(header) &&
            !memcmp(header,"RIFF",4) && !memcmp(header+8,"WAVE",4);
    audio_wav_info_t info;
    rewind(f);
    ok=ok && audio_wav_parse_file(f,&info)==ESP_OK && info.data_size>0;
    if(ok) {
        fseek(f,0,SEEK_END);
        long size=ftell(f);
        ok=size>=0 && (uint64_t)info.data_offset+info.data_size<=(uint64_t)size;
    }
    fclose(f);return ok;
}

static const char *TAG = "app_message";

enum {
    MESSAGE_SAMPLE_RATE_HZ = 16000,
    MESSAGE_PORT_OFFSET = 1,
};

typedef struct {
    esp_err_t last_err;
    esp_err_t send_err;
    bool sd_ready;
    bool mic_ready;
    bool rx_ready;
    bool send_active;
    size_t send_sent;
    size_t send_total;
    int64_t rec_started_us;
    char outbox_dir[128];
    char inbox_dir[128];
    char last_path[128];
} message_state_t;

typedef struct {
    char path[128];
} message_send_task_ctx_t;

typedef struct {
    size_t count;
    uint64_t latest_num;
    char latest_name[64];
    char random_name[64];
} message_outbox_scan_t;

static message_state_t s_message = {
    .last_err = ESP_OK,
    .send_err = ESP_OK,
};

shell_legend_t MESSAGE_LEGEND = {
    .slots = { SHELL_ICON_RECORD, SHELL_ICON_PLAY, SHELL_ICON_REFRESH, SHELL_ICON_SELECT },
};

static void message_update_legend(void)
{
    MESSAGE_LEGEND.slots[0] = s_page==MSG_NEW ? SHELL_ICON_LEFT : SHELL_ICON_UP;
    MESSAGE_LEGEND.slots[1] = s_page==MSG_NEW ? SHELL_ICON_SELECT : SHELL_ICON_DOWN;
    MESSAGE_LEGEND.slots[2] = s_page==MSG_NEW ?
        (mic_rec_is_recording()?SHELL_ICON_STOP:SHELL_ICON_RECORD) :
        s_page==MSG_OUTBOX ? SHELL_ICON_PLAY : SHELL_ICON_NONE;
    MESSAGE_LEGEND.slots[3] = s_page==MSG_NEW || s_page==MSG_INBOX ?
        SHELL_ICON_PLAY : SHELL_ICON_SELECT;
}

static uint16_t message_port(void)
{
#if CONFIG_LINK_WIFI_PC_PORT >= 65535
    return (uint16_t)CONFIG_LINK_WIFI_PC_PORT;
#else
    return (uint16_t)(CONFIG_LINK_WIFI_PC_PORT + MESSAGE_PORT_OFFSET);
#endif
}

static const char *basename_or_self(const char *path)
{
    if (!path || !path[0]) return "(none)";
    const char *base = strrchr(path, '/');
    return base ? (base + 1) : path;
}

static bool ensure_dir(const char *path)
{
    struct stat st = {0};
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    if (mkdir(path, 0777) == 0) {
        return true;
    }
    return errno == EEXIST;
}

static bool message_prepare_dirs(void)
{
    if (storage_mount_sd() != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }

    char root_dir[128];
    if (storage_sd_make_path(root_dir, sizeof(root_dir), "messages") != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }
    if (storage_sd_make_path(s_message.outbox_dir, sizeof(s_message.outbox_dir), "messages/outbox") != ESP_OK ||
        storage_sd_make_path(s_message.inbox_dir, sizeof(s_message.inbox_dir), "messages/inbox") != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }

    if (!ensure_dir(root_dir) || !ensure_dir(s_message.outbox_dir) || !ensure_dir(s_message.inbox_dir)) {
        ESP_LOGW(TAG, "message dirs create failed");
        s_message.sd_ready = false;
        return false;
    }

    s_message.sd_ready = true;
    return true;
}

static void message_interrupt_services_for_recording(void)
{
    fft_visualizer_stop();
    audio_player_stop();

    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    if (st.state != BT_AUDIO_STATE_DISABLED && st.state != BT_AUDIO_STATE_IDLE) {
        (void)bt_audio_stop_stream();
        (void)bt_audio_disconnect();
    }
}

static esp_err_t message_ensure_mic_ready(void)
{
    if (s_message.mic_ready) return ESP_OK;
    mic_rec_cfg_t cfg = {
        .sample_rate_hz = MESSAGE_SAMPLE_RATE_HZ,
        .bclk_gpio = 4,
        .ws_gpio = 22,
        .din_gpio = 36,
        .i2s_id = 0,
    };
    esp_err_t err = mic_rec_init(&cfg);
    if (err == ESP_OK) {
        s_message.mic_ready = true;
    }
    return err;
}

static esp_err_t message_start_receiver_if_possible(void)
{
    if (s_message.rx_ready) return ESP_OK;
    if (!s_message.sd_ready && !message_prepare_dirs()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = telemetry_start_file_receiver(message_port(), s_message.inbox_dir);
    if (err == ESP_OK) {
        s_message.rx_ready = true;
    }
    return err;
}

static bool message_parse_outbox_number(const char *name, uint64_t *out_num)
{
    if (!name) return false;
    if (strncmp(name, "msg", 3) != 0) return false;

    const char *p = name + 3;
    if (*p == '_') {
        ++p;
    }
    if (!isdigit((unsigned char)*p)) return false;

    uint64_t num = 0;
    while (isdigit((unsigned char)*p)) {
        if(num>(UINT64_MAX-(uint64_t)(*p-'0'))/10u)return false;
        num = (num * 10u) + (uint64_t)(*p - '0');
        ++p;
    }
    if (strcmp(p, ".wav") != 0) return false;

    if (out_num) {
        *out_num = num;
    }
    return true;
}

static esp_err_t message_build_outbox_path_from_name(const char *name, char *out, size_t out_len)
{
    if (!name || !name[0] || !out || out_len == 0) return ESP_ERR_INVALID_ARG;
    int n = snprintf(out, out_len, "%s/%s", s_message.outbox_dir, name);
    return (n > 0 && (size_t)n < out_len) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t message_select_outbox_name(const char *name)
{
    return message_build_outbox_path_from_name(name, s_message.last_path, sizeof(s_message.last_path));
}

static esp_err_t message_scan_outbox(message_outbox_scan_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    if (!s_message.outbox_dir[0]) return ESP_ERR_INVALID_STATE;

    DIR *dir = opendir(s_message.outbox_dir);
    if (!dir) return ESP_ERR_NOT_FOUND;

    struct dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL) {
        uint64_t num = 0;
        if (!message_parse_outbox_number(ent->d_name, &num)) {
            continue;
        }
        char path[256];
        if(snprintf(path,sizeof(path),"%s/%s",s_message.outbox_dir,ent->d_name)>=(int)sizeof(path) ||
           !message_valid_wav(path)) continue;

        out->count++;
        if (!out->latest_name[0] || num > out->latest_num) {
            out->latest_num = num;
            strncpy(out->latest_name, ent->d_name, sizeof(out->latest_name) - 1);
            out->latest_name[sizeof(out->latest_name) - 1] = 0;
        }
        if ((esp_random() % out->count) == 0u) {
            strncpy(out->random_name, ent->d_name, sizeof(out->random_name) - 1);
            out->random_name[sizeof(out->random_name) - 1] = 0;
        }
    }
    closedir(dir);

    return out->count > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t message_build_record_path(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    if (!s_message.sd_ready && !message_prepare_dirs()) {
        return ESP_ERR_INVALID_STATE;
    }

    message_outbox_scan_t scan = {0};
    esp_err_t scan_err = message_scan_outbox(&scan);
    if (scan_err != ESP_OK && scan_err != ESP_ERR_NOT_FOUND) {
        return scan_err;
    }

    uint64_t next_num = 1;
    if (scan_err == ESP_OK) {
        if (scan.latest_num == UINT64_MAX) return ESP_ERR_INVALID_STATE;
        next_num = scan.latest_num + 1u;
    }

    // Invalid/unfinished old recordings must never be overwritten either.
    for (;;) {
        int n = snprintf(out, out_len, "%s/msg%" PRIu64 ".wav", s_message.outbox_dir, next_num);
        if(n<=0 || (size_t)n>=out_len) return ESP_ERR_NO_MEM;
        struct stat st;
        if(stat(out,&st)!=0) return errno==ENOENT ? ESP_OK : ESP_FAIL;
        if(next_num==UINT64_MAX) return ESP_ERR_INVALID_STATE;
        ++next_num;
    }
}

static void message_capture_last_path(void)
{
    const char *path = mic_rec_last_path();
    if (!path || !path[0]) return;
    strncpy(s_message.last_path, path, sizeof(s_message.last_path) - 1);
    s_message.last_path[sizeof(s_message.last_path) - 1] = 0;
}

static void message_restore_latest_outbox_path(void)
{
    if (!s_message.outbox_dir[0]) return;

    message_outbox_scan_t scan = {0};
    if (message_scan_outbox(&scan) != ESP_OK || !scan.latest_name[0]) return;
    if (message_select_outbox_name(scan.latest_name) != ESP_OK) {
        s_message.last_path[0] = 0;
    }
}


static void message_send_progress(size_t sent, size_t total, void *user_ctx)
{
    (void)user_ctx;
    s_message.send_sent = sent;
    s_message.send_total = total;
}

static void message_send_task(void *arg)
{
    message_send_task_ctx_t *ctx = (message_send_task_ctx_t *)arg;
    esp_err_t err = ESP_FAIL;

    telemetry_cfg_t cfg = {
        .ssid = CONFIG_LINK_WIFI_SSID,
        .pass = CONFIG_LINK_WIFI_PASS,
        .pc_ip = CONFIG_LINK_WIFI_PC_IP,
        .pc_port = (uint16_t)CONFIG_LINK_WIFI_PC_PORT,
    };

    s_message.send_sent = 0;
    s_message.send_total = 0;

    err = message_direct_send(ctx->path, message_send_progress, NULL);
    if (err != ESP_ERR_NOT_FOUND) goto done;
    ESP_LOGI(TAG, "No direct message receiver; trying PC fallback");
    err = telemetry_start(&cfg);
    if (err == ESP_OK) {
        (void)message_start_receiver_if_possible();
        err = telemetry_send_file_udp_ex(ctx->path,
                                         CONFIG_LINK_WIFI_PC_IP,
                                         message_port(),
                                         message_send_progress,
                                         NULL);
    }

done:
    s_message.send_err = err;
    s_message.last_err = err;
    s_message.send_active = false;
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t message_start_recording(void)
{
    char path[128];
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;
    esp_err_t err = message_prepare_dirs() ? ESP_OK : ESP_FAIL;
    if (err != ESP_OK) return err;

    err = message_ensure_mic_ready();
    if (err != ESP_OK) return err;

    err = message_build_record_path(path, sizeof(path));
    if (err != ESP_OK) return err;

    message_interrupt_services_for_recording();
    err = mic_rec_start(path);
    if (err == ESP_OK) {
        s_record_graph=true;
        s_message.rec_started_us = esp_timer_get_time();
        strncpy(s_message.last_path, path, sizeof(s_message.last_path) - 1);
        s_message.last_path[sizeof(s_message.last_path) - 1] = 0;
    }
    return err;
}

static esp_err_t message_stop_recording(void)
{
    esp_err_t err = mic_rec_stop();
    message_capture_last_path();
    s_message.rec_started_us = 0;
    return err;
}

static esp_err_t message_toggle_playback(void)
{
    if (!s_message.last_path[0]) return ESP_ERR_NOT_FOUND;
    if (mic_rec_is_recording()) return ESP_ERR_INVALID_STATE;
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;
    if (audio_player_is_active()) {
        if(!strcmp(s_playing_path,s_message.last_path)) return audio_player_toggle_play_pause();
        audio_player_stop();
    }
    esp_err_t err=audio_player_play(s_message.last_path);
    if(err==ESP_OK) snprintf(s_playing_path,sizeof(s_playing_path),"%s",s_message.last_path);
    return err;
}

static esp_err_t message_start_send(void)
{
    if (!s_message.last_path[0]) return ESP_ERR_NOT_FOUND;
    if (mic_rec_is_recording()) return ESP_ERR_INVALID_STATE;
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;

    message_send_task_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    strncpy(ctx->path, s_message.last_path, sizeof(ctx->path) - 1);
    s_message.send_active = true;
    s_message.send_err = ESP_OK;
    s_message.send_sent = 0;
    s_message.send_total = 0;

    BaseType_t ok = xTaskCreatePinnedToCore(message_send_task,
                                            "msg_send",
                                            6144,
                                            ctx,
                                            4,
                                            NULL,
                                            0);
    if (ok != pdPASS) {
        s_message.send_active = false;
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void message_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_page=MSG_MENU; s_choice=0;
    s_message.last_err = ESP_OK;
    s_message.send_err = ESP_OK;
    s_message.sd_ready = storage_sd_is_mounted();
    s_message.rec_started_us = 0;
    if (s_message.sd_ready || message_prepare_dirs()) {
        (void)message_direct_init();
        message_restore_latest_outbox_path();
        (void)message_start_receiver_if_possible();
    }
    if (message_ensure_mic_ready() != ESP_OK) {
        s_message.mic_ready = false;
    }
    message_update_legend();
}

void message_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    if (mic_rec_is_recording()) {
        (void)message_stop_recording();
    }
    message_update_legend();
}

static int message_name_compare(const void *a,const void *b)
{
    uint64_t na,nb;
    if(message_parse_outbox_number(a,&na) && message_parse_outbox_number(b,&nb))
        return na<nb?1:na>nb?-1:0;
    return strcmp(b,a);
}

static void message_open_list(int page)
{
    uint32_t arrivals=message_arrival_snapshot();
    s_page=page; s_file_count=0; s_file_selected=0; s_list_full=false;
    if(!message_prepare_dirs()) {s_message.last_err=ESP_FAIL;return;}
    const char *base=page==MSG_INBOX?s_message.inbox_dir:s_message.outbox_dir;
    DIR *dir=opendir(base);
    if(!dir){s_message.last_err=ESP_ERR_NOT_FOUND;return;}
    struct dirent *entry;
    while((entry=readdir(dir))) {
        size_t n=strlen(entry->d_name);
        if(n<4 || n>=sizeof(s_files[0]) || strcmp(entry->d_name+n-4,".wav"))continue;
        char path[256];
        int length=snprintf(path,sizeof(path),"%s/%s",base,entry->d_name);
        if(length<0 || length>=(int)sizeof(s_message.last_path) || !message_valid_wav(path))continue;
        if(s_file_count==MSG_FILE_LIMIT){s_list_full=true;continue;}
        strcpy(s_files[s_file_count++],entry->d_name);
    }
    closedir(dir);
    qsort(s_files,s_file_count,sizeof(s_files[0]),message_name_compare);
    if(page==MSG_INBOX && !s_list_full) message_mark_seen(arrivals);
    s_message.last_err=ESP_OK;
}

static void message_new(void)
{
    s_record_graph=false;
    s_page=MSG_NEW;
    s_message.last_path[0]=0;
}

static void message_play_selected(void)
{
    if(!s_file_selected || s_file_selected>s_file_count)return;
    const char *base=s_page==MSG_INBOX?s_message.inbox_dir:s_message.outbox_dir;
    int n=snprintf(s_message.last_path,sizeof(s_message.last_path),"%s/%s",
                   base,s_files[s_file_selected-1]);
    s_message.last_err=n<0 || n>=(int)sizeof(s_message.last_path)?
        ESP_ERR_INVALID_SIZE:message_toggle_playback();
}

void message_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if(!ev || ev->type!=INPUT_EVENT_PRESS)return;
    if(s_message.send_active)return; // keep transfer source immutable
    s_message.last_err=ESP_OK;
    if(s_page==MSG_MENU) {
        if(ev->button==INPUT_BTN_A && s_choice>0)--s_choice;
        else if(ev->button==INPUT_BTN_B && s_choice<3)++s_choice;
        else if(ev->button==INPUT_BTN_D) {
            if(s_choice==0)message_new();
            else if(s_choice<3)message_open_list(s_choice==1?MSG_OUTBOX:MSG_INBOX);
            else if(ctx && ctx->request_switch)ctx->request_switch("menu_comm",ctx->request_user_data);
        }
    } else if(s_page==MSG_NEW) {
        if(ev->button==INPUT_BTN_A) {
            if(mic_rec_is_recording())s_message.last_err=message_stop_recording();
            s_page=MSG_MENU;
        } else if(ev->button==INPUT_BTN_B)s_message.last_err=message_start_send();
        else if(ev->button==INPUT_BTN_C)s_message.last_err=mic_rec_is_recording()?
            message_stop_recording():message_start_recording();
        else if(ev->button==INPUT_BTN_D)s_message.last_err=message_toggle_playback();
    } else {
        if(ev->button==INPUT_BTN_A && s_file_selected>0)--s_file_selected;
        else if(ev->button==INPUT_BTN_B && s_file_selected<s_file_count)++s_file_selected;
        else if(ev->button==INPUT_BTN_D && !s_file_selected)s_page=MSG_MENU;
        else if(ev->button==INPUT_BTN_D && s_page==MSG_OUTBOX)message_new();
        else if((ev->button==INPUT_BTN_D && s_page==MSG_INBOX) ||
                (ev->button==INPUT_BTN_C && s_page==MSG_OUTBOX))message_play_selected();
    }
    message_update_legend();
}

static void message_line(uint8_t *fb,int x,int y,int w,int h,int row,const char *text)
{
    if(w<8 || row<0 || 2+row*8+5>h)return;
    char clipped[40];
    int n=(w-4)/4;
    if(n>(int)sizeof(clipped)-1)n=sizeof(clipped)-1;
    snprintf(clipped,sizeof(clipped),"%.*s",n,text);
    oled_draw_text3x5(fb,x+2,y+2+row*8,clipped);
}

void message_app_draw(shell_app_context_t *ctx,uint8_t *fb,int x,int y,int w,int h)
{
    (void)ctx;
    if(!fb)return;
    char line[80];
    const char *title=s_page==MSG_MENU?"MESSAGES":s_page==MSG_NEW?"NEW / auto route":
        s_page==MSG_OUTBOX?"OUTBOX C play D new":"INBOX D play/pause";
    if(s_message.last_err!=ESP_OK)title=esp_err_to_name(s_message.last_err);
    message_line(fb,x,y,w,h,0,title);
    if(s_page==MSG_MENU) {
        const char *items[]={"New","Outbox","Inbox","Back"};
        for(int i=0;i<4;i++){snprintf(line,sizeof(line),"%c %s",s_choice==i?'>':' ',items[i]);message_line(fb,x,y,w,h,i+1,line);}
    } else if(s_page==MSG_NEW) {
        if(mic_rec_is_recording())snprintf(line,sizeof(line),"Recording %ds",(int)((esp_timer_get_time()-s_message.rec_started_us)/1000000));
        else if(s_message.send_active)snprintf(line,sizeof(line),"Sending...");
        else snprintf(line,sizeof(line),"%s",basename_or_self(s_message.last_path));
        message_line(fb,x,y,w,h,1,line);
        message_line(fb,x,y,w,h,2,"C record/stop D preview");
        message_line(fb,x,y,w,h,3,"A back B send");
        // One column per 50 ms of captured audio, not per UI frame. Peak
        // envelope, frozen after stop; no SD reads and no second I2S consumer.
        if(s_record_graph && w>4 && h>=37) {
            uint8_t peaks[96];
            size_t capacity=w-4<96?(size_t)(w-4):96;
            size_t n=mic_rec_envelope(peaks,capacity);
            int top=y+33, bottom=y+h-2, centre=(top+bottom)/2;
            int radius=(bottom-top)/2;
            for(size_t i=0;i<n;i++) {
                int px=x+2+(int)i;
                // Modest display gain only; recorded samples are untouched.
                unsigned level=(unsigned)peaks[i]*4;
                if(level>255)level=255;
                int extent=(int)level*radius/255;
                for(int py=centre-extent;py<=centre+extent;py++) {
                    if(px<0 || px>=PANEL_W || py<0 || py>=PANEL_H)continue;
                    unsigned bit=py*PANEL_W+px;
                    fb[bit/8]|=(uint8_t)(1u<<(7-bit%8));
                }
            }
        }
    } else {
        int rows=(h-7)/8-1;
        if(rows<1)return;
        size_t first=s_file_selected>=(size_t)rows?s_file_selected-rows+1:0;
        for(int row=0;row<rows && first+(size_t)row<=s_file_count;row++) {
            size_t i=first+row;
            snprintf(line,sizeof(line),"%c %s",i==s_file_selected?'>':' ',
                     i?s_files[i-1]:(s_list_full?"Back (96+ files)":"Back"));
            message_line(fb,x,y,w,h,row+1,line);
        }
    }
}
