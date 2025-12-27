#include "app_shell.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_player.h"
#include "bt_audio.h"
#include "storage_sd.h"

// Limit how many WAV files we keep in memory for the menu.
#define MUSIC_MAX_FILES 100

typedef struct {
    char paths[MUSIC_MAX_FILES][64];
    char names[MUSIC_MAX_FILES][24];
    int  count;
    int  sel;
    int  playing;   // index of currently playing song, -1 if none
    bool mounted;
    bool tried_mount;
} music_state_t;
static music_state_t s_music = {0};

// Music playback task
static TaskHandle_t s_music_task = NULL;
static volatile bool s_music_stop = false;
typedef struct {
    char *path;
    int   idx;
} music_task_arg_t;

const shell_legend_t MUSIC_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK, SHELL_ICON_CUSTOM2 },
};

void music_stop_playback(void)
{
    s_music_stop = true;
    if (s_music_task){
        audio_player_stop(); // ask playback to abort ASAP
        for (int i = 0; i < 50 && s_music_task; ++i) { // wait up to ~500 ms
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    // If still running after waiting, force kill to avoid orphaned task
    if (s_music_task){
        vTaskDelete(s_music_task);
        s_music_task = NULL;
    }
    s_music.playing = -1;
}

static void music_play_task(void *arg)
{
    music_task_arg_t *info = (music_task_arg_t*)arg;
    if (info && info->path) {
        s_music.playing = info->idx;
        audio_player_play(info->path); // blocking; runs in own task
        free(info->path);
    }
    free(info);
    s_music.playing = -1;
    s_music_task = NULL;
    vTaskDelete(NULL);
}

static bool has_wav_ext(const char *name)
{
    if (!name) return false;
    size_t n = strlen(name);
    if (n < 4) return false;
    const char *ext = name + n - 4;
    return strcasecmp(ext, ".wav") == 0;
}

static void music_scan(void)
{
    s_music.count = 0;
    DIR *d = opendir("/sdcard");
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && s_music.count < MUSIC_MAX_FILES){
        if (has_wav_ext(e->d_name)){ // FATFS may leave d_type unknown; rely on extension
            strncpy(s_music.names[s_music.count], e->d_name, sizeof(s_music.names[s_music.count]) - 1);
            s_music.names[s_music.count][sizeof(s_music.names[s_music.count]) - 1] = '\0';
            strncpy(s_music.paths[s_music.count], "/sdcard/", sizeof(s_music.paths[s_music.count]) - 1);
            strncat(s_music.paths[s_music.count], e->d_name,
                    sizeof(s_music.paths[s_music.count]) - strlen(s_music.paths[s_music.count]) - 1);
            s_music.count++;
        }
    }
    closedir(d);
    if (s_music.sel >= s_music.count) s_music.sel = s_music.count ? s_music.count - 1 : 0;
}

void music_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_music.mounted = false;
    s_music.tried_mount = false;
    music_scan();
}

static void music_start_selected(void)
{
    if (s_music.sel < 0 || s_music.sel >= s_music.count) return;
    int sel = s_music.sel;
    music_stop_playback();
    char *path = strdup(s_music.paths[sel]);
    if (!path) return;
    music_task_arg_t *info = (music_task_arg_t*)malloc(sizeof(music_task_arg_t));
    if (!info){
        free(path);
        return;
    }
    info->path = path;
    info->idx  = sel;
    s_music_stop = false;
    // Lower priority so UI stays responsive while playing.
    xTaskCreatePinnedToCore(music_play_task, "music_play", 4096, info, 4, &s_music_task, tskNO_AFFINITY);
}

void music_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        ctx->request_switch("menu", ctx->request_user_data);
        return;
    }
    if (ev->type == INPUT_EVENT_PRESS){
        if (ev->button == INPUT_BTN_A && s_music.sel > 0){
            s_music.sel--;
        } else if (ev->button == INPUT_BTN_B && s_music.sel + 1 < s_music.count){
            s_music.sel++;
        } else if (ev->button == INPUT_BTN_C && s_music.count > 0){
            if (audio_player_is_active()) {
                (void)audio_player_toggle_play_pause();
            } else {
                music_start_selected();
            }
        } else if (ev->button == INPUT_BTN_D && s_music.count > 0){
            // Always (re)start the selected track
            music_start_selected();
        }
    } else if (ev->type == INPUT_EVENT_LONG_PRESS) {
        if (ev->button == INPUT_BTN_D) {
            music_stop_playback(); // pause/stop
        }
    }
}

void music_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)w; (void)h;
    if (!s_music.mounted && !s_music.tried_mount){
        s_music.tried_mount = true;
        if (storage_mount_sd() == ESP_OK){
            s_music.mounted = true;
            music_scan();
        }
    }
    if (!s_music.mounted){
        oled_draw_text3x5(fb, x + 2, y + 2, "SD not mounted");
        return;
    }
    if (s_music.count == 0){
        oled_draw_text3x5(fb, x + 2, y + 2, "No WAV files");
        return;
    }
    oled_draw_text3x5(fb, x + 2, y + 2, "MUSIC");
    const int max_vis = 5;
    int start = 0;
    if (s_music.sel >= max_vis) start = s_music.sel - (max_vis - 1);
    int visible = s_music.count - start;
    if (visible > max_vis) visible = max_vis;
    for (int i = 0; i < visible; ++i){
        int idx = start + i;
        int yy = y + 10 + i * 8;
        char line[48];
        snprintf(line, sizeof(line), "%c %s", (idx == s_music.sel) ? '>' : ' ', s_music.names[idx]);
        oled_draw_text3x5(fb, x + 2, yy, line);
    }
}
