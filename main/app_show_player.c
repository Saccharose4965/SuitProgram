#include "app_shell.h"
#include "shell_apps.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"

#include "oled.h"
#include "show_package.h"
#include "show_runtime.h"
#include "storage_sd.h"

#define SHOW_PLAYER_MAX_SHOWS 32
#define SHOW_PLAYER_NAME_LEN  28

static const char *TAG = "app_show_player";

const shell_legend_t SHOW_PLAYER_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_SELECT },
};

typedef struct {
    char shows[SHOW_PLAYER_MAX_SHOWS][SHOW_PLAYER_NAME_LEN];
    int count;
    size_t selected;
    uint8_t role_id;
    bool mounted;
    bool tried_mount;
    bool info_valid;
    int info_show_index;
    show_package_info_t info;
    char status[24];
} show_player_state_t;

static show_player_state_t s_show = {0};

static int show_name_compare(const void *a, const void *b)
{
    return strcmp((const char *)a, (const char *)b);
}

static const char *show_role_name(uint8_t role_id)
{
    switch (role_id % 3u) {
        case 1: return "B";
        case 2: return "C";
        default: return "A";
    }
}

static bool show_player_make_child_path(char *dst, size_t dst_len,
                                        const char *parent, const char *child)
{
    if (!dst || dst_len == 0 || !parent || !child) return false;

    size_t parent_len = strlen(parent);
    size_t child_len = strlen(child);
    if (parent_len + 1u + child_len >= dst_len) return false;

    memcpy(dst, parent, parent_len);
    dst[parent_len] = '/';
    memcpy(dst + parent_len + 1u, child, child_len);
    dst[parent_len + 1u + child_len] = '\0';
    return true;
}

static bool show_player_make_show_bin_path(char *dst, size_t dst_len,
                                           const char *show_dir)
{
    static const char suffix[] = "/show.bin";
    if (!dst || dst_len == 0 || !show_dir) return false;

    size_t dir_len = strlen(show_dir);
    size_t suffix_len = sizeof(suffix) - 1u;
    if (dir_len + suffix_len >= dst_len) return false;

    memcpy(dst, show_dir, dir_len);
    memcpy(dst + dir_len, suffix, suffix_len + 1u);
    return true;
}

static void show_player_copy_row_label(char *dst, size_t dst_len,
                                       char marker, const char *label)
{
    if (!dst || dst_len == 0) return;
    if (!label) label = "?";

    if (dst_len == 1) {
        dst[0] = '\0';
        return;
    }

    dst[0] = marker;
    dst[1] = '\0';
    if (dst_len == 2) return;

    dst[1] = ' ';
    size_t avail = dst_len - 3u;
    size_t label_len = strnlen(label, avail);
    memcpy(dst + 2, label, label_len);
    dst[2 + label_len] = '\0';
}

static void show_player_set_status(const char *status)
{
    if (!status) status = "";
    snprintf(s_show.status, sizeof(s_show.status), "%s", status);
}

static void show_player_clamp_selection(void)
{
    size_t total = (s_show.count > 0) ? (size_t)(s_show.count + 2) : 2u;
    if (s_show.selected >= total) {
        s_show.selected = total - 1u;
    }
}

static void show_player_scan(void)
{
    s_show.count = 0;
    s_show.info_valid = false;
    s_show.info_show_index = -1;

    char root[96];
    if (storage_sd_make_path(root, sizeof(root), "shows") != ESP_OK) {
        show_player_set_status("path err");
        return;
    }

    DIR *dir = opendir(root);
    if (!dir) {
        show_player_set_status("no /shows");
        return;
    }

    struct dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL && s_show.count < SHOW_PLAYER_MAX_SHOWS) {
        const char *name = ent->d_name;
        if (!name || name[0] == '.') continue;

        char show_dir[128];
        char show_bin[160];
        size_t name_len = strlen(name);
        if (name_len >= SHOW_PLAYER_NAME_LEN) continue;
        if (!show_player_make_child_path(show_dir, sizeof(show_dir), root, name)) continue;
        if (!show_player_make_show_bin_path(show_bin, sizeof(show_bin), show_dir)) continue;

        struct stat st = {0};
        if (stat(show_dir, &st) != 0 || !S_ISDIR(st.st_mode)) continue;
        if (stat(show_bin, &st) != 0 || !S_ISREG(st.st_mode)) continue;

        memcpy(s_show.shows[s_show.count], name, name_len + 1u);
        s_show.count++;
    }

    closedir(dir);

    if (s_show.count > 1) {
        qsort(s_show.shows, (size_t)s_show.count, sizeof(s_show.shows[0]), show_name_compare);
    }

    show_player_clamp_selection();
    if (s_show.count == 0) {
        show_player_set_status("no shows");
    } else {
        show_player_set_status("ready");
    }
}

static int show_player_selected_show_index(void)
{
    if (s_show.selected < 2u) return -1;
    int idx = (int)(s_show.selected - 2u);
    if (idx < 0 || idx >= s_show.count) return -1;
    return idx;
}

static void show_player_refresh_selected_info(void)
{
    int show_idx = show_player_selected_show_index();
    if (show_idx < 0) {
        s_show.info_valid = false;
        s_show.info_show_index = -1;
        return;
    }
    if (s_show.info_valid && s_show.info_show_index == show_idx) {
        return;
    }

    show_package_info_t info = {0};
    esp_err_t err = show_package_load_info(s_show.shows[show_idx], &info);
    if (err != ESP_OK) {
        s_show.info_valid = false;
        s_show.info_show_index = -1;
        show_player_set_status("bad show");
        ESP_LOGW(TAG, "show load info failed for %s: %s",
                 s_show.shows[show_idx], esp_err_to_name(err));
        return;
    }

    s_show.info = info;
    s_show.info_valid = true;
    s_show.info_show_index = show_idx;
    show_player_set_status("inspected");
}

void show_player_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    uint8_t saved_role = show_runtime_get_status().role_id;
    memset(&s_show, 0, sizeof(s_show));
    s_show.role_id = saved_role % SHOW_MAX_ROLES;
    s_show.info_show_index = -1;

    esp_err_t err = storage_mount_sd();
    if (err == ESP_OK) {
        s_show.mounted = true;
        show_player_scan();
        show_player_refresh_selected_info();
    } else {
        s_show.mounted = false;
        s_show.tried_mount = true;
        show_player_set_status("sd fail");
        ESP_LOGW(TAG, "storage_mount_sd failed: %s", esp_err_to_name(err));
    }
}

void show_player_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;

    if (!s_show.mounted && !s_show.tried_mount) {
        s_show.tried_mount = true;
        if (storage_mount_sd() == ESP_OK) {
            s_show.mounted = true;
            show_player_scan();
            show_player_refresh_selected_info();
        }
    }

    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) {
            if (s_show.selected > 0u) {
                s_show.selected--;
                show_player_refresh_selected_info();
            }
            return;
        }
        if (ev->button == INPUT_BTN_B) {
            size_t total = (s_show.count > 0) ? (size_t)(s_show.count + 2) : 2u;
            if (s_show.selected + 1u < total) {
                s_show.selected++;
                show_player_refresh_selected_info();
            }
            return;
        }
        if (ev->button == INPUT_BTN_C) {
            if (show_runtime_owns_leds()) {
                show_runtime_status_t runtime = show_runtime_get_status();
                if (runtime.authority) {
                    show_runtime_stop(true);
                    show_player_set_status("stopped");
                } else {
                    show_player_set_status("master controls");
                }
                return;
            }
            s_show.role_id = (uint8_t)((s_show.role_id + 1u) % 3u);
            esp_err_t err = show_runtime_set_role(s_show.role_id);
            show_player_set_status(err == ESP_OK ? "role set" : "role failed");
            return;
        }
        if (ev->button == INPUT_BTN_D) {
            if (s_show.selected == 0u) {
                if (ctx->request_switch) {
                    ctx->request_switch("menu", ctx->request_user_data);
                }
                return;
            }
            if (s_show.selected == 1u) {
                s_show.role_id = (uint8_t)((s_show.role_id + 1u) % 3u);
                esp_err_t err = show_runtime_set_role(s_show.role_id);
                show_player_set_status(err == ESP_OK ? "role set" : "role failed");
                return;
            }
            show_player_refresh_selected_info();
            if (s_show.info_valid) {
                int show_idx = show_player_selected_show_index();
                show_runtime_status_t runtime = show_runtime_get_status();
                const char *slug = s_show.shows[show_idx];
                esp_err_t err;
                if (runtime.owns_leds && runtime.authority &&
                    strcmp(runtime.slug, slug) == 0) {
                    err = show_runtime_toggle_master_pause();
                } else if (runtime.owns_leds && !runtime.authority) {
                    show_player_set_status("following master");
                    return;
                } else {
                    if (runtime.owns_leds) show_runtime_stop(true);
                    err = show_runtime_load(slug, s_show.role_id);
                    if (err == ESP_OK) err = show_runtime_start_master();
                }
                show_player_set_status(err == ESP_OK ? "transport" : esp_err_to_name(err));
            }
            return;
        }
    } else if (ev->type == INPUT_EVENT_LONG_PRESS) {
        if (ev->button == INPUT_BTN_D) {
            show_player_scan();
            show_player_refresh_selected_info();
        }
    }
}

void show_player_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)h;
    if (!fb) return;

    if (!s_show.mounted) {
        oled_draw_text3x5(fb, x + 2, y + 2, "SHOW PLAYER");
        oled_draw_text3x5(fb, x + 2, y + 12, "SD not mounted");
        oled_draw_text3x5(fb, x + 2, y + 22, s_show.status);
        return;
    }

    oled_draw_text3x5(fb, x + 2, y + 2, "SHOW PLAYER");

    char line[40];
    show_runtime_status_t runtime = show_runtime_get_status();
    const char *sync = runtime.authority ? "M" : (runtime.owns_leds ? "F" : "-");
    snprintf(line, sizeof(line), "role:%s %s %s%s", show_role_name(s_show.role_id),
             sync, show_runtime_state_name(runtime.state), runtime.synced ? "*" : "");
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    if (runtime.loaded) {
        snprintf(line, sizeof(line), "%lu/%lus peers:%u",
                 (unsigned long)(runtime.playhead_ms / 1000u),
                 (unsigned long)((runtime.duration_ms + 500u) / 1000u),
                 (unsigned)runtime.peer_count);
        oled_draw_text3x5(fb, x + 2, y + 18, line);
    } else if (s_show.info_valid) {
        snprintf(line, sizeof(line), "dur:%lus v:%u.%u",
                 (unsigned long)((s_show.info.duration_ms + 500u) / 1000u),
                 (unsigned)s_show.info.version_major,
                 (unsigned)s_show.info.version_minor);
        oled_draw_text3x5(fb, x + 2, y + 18, line);
    } else {
        oled_draw_text3x5(fb, x + 2, y + 18, "no show info");
    }

    if (runtime.error[0]) {
        oled_draw_text3x5(fb, x + 2, y + 26, runtime.error);
    } else if (s_show.info_valid) {
        uint8_t role = (uint8_t)(s_show.role_id % 3u);
        uint32_t clips = s_show.info.roles[role].clip_count;
        snprintf(line, sizeof(line), "clips:%lu bpm:%lu", (unsigned long)clips,
                 (unsigned long)((s_show.info.tempo_millibpm + 500u) / 1000u));
        oled_draw_text3x5(fb, x + 2, y + 26, line);
    } else {
        oled_draw_text3x5(fb, x + 2, y + 26, s_show.status);
    }

    if (s_show.count == 0) {
        oled_draw_text3x5(fb, x + 2, y + 38, "no shows in /shows");
        return;
    }

    const int list_y = y + 36;
    const int max_vis = 3;
    int selected_row = (int)s_show.selected;
    int total_rows = s_show.count + 2;
    int start = 0;
    if (selected_row >= max_vis) {
        start = selected_row - (max_vis - 1);
    }
    int visible = total_rows - start;
    if (visible > max_vis) visible = max_vis;

    for (int i = 0; i < visible; ++i) {
        int row = start + i;
        int yy = list_y + i * 8;
        const char *label = NULL;
        if (row == 0) {
            label = "Back";
        } else if (row == 1) {
            snprintf(line, sizeof(line), "Role:%s", show_role_name(s_show.role_id));
            label = line;
        } else {
            label = s_show.shows[row - 2];
        }

        char rowbuf[40];
        show_player_copy_row_label(rowbuf, sizeof(rowbuf),
                                   (row == selected_row) ? '>' : ' ',
                                   label);
        oled_draw_text3x5(fb, x + 2, yy, rowbuf);
    }

    (void)w;
}
