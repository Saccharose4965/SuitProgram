#include "app_shell.h"
#include "ui_draw.h"
#include "audio_policy.h"

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
    size_t total = (s_show.count > 0) ? (size_t)(s_show.count + 3) : 3u;
    if (s_show.selected >= total) {
        s_show.selected = total - 1u;
    }
}

static void show_player_scan(void)
{
    s_show.count = 0;

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
    if (s_show.selected < 3u) return -1;
    int idx = (int)(s_show.selected - 3u);
    if (idx < 0 || idx >= s_show.count) return -1;
    return idx;
}

void show_player_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    audio_policy_fft_set_show_app(true);
    show_runtime_set_menu_open(true);
    uint8_t saved_role = show_runtime_get_status().role_id;
    memset(&s_show, 0, sizeof(s_show));
    s_show.role_id = saved_role % SHOW_MAX_ROLES;
    s_show.selected = 3;

    esp_err_t err = storage_mount_sd();
    if (err == ESP_OK) {
        s_show.mounted = true;
        show_player_scan();
    } else {
        s_show.mounted = false;
        show_player_set_status("sd fail");
        ESP_LOGW(TAG, "storage_mount_sd failed: %s", esp_err_to_name(err));
    }
}

void show_player_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    s_show.role_id = show_runtime_get_status().role_id;

    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) {
            if (s_show.selected > 0u) {
                s_show.selected--;
            }
            return;
        }
        if (ev->button == INPUT_BTN_B) {
            size_t total = (s_show.count > 0) ? (size_t)(s_show.count + 3) : 3u;
            if (s_show.selected + 1u < total) {
                s_show.selected++;
            }
            return;
        }
        if (ev->button == INPUT_BTN_C) {
            if (s_show.selected == 2u) {
                uint16_t delay = show_runtime_get_bt_delay_ms();
                show_runtime_set_bt_delay_ms(delay >= 25 ? delay - 25 : 0);
                return;
            }
            if (show_runtime_owns_leds() && s_show.selected!=1u) {
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
            show_player_set_status(err == ESP_OK ? "role requested" : "roles busy");
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
                show_player_set_status(err == ESP_OK ? "role requested" : "roles busy");
                return;
            }
            if (s_show.selected == 2u) {
                show_runtime_set_bt_delay_ms(show_runtime_get_bt_delay_ms() + 25);
                return;
            }
            int show_idx = show_player_selected_show_index();
            if (show_idx >= 0) {
                show_runtime_status_t runtime = show_runtime_get_status();
                if (!runtime.role_ready || runtime.role_pending) {
                    show_player_set_status("finding roles");
                    return;
                }
                const char *slug = s_show.shows[show_idx];
                esp_err_t err;
                if (runtime.owns_leds && runtime.authority &&
                    strcmp(runtime.slug, slug) == 0) {
                    err = show_runtime_toggle_master_pause();
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
            s_show.mounted = storage_mount_sd() == ESP_OK;
            if (!s_show.mounted) { show_player_set_status("sd fail"); return; }
            show_player_scan();
        }
    }
}

void show_player_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    show_runtime_set_menu_open(false);
    audio_policy_fft_set_show_app(false);
}

void show_player_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb || w < 16 || h < 5) return;
    show_runtime_status_t rt = show_runtime_get_status();
    s_show.role_id = rt.role_id;
    char line[40];
    // 3x5 font: four pixels wide including spacing; stay within content bounds.
    size_t cols = (size_t)(w - 4) / 4;
    if (cols >= sizeof(line)) cols = sizeof(line) - 1;
    snprintf(line, sizeof(line), "%c:%s A%c B%c C%c",
             rt.role_ready ? 'A' + rt.role_id : '?', rt.authority ? "lead" : (rt.owns_leds ? "follow" : "ready"),
             rt.master_roles & 1 ? '*' : (rt.present_roles & 1 ? '+' : '-'),
             rt.master_roles & 2 ? '*' : (rt.present_roles & 2 ? '+' : '-'),
             rt.master_roles & 4 ? '*' : (rt.present_roles & 4 ? '+' : '-'));
    line[cols] = 0;
    oled_draw_text3x5(fb, x + 2, y, line);
    if (h < 13) return;
    if (!s_show.mounted) snprintf(line, sizeof(line), "SD not mounted");
    else if (!rt.role_ready || rt.role_pending) snprintf(line, sizeof(line), "Finding roles...");
    else if (rt.error[0]) snprintf(line, sizeof(line), "%s", rt.error);
    else if (rt.role_conflict) snprintf(line, sizeof(line), "Role conflict: press C");
    else if (rt.owns_leds) snprintf(line, sizeof(line), "%s %lus", show_runtime_state_name(rt.state),
                                   (unsigned long)(rt.playhead_ms / 1000));
    else snprintf(line, sizeof(line), "%s", s_show.count ? "D:play C:role" : "No /shows packages");
    line[cols] = 0;
    oled_draw_text3x5(fb, x + 2, y + 8, line);

    const int list_top = 16, row_h = 8;
    int visible = h >= list_top + 5 ? 1 + (h - list_top - 5) / row_h : 0;
    int total = s_show.count + 3;
    if (visible > total) visible = total;
    if (!visible) return;
    int start = (int)s_show.selected - visible + 1;
    if (start < 0) start = 0;
    for (int i = 0; i < visible && start + i < total; ++i) {
        int row = start + i;
        char label[32];
        if (row == 0) snprintf(label, sizeof(label), "Back");
        else if (row == 1) snprintf(label, sizeof(label), "Role %s", show_role_name(rt.role_id));
        else if (row == 2) snprintf(label, sizeof(label), "BT delay %ums", show_runtime_get_bt_delay_ms());
        else snprintf(label, sizeof(label), "%s", s_show.shows[row - 3]);
        show_player_copy_row_label(line, cols + 1, row == (int)s_show.selected ? '>' : ' ', label);
        oled_draw_text3x5(fb, x + 2, y + list_top + i * row_h, line);
    }
}
