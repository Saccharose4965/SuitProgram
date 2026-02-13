#include "bt_audio_app.h"
#include "bt_audio.h"
#include "bt_audio_internal.h"

#include <stdio.h>
#include <string.h>

#include "oled.h"

typedef struct {
    bt_audio_device_t devs[8];
    int count;
    int sel;
    int start;
    bool peer_extra;
    int  peer_index;
} bt_ui_state_t;

static const int k_bt_list_max_vis = 4;

static bt_ui_state_t s_bt_ui = {0};
static bt_audio_state_t s_bt_prev_state = BT_AUDIO_STATE_DISABLED;

static int bt_find_dev_idx(const bt_audio_device_t *list, int count, const uint8_t bda[6])
{
    for (int i = 0; i < count; ++i) {
        if (memcmp(list[i].bda, bda, 6) == 0) return i;
    }
    return -1;
}

static void bt_update_viewport(void)
{
    const int max_vis = k_bt_list_max_vis;
    if (s_bt_ui.sel < 0) s_bt_ui.sel = 0;
    if (s_bt_ui.sel >= s_bt_ui.count) {
        s_bt_ui.sel = s_bt_ui.count ? s_bt_ui.count - 1 : 0;
    }

    int max_start = (s_bt_ui.count > max_vis) ? (s_bt_ui.count - max_vis) : 0;
    if (s_bt_ui.start < 0) s_bt_ui.start = 0;
    if (s_bt_ui.start > max_start) s_bt_ui.start = max_start;

    if (s_bt_ui.sel < s_bt_ui.start) {
        s_bt_ui.start = s_bt_ui.sel;
    } else if (s_bt_ui.sel >= s_bt_ui.start + max_vis) {
        s_bt_ui.start = s_bt_ui.sel - (max_vis - 1);
    }
}

static void bt_refresh_list(void)
{
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);

    s_bt_ui.count = bt_audio_get_devices(s_bt_ui.devs, 8);
    s_bt_ui.peer_extra = false;
    s_bt_ui.peer_index = -1;

    if ((st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING) && st.peer_bda[0] != 0) {
        int idx = bt_find_dev_idx(s_bt_ui.devs, s_bt_ui.count, st.peer_bda);
        if (idx < 0 && s_bt_ui.count < 8) {
            idx = s_bt_ui.count;
            memcpy(s_bt_ui.devs[idx].bda, st.peer_bda, 6);
            if (st.peer_name[0]) {
                strncpy(s_bt_ui.devs[idx].name, st.peer_name, sizeof(s_bt_ui.devs[idx].name) - 1);
                s_bt_ui.devs[idx].name[sizeof(s_bt_ui.devs[idx].name) - 1] = '\0';
            } else {
                s_bt_ui.devs[idx].name[0] = '\0';
            }
            s_bt_ui.peer_extra = true;
            s_bt_ui.peer_index = idx;
            s_bt_ui.count++;
        } else if (idx >= 0) {
            s_bt_ui.peer_index = idx;
        }
    }

    bt_update_viewport();
}

static void bt_make_label(const bt_audio_device_t *dev, char *out, size_t out_sz)
{
    if (!dev || !out || out_sz == 0) return;
    if (dev->name[0]) {
        strncpy(out, dev->name, out_sz - 1);
        out[out_sz - 1] = '\0';
    } else {
        snprintf(out, out_sz, "%02X:%02X:%02X", dev->bda[3], dev->bda[4], dev->bda[5]);
    }
}

void bt_audio_app_init(void)
{
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    // Only kick off a scan if we're not already connected/connecting/streaming.
    if (!(st.state == BT_AUDIO_STATE_CONNECTED ||
          st.state == BT_AUDIO_STATE_STREAMING ||
          st.state == BT_AUDIO_STATE_CONNECTING ||
          st.state == BT_AUDIO_STATE_DISCOVERING)) {
        (void)bt_audio_scan(); // show list immediately, no auto-connect
    }
}

void bt_audio_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;
    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) {           // up
            if (s_bt_ui.sel > 0) s_bt_ui.sel--;
        } else if (ev->button == INPUT_BTN_B) {    // down
            if (s_bt_ui.sel + 1 < s_bt_ui.count) s_bt_ui.sel++;
        } else if (ev->button == INPUT_BTN_C) {    // scan
            (void)bt_audio_scan();
            s_bt_ui.sel = 0;
            bt_refresh_list();
        } else if (ev->button == INPUT_BTN_D) {    // select/connect/disconnect
            bt_audio_status_t st = {0};
            bt_audio_get_status(&st);
            bool connected = (st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING);
            bool selecting_peer = (s_bt_ui.peer_index >= 0 && s_bt_ui.sel == s_bt_ui.peer_index);
            if (connected && selecting_peer) {
                (void)bt_audio_disconnect();
            } else {
                if (connected && !selecting_peer) {
                    (void)bt_audio_disconnect();
                }
                (void)bt_audio_connect_index(s_bt_ui.sel);
            }
        }
    }

    bt_update_viewport();
}

void bt_audio_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    (void)w;
    (void)h;
    bt_refresh_list();
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    if ((s_bt_prev_state == BT_AUDIO_STATE_CONNECTED || s_bt_prev_state == BT_AUDIO_STATE_STREAMING) &&
        !(st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING)) {
        bt_audio_invoke_disconnect_cb();
    }
    s_bt_prev_state = st.state;

    char line[48];
    snprintf(line, sizeof(line), "BT:%s", bt_audio_state_name(st.state));
    oled_draw_text3x5(fb, x + 2, y + 2, line);
    if (st.peer_name[0]) {
        snprintf(line, sizeof(line), "Conn:%s", st.peer_name);
        oled_draw_text3x5(fb, x + 2, y + 10, line);
    }

    const int max_vis = k_bt_list_max_vis;
    int start = s_bt_ui.start;
    int visible = s_bt_ui.count - start;
    if (visible > max_vis) visible = max_vis;
    if (visible < 0) visible = 0;

    int list_y = y + 18;
    if (s_bt_ui.count == 0) {
        oled_draw_text3x5(fb, x + 2, list_y, "Scanning...");
    } else {
        for (int i = 0; i < visible; ++i) {
            int idx = start + i;
            int yy = list_y + i * 8;
            char dev_label[24];
            bt_make_label(&s_bt_ui.devs[idx], dev_label, sizeof(dev_label));
            bool is_current = (memcmp(s_bt_ui.devs[idx].bda, st.peer_bda, sizeof(st.peer_bda)) == 0) &&
                              (st.state == BT_AUDIO_STATE_CONNECTED ||
                               st.state == BT_AUDIO_STATE_STREAMING ||
                               st.state == BT_AUDIO_STATE_CONNECTING);
            snprintf(line, sizeof(line), "%c%s %s",
                     (idx == s_bt_ui.sel) ? '>' : ' ',
                     is_current ? "[*]" : "[ ]",
                     dev_label);
            oled_draw_text3x5(fb, x + 2, yy, line);
        }
    }
}
