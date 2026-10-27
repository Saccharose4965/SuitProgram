#include "remote_control_app.h"

#include <stdio.h>
#include <string.h>

#include "oled.h"
#include "remote_control.h"
#include "shell_ui.h"
#include "ui_draw.h"

shell_legend_t REMOTE_CONTROL_LEGEND = {
    .slots = {SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_SELECT},
};

static size_t s_selected;
static shell_menu_motion_t s_peer_motion;
static shell_menu_motion_t s_remote_menu_motion;
static shell_grid_motion_t s_remote_grid_motion;
static uint8_t s_last_first;
static remote_control_view_kind_t s_last_kind;

static size_t clamp_selection(size_t selected, size_t count)
{
    if (!count) return 0;
    return selected < count ? selected : count - 1u;
}

void remote_control_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_selected = 0;
    s_last_kind = REMOTE_CONTROL_VIEW_NONE;
    shell_ui_menu_reset(&s_peer_motion, 0);
    REMOTE_CONTROL_LEGEND = (shell_legend_t){
        .slots={SHELL_ICON_UP,SHELL_ICON_DOWN,SHELL_ICON_NONE,SHELL_ICON_SELECT},
    };
}

void remote_control_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    if (remote_control_role() == REMOTE_CONTROL_CONTROLLER) remote_control_end();
}

void remote_control_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (remote_control_role() != REMOTE_CONTROL_CONTROLLER) {
        shell_ui_menu_tick(&s_peer_motion, dt_sec, s_selected);
        return;
    }
    remote_control_view_t view;
    if (!remote_control_get_view(&view)) return;
    size_t local = view.selected >= view.first ? view.selected - view.first : 0;
    local = clamp_selection(local, view.entry_count);
    if (view.kind != s_last_kind || view.first != s_last_first) {
        shell_ui_menu_reset(&s_remote_menu_motion, local);
        shell_ui_grid_reset(&s_remote_grid_motion, local);
    } else if (view.kind == REMOTE_CONTROL_VIEW_MENU) {
        shell_ui_menu_tick(&s_remote_menu_motion, dt_sec, local);
    } else if (view.kind == REMOTE_CONTROL_VIEW_GRID) {
        shell_ui_grid_tick(&s_remote_grid_motion, dt_sec, local);
    }
    s_last_kind = view.kind;
    s_last_first = view.first;
}

void remote_control_app_handle_input(shell_app_context_t *ctx,
                                     const input_event_t *event)
{
    (void)ctx;
    if (!event || event->type != INPUT_EVENT_PRESS) return;
    if (remote_control_role() == REMOTE_CONTROL_CONTROLLER) {
        if (event->button >= INPUT_BTN_A && event->button <= INPUT_BTN_D)
            (void)remote_control_send_input(event);
        return;
    }

    remote_control_peer_t peers[REMOTE_CONTROL_MAX_PEERS];
    size_t count = remote_control_get_peers(peers, REMOTE_CONTROL_MAX_PEERS);
    s_selected = clamp_selection(s_selected, count);
    if (event->button == INPUT_BTN_A && s_selected > 0) s_selected--;
    else if (event->button == INPUT_BTN_B && s_selected + 1u < count) s_selected++;
    else if (event->button == INPUT_BTN_D && count) {
        if (remote_control_begin(peers[s_selected].mac) == ESP_OK) {
            s_last_kind = REMOTE_CONTROL_VIEW_NONE;
        }
    }
}

static void draw_peer_picker(uint8_t *fb, int x, int y, int w, int h)
{
    remote_control_peer_t peers[REMOTE_CONTROL_MAX_PEERS];
    size_t count = remote_control_get_peers(peers, REMOTE_CONTROL_MAX_PEERS);
    if (!count) {
        oled_draw_text3x5(fb, x+4, y+4, "REMOTE CONTROL");
        oled_draw_text3x5(fb, x+4, y+16, "Waiting for another suit");
        return;
    }
    shell_menu_entry_t entries[REMOTE_CONTROL_MAX_PEERS];
    char labels[REMOTE_CONTROL_MAX_PEERS][20];
    for (size_t i=0; i<count; ++i) {
        snprintf(labels[i], sizeof(labels[i]), "Suit %02X:%02X:%02X",
                 peers[i].mac[3], peers[i].mac[4], peers[i].mac[5]);
        entries[i] = (shell_menu_entry_t){.id=labels[i], .label=labels[i]};
    }
    s_selected = clamp_selection(s_selected, count);
    shell_menu_view_t menu = {.entries=entries, .count=count, .selected=s_selected,
                              .title="REMOTE CONTROL", .motion=&s_peer_motion};
    shell_ui_draw_menu(fb, x, y, w, h, &menu);
}

static void draw_remote_view(uint8_t *fb, int x, int y, int w, int h,
                             const remote_control_view_t *view)
{
    REMOTE_CONTROL_LEGEND = (shell_legend_t){0};
    for (unsigned i=0; i<4; ++i) {
        if (view->legend[i] <= SHELL_ICON_STOP)
            REMOTE_CONTROL_LEGEND.slots[i] = (shell_icon_id_t)view->legend[i];
    }
    size_t count = view->entry_count;
    size_t local = view->selected >= view->first ? view->selected-view->first : 0;
    local = clamp_selection(local, count);
    if (view->kind == REMOTE_CONTROL_VIEW_NONE || !count) {
        char line[32];
        oled_draw_text3x5(fb, x+4, y+4, "REMOTE APP");
        snprintf(line, sizeof(line), "%s", view->app_id[0] ? view->app_id : "unknown");
        oled_draw_text3x5(fb, x+4, y+16, line);
        oled_draw_text3x5(fb, x+4, y+28, "Custom view not mirrored");
        return;
    }
    shell_menu_entry_t entries[REMOTE_CONTROL_MAX_ENTRIES];
    for (size_t i=0; i<count; ++i)
        entries[i] = (shell_menu_entry_t){.id=view->labels[i], .label=view->labels[i]};
    if (view->kind == REMOTE_CONTROL_VIEW_GRID) {
        shell_grid_view_t grid = {.entries=entries, .count=count, .selected=local,
                                  .motion=&s_remote_grid_motion};
        shell_ui_draw_grid(fb, x, y, w, h, &grid);
    } else {
        shell_menu_view_t menu = {.entries=entries, .count=count, .selected=local,
                                  .title=view->title, .motion=&s_remote_menu_motion};
        shell_ui_draw_menu(fb, x, y, w, h, &menu);
    }
}

void remote_control_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                             int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;
    remote_control_view_t view;
    if (remote_control_role() == REMOTE_CONTROL_CONTROLLER) {
        if (remote_control_get_view(&view)) draw_remote_view(fb,x,y,w,h,&view);
        else {
            oled_draw_text3x5(fb,x+4,y+4,"REMOTE CONTROL");
            oled_draw_text3x5(fb,x+4,y+16,"Opening target menu...");
        }
        shell_ui_draw_remote_corners(fb, true);
        return;
    }
    draw_peer_picker(fb,x,y,w,h);
}
