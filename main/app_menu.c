#include "app_shell.h"

#include <string.h>

#include "esp_system.h"

typedef shell_menu_entry_t menu_entry_t;

typedef enum {
    MENU_ROOT = 0,
    MENU_SETTINGS,
    MENU_SIM,
    MENU_GAMES,
    MENU_COMM,
    MENU_LED,
} menu_page_t;

typedef struct {
    size_t selected;
    menu_page_t page;
} menu_state_t;

static menu_state_t g_menu_state = { .selected = 0, .page = MENU_ROOT };

static const menu_entry_t g_menu_root_entries[] = {
    { "menu_settings", "Settings"    },
    { "menu_games",    "Games"       },
    { "menu_sim",      "Simulations" },
    { "menu_comm",     "Comm"        },
    { "menu_led",      "LEDs"        },
    { "music",         "Music"       },
};
static const menu_entry_t g_menu_settings_entries[] = {
    { "menu_root", "Back"      },
    { "status",    "Status"    },
    { "bt",        "Bluetooth" },
    { "volume",    "Volume"    },
    { "keyboard",  "Keyboard"  },
    { "reboot",    "Restart"   },
};
static const menu_entry_t g_menu_sim_entries[] = {
    { "menu_root", "Back"   },
    { "gps",       "GPS"    },
    { "fft",       "FFT"    },
    { "fluid",     "Fluid"  },
    { "threedee",  "3D Render" },
    { "bad_apple", "Bad Apple" },
};
static const menu_entry_t g_menu_games_entries[] = {
    { "menu_root", "Back"   },
    { "flappy",    "Flappy" },
    { "t2048",     "2048"   },
    { "tetris",    "Tetris" },
    { "pong",      "Pong"   },
    { "snake",     "Snake"  },
};
static const menu_entry_t g_menu_comm_entries[] = {
    { "menu_root", "Back"      },
    { "bt",        "Bluetooth" },
    { "message",   "Message"   },
    { "call",      "Call"      },
    { "file_rx",   "File RX"   },
};
static const menu_entry_t g_menu_led_entries[] = {
    { "menu_root",   "Back"           },
    { "leds_audio",  "Audio Reactive" },
    { "leds_custom", "Custom"         },
};

static void menu_page_info(const menu_state_t *st, const menu_entry_t **entries, size_t *count, const char **title)
{
    if (entries) *entries = NULL;
    if (count)   *count = 0;
    if (title)   *title = "MENU";
    if (!st) return;

    switch (st->page){
        case MENU_ROOT:
            if (entries) *entries = g_menu_root_entries;
            if (count)   *count   = sizeof(g_menu_root_entries)/sizeof(g_menu_root_entries[0]);
            if (title)   *title   = "MAIN";
            break;
        case MENU_SETTINGS:
            if (entries) *entries = g_menu_settings_entries;
            if (count)   *count   = sizeof(g_menu_settings_entries)/sizeof(g_menu_settings_entries[0]);
            if (title)   *title   = "SETTINGS";
            break;
        case MENU_SIM:
            if (entries) *entries = g_menu_sim_entries;
            if (count)   *count   = sizeof(g_menu_sim_entries)/sizeof(g_menu_sim_entries[0]);
            if (title)   *title   = "SIM";
            break;
        case MENU_GAMES:
            if (entries) *entries = g_menu_games_entries;
            if (count)   *count   = sizeof(g_menu_games_entries)/sizeof(g_menu_games_entries[0]);
            if (title)   *title   = "GAMES";
            break;
        case MENU_COMM:
            if (entries) *entries = g_menu_comm_entries;
            if (count)   *count   = sizeof(g_menu_comm_entries)/sizeof(g_menu_comm_entries[0]);
            if (title)   *title   = "COMM";
            break;
        case MENU_LED:
            if (entries) *entries = g_menu_led_entries;
            if (count)   *count   = sizeof(g_menu_led_entries)/sizeof(g_menu_led_entries[0]);
            if (title)   *title   = "LED";
            break;
        default:
            break;
    }
}

const char *menu_hud_label(void)
{
    switch (g_menu_state.page){
        case MENU_ROOT:     return "/main";
        case MENU_SETTINGS: return "/settings";
        case MENU_SIM:      return "/sim";
        case MENU_GAMES:    return "/games";
        case MENU_COMM:     return "/comm";
        case MENU_LED:      return "/led";
        default:            return "/menu";
    }
}

void menu_init(shell_app_context_t *ctx)
{
    (void)ctx;
    g_menu_state.page = MENU_ROOT;
    g_menu_state.selected = 0;
    shell_ui_menu_reset(g_menu_state.selected);
}

void menu_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    const menu_entry_t *entries = NULL;
    size_t count = 0;
    menu_page_info(&g_menu_state, &entries, &count, NULL);
    if (!entries || count == 0) return;

    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) { // up
            if (g_menu_state.selected > 0) g_menu_state.selected--;
        } else if (ev->button == INPUT_BTN_B) { // down
            if (g_menu_state.selected + 1 < count) g_menu_state.selected++;
        } else if (ev->button == INPUT_BTN_D) { // select
            const char *id = entries[g_menu_state.selected].id;
            if (strcmp(id, "menu_root") == 0) {
                g_menu_state.page = MENU_ROOT;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if (strcmp(id, "menu_settings") == 0) {
                g_menu_state.page = MENU_SETTINGS;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if(strcmp(id, "menu_sim") == 0) {
                g_menu_state.page = MENU_SIM;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if (strcmp(id, "menu_games") == 0) {
                g_menu_state.page = MENU_GAMES;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if (strcmp(id, "menu_comm") == 0) {
                g_menu_state.page = MENU_COMM;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if (strcmp(id, "menu_led") == 0) {
                g_menu_state.page = MENU_LED;
                g_menu_state.selected = 0;
                shell_ui_menu_reset(g_menu_state.selected);
            } else if (strcmp(id, "reboot") == 0) {
                esp_restart();
            } else {
                ctx->request_switch(id, ctx->request_user_data);
            }
        }
    }
}

void menu_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    shell_ui_menu_tick(dt_sec, g_menu_state.selected);
}

void menu_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)w; (void)h;
    if (!ctx || !fb) return;

    const menu_entry_t *entries = NULL;
    size_t count = 0;
    const char *title = NULL;
    menu_page_info(&g_menu_state, &entries, &count, &title);
    if (!entries || count == 0) return;

    if (g_menu_state.selected >= count) {
        g_menu_state.selected = count - 1;
        shell_ui_menu_reset(g_menu_state.selected);
    }

    shell_menu_view_t view = {
        .entries  = entries,
        .count    = count,
        .selected = g_menu_state.selected,
        .title    = title,
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}

const shell_legend_t MENU_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_CUSTOM2 },
};
