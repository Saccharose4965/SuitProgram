#include "app_shell.h"
#include "shell_apps.h"
#include "shell_audio.h"
#include "shell_profiler.h"
#include "app_settings.h"
#include "audio.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "hw.h"
#include "oled.h"
#include "gps.h"
#include "led_modes.h"
#include "power.h"
#include "link.h"
#include "bt_audio.h"
#include "orientation_service.h"
#include "pong.h"

static const char *TAG = "shell";

#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#else
#define APP_TASK_CORE 0
#endif

static DRAM_ATTR uint8_t s_fb[PANEL_W * PANEL_H / 8];
static DRAM_ATTR uint8_t s_fb_oled_a[PANEL_W * PANEL_H / 8];
static DRAM_ATTR uint8_t s_fb_oled_b[PANEL_W * PANEL_H / 8];
static uint8_t *s_fb_oled_latest = s_fb_oled_a;
static TaskHandle_t s_oled_task = NULL;
static TaskHandle_t s_startup_tone_task = NULL;

static size_t s_current_app = 0;
static size_t s_prev_app = 0;
static size_t s_queued_app_index = 0;
static bool   s_app_switch_queued = false;

static shell_app_context_t s_ctx = {0};

static void link_frame_handler(link_msg_type_t type, const uint8_t *payload, size_t len, void *user_ctx);
static uint8_t s_peer_mac_cfg[6] = {0};

static int hex_nib(char c){
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return -1;
}

static bool parse_mac_string(const char *s, uint8_t out[6]){
    if (!s || !out) return false;
    uint8_t tmp[6] = {0};
    int nibbles = 0;
    for (const char *p = s; *p; ++p){
        if (*p == ':' || *p == '-' || *p == ' ') continue;
        int v = hex_nib(*p);
        if (v < 0) return false;
        int byte_idx = nibbles / 2;
        if (byte_idx >= 6) return false;
        if (nibbles % 2 == 0) tmp[byte_idx] = (uint8_t)(v << 4);
        else tmp[byte_idx] |= (uint8_t)v;
        nibbles++;
    }
    if (nibbles != 12) return false;
    memcpy(out, tmp, 6);
    return true;
}

// ======================================================================
// Framebuffer helpers
// ======================================================================

static inline void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void oled_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t *buf = s_fb_oled_latest;
        if (buf) {
            oled_blit_full(buf);
        }
    }
}

static void startup_tone_task(void *arg)
{
    (void)arg;
    // Let shell init continue; play tone asynchronously.
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t tone_err = audio_play_tone(440, 1000);
    if (tone_err != ESP_OK) {
        ESP_LOGW(TAG, "startup tone failed: %s", esp_err_to_name(tone_err));
    } else {
        ESP_LOGI(TAG, "startup tone played");
    }
    s_startup_tone_task = NULL;
    vTaskDelete(NULL);
}

static inline void oled_submit_frame(void)
{
    if (!s_oled_task) {
        oled_blit_full(s_fb);
        return;
    }
    uint8_t *target = (s_fb_oled_latest == s_fb_oled_a) ? s_fb_oled_b : s_fb_oled_a;
    memcpy(target, s_fb, sizeof(s_fb));
    s_fb_oled_latest = target;
    xTaskNotifyGive(s_oled_task);
}

// ======================================================================
// App helpers
// ======================================================================

static const shell_app_desc_t *current_app(void)
{
    if (!s_ctx.registry || s_ctx.registry_count == 0 || s_current_app >= s_ctx.registry_count) return NULL;
    return &s_ctx.registry[s_current_app];
}

static const shell_app_desc_t *find_app_by_id(const char *id, size_t *out_index)
{
    if (!s_ctx.registry || !id) return NULL;
    for (size_t i = 0; i < s_ctx.registry_count; ++i) {
        if (s_ctx.registry[i].id && strcmp(s_ctx.registry[i].id, id) == 0) {
            if (out_index) *out_index = i;
            return &s_ctx.registry[i];
        }
    }
    return NULL;
}

static void switch_to_index(size_t idx)
{
    if (!s_ctx.registry || idx >= s_ctx.registry_count) return;

    const shell_app_desc_t *old = current_app();
    if (old && old->deinit) {
        old->deinit(&s_ctx);
    }

    s_prev_app = s_current_app;
    s_current_app = idx;
    s_ctx.current_index = idx;

    const shell_app_desc_t *app = current_app();
    if (app && app->init) {
        app->init(&s_ctx);
    }
    ESP_LOGI(TAG, "switched to app [%s]", app ? app->id : "none");
}

static void queue_app_switch_request(const char *id, void *user_data)
{
    (void)user_data;
    size_t idx = 0;
    if (find_app_by_id(id, &idx)) {
        s_queued_app_index = idx;
        s_app_switch_queued = true;
    } else {
        ESP_LOGW(TAG, "request to unknown app '%s'", id ? id : "(null)");
    }
}

static void apply_queued_app_switch(void)
{
    if (!s_app_switch_queued) return;
    s_app_switch_queued = false;
    if (s_queued_app_index == s_current_app) return;
    switch_to_index(s_queued_app_index);
}

// Link frame dispatch (shared across apps)
// we may want to split this out into a separate component
static void link_frame_handler(link_msg_type_t type, const uint8_t *payload, size_t len, void *user_ctx)
{
    (void)user_ctx;
    if (type == LINK_MSG_GAME && payload && len >= 2){
        pong_handle_link_frame(type, payload, len);
    } else if (type == LINK_MSG_INFO && payload && len >= 9){
        // Versioned system_state snapshot
        uint8_t ver = payload[0];
        if (ver == 1 && len >= 9){
            uint8_t led_mode = payload[1];
            uint8_t b0 = payload[2], b1 = payload[3], b2 = payload[4];
            system_connection_t conn = (system_connection_t)payload[5];
            bool t_valid = payload[6] ? true : false;
            uint8_t h = payload[7], m = payload[8];
            system_state_set_led_mode((int)led_mode, NULL);
            system_state_set_battery(0, b0);
            system_state_set_battery(1, b1);
            system_state_set_battery(2, b2);
            system_state_set_connection(conn);
            system_state_set_time(t_valid, h, m);
        }
    }
}

// ======================================================================
// Built-in apps
// ======================================================================

static const shell_app_desc_t s_builtin_apps[] = {
    {
        .id     = "title",
        .name   = "Title",
        .flags  = 0,
        .legend = &MENU_LEGEND,
        .init   = NULL,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = NULL,
        .draw   = NULL, // blank title screen for now
    },
    {
        .id     = "menu",
        .name   = "Menu",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &MENU_LEGEND,
        .init   = menu_init,
        .deinit = NULL,
        .tick   = menu_tick,
        .handle_input = menu_handle_input,
        .draw   = menu_draw,
    },
    {
        .id     = "status",
        .name   = "Status",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &STATUS_LEGEND,
        .init   = NULL,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = status_handle_input,
        .draw   = status_draw,
    },
    {
        .id     = "volume",
        .name   = "Volume",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &VOLUME_LEGEND,
        .init   = volume_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = volume_handle_input,
        .draw   = volume_draw,
    },
    {
        .id     = "service_restart",
        .name   = "Service Restart",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &SERVICE_RESTART_LEGEND,
        .init   = service_restart_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = service_restart_app_handle_input,
        .draw   = service_restart_app_draw,
    },
    {
        .id     = "adc_debug",
        .name   = "ADC Debug",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &ADC_DEBUG_LEGEND,
        .init   = adc_debug_app_init,
        .deinit = NULL,
        .tick   = adc_debug_app_tick,
        .handle_input = adc_debug_app_handle_input,
        .draw   = adc_debug_app_draw,
    },
    {
        .id     = "calculator",
        .name   = "Calculator",
        .flags  = SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &CALCULATOR_LEGEND,
        .init   = calculator_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = calculator_app_handle_input,
        .draw   = calculator_app_draw,
    },
    {
        .id     = "leds_audio",
        .name   = "LED Audio",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &LEDS_LEGEND,
        .init   = leds_audio_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = leds_app_handle_input,
        .draw   = leds_app_draw,
    },
    {
        .id     = "leds_custom",
        .name   = "LED Custom",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &LEDS_LEGEND,
        .init   = leds_custom_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = leds_app_handle_input,
        .draw   = leds_app_draw,
    },
    {
        .id     = "music",
        .name   = "Music",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &MUSIC_LEGEND,
        .init   = music_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = music_app_handle_input,
        .draw   = music_app_draw,
    },
    {
        .id     = "bt",
        .name   = "Bluetooth",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &BT_AUDIO_LEGEND,
        .init   = bt_app_init_wrapper,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = bt_handle_input_wrapper,
        .draw   = bt_draw_wrapper,
    },
    {
        .id     = "file_rx",
        .name   = "File RX",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &FILE_RX_LEGEND,
        .init   = file_rx_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = file_rx_app_handle_input,
        .draw   = file_rx_app_draw,
    },
    {
        .id     = "keyboard",
        .name   = "Keyboard",
        .flags  = 0,
        .legend = &KEYBOARD_LEGEND,
        .init   = keyboard_app_init,
        .deinit = NULL,
        .tick   = keyboard_app_tick,
        .handle_input = keyboard_app_handle_input,
        .draw   = keyboard_app_draw,
    },
    {
        .id     = "fft",
        .name   = "FFT viewer",
        .flags  = 0,
        .legend = &FFT_LEGEND,
        .init   = fft_app_init,
        .deinit = fft_app_deinit,
        .tick   = NULL,
        .handle_input = fft_stub_handle_input,
        .draw   = fft_app_draw,
    },
    {
        .id     = "fluid",
        .name   = "Fluid",
        .flags  = 0,
        .legend = &FLUID_LEGEND,
        .init   = fluid_app_init_wrapper,
        .deinit = fluid_app_deinit_wrapper,
        .tick   = fluid_app_tick_wrapper,
        .handle_input = fluid_app_handle_input_wrapper,
        .draw   = fluid_app_draw_wrapper,
    },
    {
        .id     = "threedee",
        .name   = "3D Render",
        .flags  = SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &THREEDEE_LEGEND,
        .init   = threedee_app_init_wrapper,
        .deinit = threedee_app_deinit_wrapper,
        .tick   = threedee_tick_wrapper,
        .handle_input = threedee_handle_input_wrapper,
        .draw   = threedee_draw_wrapper,
    },
    {
        .id     = "bad_apple",
        .name   = "Bad Apple",
        .flags  = 0,
        .legend = &BAD_APPLE_LEGEND,
        .init   = bad_apple_app_init,
        .deinit = bad_apple_app_deinit,
        .tick   = bad_apple_app_tick,
        .handle_input = bad_apple_app_handle_input,
        .draw   = bad_apple_app_draw,
    },
    {
        .id     = "pong",
        .name   = "Pong",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &PONG_LEGEND,
        .init   = pong_app_init_wrapper,
        .deinit = NULL,
        .tick   = pong_tick_wrapper,
        .handle_input = pong_handle_input_wrapper,
        .draw   = pong_draw_wrapper,
    },
    {
        .id     = "snake",
        .name   = "Snake",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &SNAKE_LEGEND,
        .init   = snake_app_init_wrapper,
        .deinit = NULL,
        .tick   = snake_tick_wrapper,
        .handle_input = snake_handle_input_wrapper,
        .draw   = snake_draw_wrapper,
    },
    {
        .id     = "flappy",
        .name   = "Flappy (legacy)",
        .flags  = 0,
        .legend = &FLAPPY_LEGEND,
        .init   = flappy_app_init,
        .deinit = flappy_app_deinit,
        .tick   = NULL,
        .handle_input = flappy_stub_handle_input,
        .draw   = flappy_draw_wrapper,
    },
    {
        .id     = "t2048",
        .name   = "2048 (legacy)",
        .flags  = 0,
        .legend = &T2048_LEGEND,
        .init   = t2048_app_init,
        .deinit = t2048_app_deinit,
        .tick   = NULL,
        .handle_input = t2048_stub_handle_input,
        .draw   = t2048_draw_wrapper,
    },
    {
        .id     = "tetris",
        .name   = "Tetris",
        .flags  = 0,
        .legend = &TETRIS_LEGEND,
        .init   = tetris_app_init,
        .deinit = tetris_app_deinit,
        .tick   = NULL,
        .handle_input = NULL,
        .draw   = tetris_draw_wrapper,
    },
};

// ======================================================================
// Shell loop
// ======================================================================

static void process_input_events(const shell_app_desc_t *app, TickType_t now_ticks)
{
    input_event_t ev;
    while (input_poll(&ev, now_ticks)) {
        // Any press on the title screen jumps to menu
        if (ev.type == INPUT_EVENT_PRESS && (!app || (app->id && strcmp(app->id, "title") == 0))) {
            queue_app_switch_request("menu", NULL);
            continue;
        }
        // Global escape to menu
        if (ev.type == INPUT_EVENT_LONG_PRESS) {
            if (ev.button == INPUT_BTN_AB_COMBO) {
                queue_app_switch_request("menu", NULL);
                continue;
            } else if (ev.button == INPUT_BTN_BC_COMBO) {
                esp_restart();
                continue;
            }
        }
        if (app && app->handle_input) {
            app->handle_input(&s_ctx, &ev);
        }
        apply_queued_app_switch();
        app = current_app();
        if (!app) break;
    }
}

static bool shell_init_hw_and_display(void)
{
    ESP_LOGI(TAG, "stage: start");
    esp_err_t spi2 = hw_spi2_init_once();
    if (spi2 != ESP_OK) {
        ESP_LOGE(TAG, "spi2 init failed: %s", esp_err_to_name(spi2));
        return false;
    }
    hw_gpio_init();
    oled_init();
    oled_clear();
    if (!s_oled_task) {
        xTaskCreatePinnedToCore(oled_task, "oled", 2048, NULL, 2, &s_oled_task, APP_TASK_CORE);
    }

    // Audio bus for FFT (shared RX/TX)
    if (!shell_audio_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT won't run");
    } else {
        // Startup speaker sanity check (async so shell task does not trip WDT).
        if (!s_startup_tone_task) {
            BaseType_t ok = xTaskCreatePinnedToCore(
                startup_tone_task,
                "tone_boot",
                3072,
                NULL,
                1,
                &s_startup_tone_task,
                APP_TASK_CORE
            );
            if (ok != pdPASS) {
                s_startup_tone_task = NULL;
                ESP_LOGW(TAG, "startup tone task create failed");
            }
        }
    }

    esp_err_t ori = orientation_service_start();
    if (ori != ESP_OK) {
        ESP_LOGW(TAG, "orientation service not started: %s", esp_err_to_name(ori));
    }

    return true;
}

static void shell_setup_link(void)
{
    const char *mac_str = CONFIG_LINK_ESPNOW_PEER_MAC;
    const uint8_t *peer_mac = NULL;
    if (mac_str && mac_str[0] && parse_mac_string(mac_str, s_peer_mac_cfg)){
        peer_mac = s_peer_mac_cfg;
    }
    link_config_t link_cfg = {
        .ssid   = CONFIG_LINK_WIFI_SSID,
        .pass   = CONFIG_LINK_WIFI_PASS,
        .pc_ip  = CONFIG_LINK_WIFI_PC_IP,
        .pc_port= CONFIG_LINK_WIFI_PC_PORT,
        .espnow_peer_mac = peer_mac,
    };
    bool link_ok = false;
    esp_err_t link_err = link_init(&link_cfg);
    if (link_err != ESP_OK) {
        ESP_LOGW(TAG, "link init failed: %s", esp_err_to_name(link_err));
    } else {
        link_ok = true;
    }
    if (link_ok) {
        (void)link_set_frame_rx(link_frame_handler, NULL);
        (void)link_start_info_broadcast(1000);
    }
}

static void shell_seed_initial_system_state(void)
{
    // Seed HUD with something legible until services wire in real data.
    {
        int led_mode = led_modes_current();
        const char *label = led_modes_name(led_mode);
        system_state_set_led_mode(led_mode, label ? label : "led");
    }
    system_state_set_battery(0, 80);
    system_state_set_battery(1, 65);
    system_state_set_battery(2, 50);
    system_state_set_connection(SYS_CONN_CONNECTING);
}

static void shell_init_input_and_apps(void)
{
    input_init();

    bt_audio_set_disconnect_cb(music_stop_playback);

    s_ctx.registry = s_builtin_apps;
    s_ctx.registry_count = sizeof(s_builtin_apps) / sizeof(s_builtin_apps[0]);
    s_ctx.request_switch = queue_app_switch_request;
    s_ctx.request_user_data = NULL;

    // Boot straight into the menu (skip blank title screen)
    size_t menu_idx = 0;
    if (!find_app_by_id("menu", &menu_idx)) {
        menu_idx = 0;
    }
    s_current_app = menu_idx;
    s_prev_app = menu_idx;
    switch_to_index(menu_idx);
}

static void shell_run_loop(void)
{
    const TickType_t frame_period = pdMS_TO_TICKS(33);
    const uint32_t frame_budget_us = (uint32_t)frame_period * (uint32_t)portTICK_PERIOD_MS * 1000u;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tick = last_wake;
    ESP_LOGI(TAG, "stage: enter loop");

    while (1) {
        int64_t frame_work_start_us = esp_timer_get_time();
        uint32_t tick_us = 0;
        uint32_t draw_us = 0;

        TickType_t now = xTaskGetTickCount();
        float dt_sec = (float)(now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        last_tick = now;

        const shell_app_desc_t *app = current_app();
        process_input_events(app, now);
        apply_queued_app_switch();
        app = current_app();
        if (!app) {
            uint32_t work_us = (uint32_t)(esp_timer_get_time() - frame_work_start_us);
            shell_profiler_on_frame("none", false, frame_budget_us, 0, 0, work_us);
            vTaskDelayUntil(&last_wake, frame_period);
            continue;
        }

        if (app->tick) {
            int64_t t0 = esp_timer_get_time();
            app->tick(&s_ctx, dt_sec);
            tick_us = (uint32_t)(esp_timer_get_time() - t0);
        }

        system_state_t state_snapshot = system_state_get();
        s_ctx.state = &state_snapshot;

        if (!(app->flags & SHELL_APP_FLAG_EXTERNAL)) {
            fb_clear();

            int content_y = (app->flags & SHELL_APP_FLAG_SHOW_HUD) ? SHELL_UI_HUD_HEIGHT : 0;
            int content_h = PANEL_H - content_y -
                            ((app->flags & SHELL_APP_FLAG_SHOW_LEGEND) ? SHELL_UI_LEGEND_HEIGHT : 0);
            if (content_h < 0) content_h = 0;

            if (app->draw) {
                int64_t t0 = esp_timer_get_time();
                app->draw(&s_ctx, s_fb, 0, content_y, PANEL_W, content_h);
                draw_us = (uint32_t)(esp_timer_get_time() - t0);
            }

            if (app->flags & SHELL_APP_FLAG_SHOW_HUD) {
                const char *hud_label = state_snapshot.led_mode_name;
                if (app->id && strcmp(app->id, "menu") == 0) {
                    hud_label = menu_hud_label();
                }
                shell_ui_draw_hud(s_fb, &state_snapshot, hud_label);
            }

            if (app->flags & SHELL_APP_FLAG_SHOW_LEGEND) {
                if (app->legend) {
                    shell_ui_draw_legend(s_fb, app->legend);
                }
            }

            oled_submit_frame();
        }

        uint32_t work_us = (uint32_t)(esp_timer_get_time() - frame_work_start_us);
        shell_profiler_on_frame(app->id ? app->id : "none",
                                (app->flags & SHELL_APP_FLAG_EXTERNAL) != 0,
                                frame_budget_us,
                                tick_us,
                                draw_us,
                                work_us);
        vTaskDelayUntil(&last_wake, frame_period);
    }
}

void app_shell_start(void)
{
    if (!shell_init_hw_and_display()) return;
    ESP_LOGI(TAG, "stage: oled, audio and imu init done");
    system_state_init();
    ESP_LOGI(TAG, "stage: system_state_init done");
    app_settings_init();
    ESP_LOGI(TAG, "stage: settings init done");
    // (void)led_modes_start();     // temporarily disabled
    // (void)power_monitor_start(); // temporarily disabled
    shell_setup_link(); // TODO: better names for these setup stages? and sepaate components for each service init? 
    ESP_LOGI(TAG, "stage: link setup done");
    // gps_services_start(9600); // temporarily disabled for now
    shell_seed_initial_system_state();
    shell_init_input_and_apps();
    shell_profiler_init();
    ESP_LOGI(TAG, "stage: input init and app init done");
    shell_run_loop();
}
