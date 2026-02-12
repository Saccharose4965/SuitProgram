#include "app_shell.h"
#include "shell_apps.h"
#include "shell_audio.h"
#include "shell_orientation.h"
#include "app_settings.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "hw.h"
#include "oled.h"
#include "gps.h"
#include "led_modes.h"
#include "power.h"
#include "link.h"
#include "audio_rx.h"
#include "bt_audio.h"
#include "storage_sd.h"
#include "mpu6500.h"
#include "orientation.h"
#include "pong.h"

static const char *TAG = "shell";

static DRAM_ATTR uint8_t s_fb[PANEL_W * PANEL_H / 8];
static DRAM_ATTR uint8_t s_fb_oled_a[PANEL_W * PANEL_H / 8];
static DRAM_ATTR uint8_t s_fb_oled_b[PANEL_W * PANEL_H / 8];
static uint8_t *s_fb_oled_latest = s_fb_oled_a;
static TaskHandle_t s_oled_task = NULL;

static size_t s_current_app = 0;
static size_t s_prev_app = 0;
static size_t s_pending_app = 0;
static bool   s_has_pending = false;

static shell_app_context_t s_ctx = {0};
static TaskHandle_t s_gps_time_task = NULL;
static bool s_imu_inited = false;
static TaskHandle_t s_imu_orient_task = NULL;
static mpu6500_t s_imu = { .mux = portMUX_INITIALIZER_UNLOCKED };
static imu_orientation_t s_ori = {0};

static void link_frame_handler(link_msg_type_t type, const uint8_t *payload, size_t len, void *user_ctx);
static inline void log_stage(const char *msg){ ESP_LOGI(TAG, "stage: %s", msg); }
static uint8_t s_peer_mac_cfg[6] = {0};

imu_orientation_t *shell_orientation_ctx(void)
{
    return &s_ori;
}

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

static void request_switch_cb(const char *id, void *user_data);

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

static void request_switch_cb(const char *id, void *user_data)
{
    (void)user_data;
    size_t idx = 0;
    if (find_app_by_id(id, &idx)) {
        s_pending_app = idx;
        s_has_pending = true;
    } else {
        ESP_LOGW(TAG, "request to unknown app '%s'", id ? id : "(null)");
    }
}

static void pump_pending_switch(void)
{
    if (!s_has_pending) return;
    s_has_pending = false;
    if (s_pending_app == s_current_app) return;
    switch_to_index(s_pending_app);
}

// Link frame dispatch (shared across apps)
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

static const shell_legend_t BT_AUDIO_LEGEND = BT_AUDIO_SHELL_LEGEND_INIT;

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
        .id     = "leds",
        .name   = "LEDs",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &LEDS_LEGEND,
        .init   = leds_app_init,
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
        .init   = bt_app_init,
        .deinit = NULL,
        .tick   = NULL,
        .handle_input = bt_app_handle_input,
        .draw   = bt_app_draw,
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
        .flags  = SHELL_APP_FLAG_EXTERNAL,
        .legend = &FFT_LEGEND,
        .init   = fft_app_init,
        .deinit = fft_app_deinit,
        .tick   = NULL,
        .handle_input = fft_stub_handle_input,
        .draw   = NULL,
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
        .flags  = SHELL_APP_FLAG_EXTERNAL,
        .legend = &FLAPPY_LEGEND,
        .init   = flappy_app_init,
        .deinit = flappy_app_deinit,
        .tick   = NULL,
        .handle_input = flappy_stub_handle_input,
        .draw   = NULL,
    },
    {
        .id     = "t2048",
        .name   = "2048 (legacy)",
        .flags  = SHELL_APP_FLAG_EXTERNAL,
        .legend = &T2048_LEGEND,
        .init   = t2048_app_init,
        .deinit = t2048_app_deinit,
        .tick   = NULL,
        .handle_input = t2048_stub_handle_input,
        .draw   = NULL,
    },
    {
        .id     = "tetris",
        .name   = "Tetris",
        .flags  = SHELL_APP_FLAG_EXTERNAL,
        .legend = &TETRIS_LEGEND,
        .init   = tetris_app_init,
        .deinit = tetris_app_deinit,
        .tick   = NULL,
        .handle_input = NULL,
        .draw   = NULL,
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
            request_switch_cb("menu", NULL);
            continue;
        }
        // Global escape to menu
        if (ev.type == INPUT_EVENT_LONG_PRESS) {
            if (ev.button == INPUT_BTN_AB_COMBO) {
                request_switch_cb("menu", NULL);
                continue;
            } else if (ev.button == INPUT_BTN_BC_COMBO) {
                esp_restart();
                continue;
            }
        }
        if (app && app->handle_input) {
            app->handle_input(&s_ctx, &ev);
        }
        pump_pending_switch();
        app = current_app();
        if (!app) break;
    }
}

// ======================================================================
// GPS time bridge → system_state (for HUD clock)
// ======================================================================

static void gps_time_task(void *arg)
{
    (void)arg;
    const TickType_t stale_window = pdMS_TO_TICKS(30000); // mark invalid if >30 s old
    TickType_t last_valid = 0;
    for (;;) {
        gps_fix_t f = gps_cached_fix();
        TickType_t now = xTaskGetTickCount();

        bool have_time = f.time_valid && f.hour >= 0 && f.hour < 24 && f.min >= 0 && f.min < 60;
        if (have_time) {
            system_state_set_time(true, (uint8_t)f.hour, (uint8_t)f.min);
            last_valid = now;
        } else if (last_valid != 0 && (now - last_valid) > stale_window) {
            system_state_set_time(false, 0, 0);
            last_valid = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Simple task to feed MPU samples into the orientation filter
static void imu_orientation_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(5); // ~200 Hz
    if (period < 1) period = 1;           // ensure non-zero increment for vTaskDelayUntil
    uint64_t last_t_us = 0;
    for (;;) {
        vTaskDelayUntil(&last, period);
        mpu6500_sample_t raw = mpu6500_latest(&s_imu);
        if (raw.t_us == 0 || raw.t_us == last_t_us) continue;
        last_t_us = raw.t_us;
        imu_orientation_update(
            &s_ori,
            raw.ax_g, raw.ay_g, raw.az_g,
            raw.gx_dps, raw.gy_dps, raw.gz_dps,
            raw.t_us
        );
    }
}

void app_shell_start(void)
{
    log_stage("start");
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init_fixed();
    oled_init();
    oled_clear();
    if (!s_oled_task) {
        xTaskCreatePinnedToCore(oled_task, "oled", 2048, NULL, 2, &s_oled_task, tskNO_AFFINITY);
    }
    log_stage("oled init done");
    
    // Audio bus for FFT (shared RX/TX)
    if (!shell_audio_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT won't run");
    }
    log_stage("audio init done");

    if (!s_imu_inited) {
        hw_spi2_idle_all_cs_high();
        const gpio_num_t cs = PIN_CS_IMU1;
        const uint32_t freqs[] = { 8000000, 1000000 };
        for (size_t f = 0; f < sizeof(freqs)/sizeof(freqs[0]); ++f) {
            spi_device_handle_t dev = NULL;
            if (hw_spi2_add_device(cs, freqs[f], 0, 4, &dev) != ESP_OK || !dev) continue;
            mpu6500_config_t cfg = { .spi = dev, .tag = "mpu_shell" };
            if (mpu6500_init(&s_imu, &cfg) == ESP_OK) {
                if (mpu6500_start_sampler(&s_imu, 200, 2, 1, 0) == ESP_OK) {
                    s_imu_inited = true;
                    ESP_LOGI(TAG, "IMU sampler started @CS%u (%u Hz)", (unsigned)cs, (unsigned)freqs[f]);
                    imu_orientation_init(&s_ori, 0.05f);
                    if (!s_imu_orient_task) {
                        xTaskCreatePinnedToCore(imu_orientation_task, "imu_orient", 3072, NULL, 2, &s_imu_orient_task, 1);
                    }
                    break;
                }
            }
            (void)spi_bus_remove_device(dev);
        }
        if (!s_imu_inited) {
            ESP_LOGW(TAG, "IMU sampler not started (no device)");
        }
    }
    log_stage("imu init done");

    system_state_init();
    log_stage("system_state_init done");

    app_settings_init();
    log_stage("settings init done");

    // Toggle subsystems that have caused instability on some hardware spins.
#ifdef CONFIG_APP_ENABLE_LED_MODES
    const bool enable_led_modes = CONFIG_APP_ENABLE_LED_MODES;
#else
    const bool enable_led_modes = false; // default off to avoid RMT-related resets
#endif
#ifdef CONFIG_APP_ENABLE_POWER_MON
    const bool enable_power = CONFIG_APP_ENABLE_POWER_MON;
#else
    const bool enable_power = false; // default off while debugging resets
#endif

    if (enable_led_modes) {
        if (led_modes_start() != ESP_OK) {
            ESP_LOGW(TAG, "LED modes failed to start");
        }
        log_stage("led_modes_start done");
    } else {
        ESP_LOGI(TAG, "LED modes disabled (debug)");
    }

    if (enable_power) {
        if (power_monitor_start() != ESP_OK) {
            ESP_LOGW(TAG, "Power monitor failed to start");
        }
        log_stage("power_monitor_start done");
    } else {
        ESP_LOGI(TAG, "Power monitor disabled (debug)");
    }
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

        // Make sure SD is mounted before audio_rx tries to write to it
        esp_err_t sd = storage_mount_sd();
        if (sd != ESP_OK) {
            ESP_LOGW(TAG, "SD mount failed (%s); audio_rx recording disabled",
                    esp_err_to_name(sd));
        } else {
            ESP_LOGI(TAG, "SD mounted; audio_rx recordings will be written to /sdcard");
        }
        
        // Start audio receiver once lwIP / Wi-Fi stack is up
        esp_err_t ar = audio_rx_start(AUDIO_RX_DEFAULT_PORT);
        if (ar != ESP_OK) {
            ESP_LOGW(TAG, "audio_rx_start failed: %s", esp_err_to_name(ar));
        }
    }
    log_stage("link setup done");

#ifdef CONFIG_APP_ENABLE_GPS
    const bool enable_gps = CONFIG_APP_ENABLE_GPS;
#else
    const bool enable_gps = false; // default off while isolating resets
#endif
    if (enable_gps) {
        gps_service_start(9600);
        if (!s_gps_time_task) {
            xTaskCreatePinnedToCore(gps_time_task, "gps_time", 2048, NULL, 4, &s_gps_time_task, tskNO_AFFINITY);
        }
        log_stage("gps start done");
    } else {
        ESP_LOGI(TAG, "GPS disabled (debug)");
    }
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

    input_config_t input_cfg = {
        .long_press_ms = 1200,     // 1.2s long-press for A+B menu escape
        .ab_combo_mv = 1100,       // A+B combo target (~1.1 V)
        .ab_combo_tol_mv = 350,    // tolerate ladder noise
        .ab_combo_verify_ms = 80,
    };
    input_init(&input_cfg);
    log_stage("input init done");

    bt_audio_set_disconnect_cb(music_stop_playback);

    s_ctx.registry = s_builtin_apps;
    s_ctx.registry_count = sizeof(s_builtin_apps) / sizeof(s_builtin_apps[0]);
    s_ctx.request_switch = request_switch_cb;
    s_ctx.request_user_data = NULL;

    // Boot straight into the menu (skip blank title screen)
    size_t menu_idx = 0;
    if (!find_app_by_id("menu", &menu_idx)) {
        menu_idx = 0;
    }
    s_current_app = menu_idx;
    s_prev_app = menu_idx;
    switch_to_index(menu_idx);

    const TickType_t frame_period = pdMS_TO_TICKS(33);
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tick = last_wake;
    log_stage("enter loop");

    while (1) {
        TickType_t now = xTaskGetTickCount();
        float dt_sec = (float)(now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        last_tick = now;

        const shell_app_desc_t *app = current_app();
        process_input_events(app, now);
        pump_pending_switch();
        app = current_app();
        if (!app) {
            vTaskDelayUntil(&last_wake, frame_period);
            continue;
        }

        if (app->tick) {
            app->tick(&s_ctx, dt_sec);
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
                app->draw(&s_ctx, s_fb, 0, content_y, PANEL_W, content_h);
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
        vTaskDelayUntil(&last_wake, frame_period);
    }
}
