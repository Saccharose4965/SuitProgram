#include "app_shell.h"
#include "shell_apps.h"
#include "shell_audio.h"
#include "shell_profiler.h"
#include "app_settings.h"
#include "audio.h"
#include "audio_player.h"
#include "audio_rx.h"
#include "fft.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "hw.h"
#include "oled.h"
#include "gps.h"
#include "led_layout.h"
#include "led_modes.h"
#include "logo.h"
#include "power.h"
#include "link.h"
#include "bt_audio.h"
#include "orientation_service.h"
#include "pong.h"

static const char *TAG = "shell";
enum { OLED_FB_BYTES = PANEL_W * PANEL_H / 8 };

#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#define OLED_TASK_CORE 0
#else
#define APP_TASK_CORE 0
#define OLED_TASK_CORE 0
#endif

static DRAM_ATTR uint8_t s_fb_oled_a[OLED_FB_BYTES];
static DRAM_ATTR uint8_t s_fb_oled_b[OLED_FB_BYTES];
static uint8_t *s_render_fb = s_fb_oled_a;
static uint8_t *s_oled_fb = s_fb_oled_b;
static TaskHandle_t s_oled_task = NULL;
static QueueHandle_t s_oled_q = NULL;
static uint64_t s_last_submitted_hash = UINT64_MAX;
static TaskHandle_t s_logo_task = NULL;
static volatile bool s_logo_active = false;
static TaskHandle_t s_startup_tone_task = NULL;
static TaskHandle_t s_shell_task = NULL;
static esp_timer_handle_t s_shell_frame_timer = NULL;

static size_t s_current_app = 0;
static size_t s_prev_app = 0;
static size_t s_queued_app_index = 0;
static bool   s_app_switch_queued = false;

static shell_app_context_t s_ctx = {0};

static const uint32_t SHELL_FRAME_BUDGET_US = 16667u;

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

static inline void fb_clear(uint8_t *fb)
{
    if (!fb) return;
    memset(fb, 0, OLED_FB_BYTES);
}

static inline uint64_t fb_hash(const uint8_t *fb)
{
    if (!fb) return 0;
    uint64_t h = 1469598103934665603ULL; // FNV-1a 64-bit
    for (size_t i = 0; i < OLED_FB_BYTES; ++i) {
        h ^= (uint64_t)fb[i];
        h *= 1099511628211ULL;
    }
    return h;
}

static void oled_task(void *arg)
{
    (void)arg;
    uint8_t *buf = NULL;
    for (;;) {
        if (!s_oled_q) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xQueueReceive(s_oled_q, &buf, portMAX_DELAY) == pdTRUE && buf) {
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

static void shell_frame_timer_cb(void *arg)
{
    TaskHandle_t task = (TaskHandle_t)arg;
    if (task) {
        xTaskNotifyGive(task);
    }
}

static bool shell_frame_clock_start(void)
{
    if (s_shell_frame_timer) return true;

    s_shell_task = xTaskGetCurrentTaskHandle();
    if (!s_shell_task) return false;

    const esp_timer_create_args_t args = {
        .callback = shell_frame_timer_cb,
        .arg = s_shell_task,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "shell_frame",
    };

    esp_err_t err = esp_timer_create(&args, &s_shell_frame_timer);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "shell frame timer create failed: %s", esp_err_to_name(err));
        s_shell_frame_timer = NULL;
        return false;
    }

    err = esp_timer_start_periodic(s_shell_frame_timer, SHELL_FRAME_BUDGET_US);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "shell frame timer start failed: %s", esp_err_to_name(err));
        (void)esp_timer_delete(s_shell_frame_timer);
        s_shell_frame_timer = NULL;
        return false;
    }

    while (ulTaskNotifyTake(pdTRUE, 0) > 0) {}
    return true;
}

static inline void shell_wait_next_frame(void)
{
    if (s_shell_frame_timer) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        return;
    }

    vTaskDelay(2);
}

static void startup_logo_task(void *arg)
{
    (void)arg;
    anim_logo();
    s_logo_active = false;
    s_logo_task = NULL;
    vTaskDelete(NULL);
}

static void shell_start_logo_async(void)
{
    if (s_logo_active || s_logo_task) return;
    s_logo_active = true;
    BaseType_t ok = xTaskCreatePinnedToCore(
        startup_logo_task,
        "logo_boot",
        3072,
        NULL,
        1,
        &s_logo_task,
        OLED_TASK_CORE
    );
    if (ok != pdPASS) {
        s_logo_active = false;
        s_logo_task = NULL;
        ESP_LOGW(TAG, "logo task create failed; falling back to blocking logo");
        anim_logo();
    }
}

static void shell_wait_for_logo(void)
{
    while (s_logo_active) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static inline void oled_submit_frame(void)
{
    uint8_t *just_drawn = s_render_fb;
    s_render_fb = (s_render_fb == s_fb_oled_a) ? s_fb_oled_b : s_fb_oled_a;

    uint64_t hash = fb_hash(just_drawn);
    if (hash == s_last_submitted_hash) {
        return;
    }
    s_last_submitted_hash = hash;
    s_oled_fb = just_drawn;

    if (!s_oled_task || !s_oled_q) {
        oled_blit_full(s_oled_fb);
        return;
    }

    uint8_t *p = s_oled_fb;
    xQueueOverwrite(s_oled_q, &p);
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

static void shell_graceful_restart(void)
{
    ESP_LOGI(TAG, "graceful restart: begin");

    s_app_switch_queued = false;

    const shell_app_desc_t *app = current_app();
    if (app && app->deinit) {
        ESP_LOGI(TAG, "graceful restart: deinit app [%s]", app->id ? app->id : "?");
        app->deinit(&s_ctx);
    }

    led_modes_enable(false);
    led_beat_enable(false);

    fft_set_display_enabled(false);
    fft_visualizer_stop();

    audio_player_stop();

    if (audio_rx_is_running()) {
        esp_err_t rx_err = audio_rx_stop();
        if (rx_err != ESP_OK) {
            ESP_LOGW(TAG, "graceful restart: audio_rx_stop failed: %s", esp_err_to_name(rx_err));
        }
    }

    esp_err_t bt_stop_err = bt_audio_stop_stream();
    if (bt_stop_err != ESP_OK && bt_stop_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "graceful restart: bt stop stream failed: %s", esp_err_to_name(bt_stop_err));
    }
    esp_err_t bt_disc_err = bt_audio_disconnect();
    if (bt_disc_err != ESP_OK && bt_disc_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "graceful restart: bt disconnect failed: %s", esp_err_to_name(bt_disc_err));
    }

    esp_err_t link_frame_err = link_set_frame_rx(NULL, NULL);
    if (link_frame_err != ESP_OK) {
        ESP_LOGW(TAG, "graceful restart: link frame rx detach failed: %s", esp_err_to_name(link_frame_err));
    }
    esp_err_t link_rx_err = link_set_rx(NULL, NULL);
    if (link_rx_err != ESP_OK) {
        ESP_LOGW(TAG, "graceful restart: link rx detach failed: %s", esp_err_to_name(link_rx_err));
    }

    esp_err_t audio_err = audio_disable_all();
    if (audio_err != ESP_OK) {
        ESP_LOGW(TAG, "graceful restart: audio disable failed: %s", esp_err_to_name(audio_err));
    }

    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "graceful restart: restarting now");
    esp_restart();
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
        .flags  = SHELL_APP_FLAG_SHOW_HUD,
        .legend = NULL,
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
        .tick   = leds_app_tick,
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
        .tick   = leds_app_tick,
        .handle_input = leds_app_handle_input,
        .draw   = leds_app_draw,
    },
    {
        .id     = "leds_layout",
        .name   = "LED Layout",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &LED_LAYOUT_LEGEND,
        .init   = led_layout_app_init,
        .deinit = led_layout_app_deinit,
        .tick   = led_layout_app_tick,
        .handle_input = led_layout_app_handle_input,
        .draw   = led_layout_app_draw,
    },
    {
        .id     = "manual_bpm",
        .name   = "Manual BPM",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &MANUAL_BPM_LEGEND,
        .init   = manual_bpm_app_init,
        .deinit = manual_bpm_app_deinit,
        .tick   = manual_bpm_app_tick,
        .handle_input = manual_bpm_app_handle_input,
        .draw   = manual_bpm_app_draw,
    },
    {
        .id     = "fft_sync",
        .name   = "FFT Sync",
        .flags  = SHELL_APP_FLAG_SHOW_HUD | SHELL_APP_FLAG_SHOW_LEGEND,
        .legend = &FFT_SYNC_LEGEND,
        .init   = fft_sync_app_init,
        .deinit = fft_sync_app_deinit,
        .tick   = NULL,
        .handle_input = fft_sync_app_handle_input,
        .draw   = fft_sync_app_draw,
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
                shell_graceful_restart();
                return;
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
    s_render_fb = s_fb_oled_a;
    s_oled_fb = s_fb_oled_b;
    fb_clear(s_fb_oled_a);
    fb_clear(s_fb_oled_b);
    s_last_submitted_hash = UINT64_MAX;
    if (!s_oled_q) {
        s_oled_q = xQueueCreate(1, sizeof(uint8_t *));
        if (!s_oled_q) {
            ESP_LOGE(TAG, "oled queue create failed");
            return false;
        }
    }
    if (!s_oled_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(oled_task, "oled", 2048, NULL, 2, &s_oled_task, OLED_TASK_CORE);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "oled task create failed");
            s_oled_task = NULL;
            return false;
        }
    }

    shell_start_logo_async();

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
    int64_t last_tick_us = esp_timer_get_time();
    (void)shell_frame_clock_start();
    ESP_LOGI(TAG, "stage: enter loop");

    while (1) {
        int64_t frame_work_start_us = esp_timer_get_time();
        uint32_t tick_us = 0;
        uint32_t draw_us = 0;

        int64_t now_us = frame_work_start_us;
        float dt_sec = (float)(now_us - last_tick_us) * 1e-6f;
        if (dt_sec < 0.0f) dt_sec = 0.0f;
        last_tick_us = now_us;

        TickType_t now = xTaskGetTickCount();

        const shell_app_desc_t *app = current_app();
        process_input_events(app, now);
        apply_queued_app_switch();
        app = current_app();
        if (!app) {
            uint32_t work_us = (uint32_t)(esp_timer_get_time() - frame_work_start_us);
            shell_profiler_on_frame("none", false, SHELL_FRAME_BUDGET_US, 0, 0, work_us);
            shell_wait_next_frame();
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
            fb_clear(s_render_fb);

            int content_y = (app->flags & SHELL_APP_FLAG_SHOW_HUD) ? SHELL_UI_HUD_HEIGHT : 0;
            int content_h = PANEL_H - content_y -
                            ((app->flags & SHELL_APP_FLAG_SHOW_LEGEND) ? SHELL_UI_LEGEND_HEIGHT : 0);
            if (content_h < 0) content_h = 0;

            if (app->draw) {
                int64_t t0 = esp_timer_get_time();
                app->draw(&s_ctx, s_render_fb, 0, content_y, PANEL_W, content_h);
                draw_us = (uint32_t)(esp_timer_get_time() - t0);
            }

            if (app->flags & SHELL_APP_FLAG_SHOW_HUD) {
                const char *hud_label = state_snapshot.led_mode_name;
                if (app->id && strcmp(app->id, "menu") == 0) {
                    hud_label = menu_hud_label();
                }
                shell_ui_draw_hud(s_render_fb, &state_snapshot, hud_label);
            }

            if (app->flags & SHELL_APP_FLAG_SHOW_LEGEND) {
                if (app->legend) {
                    shell_ui_draw_legend(s_render_fb, app->legend);
                }
            }

            oled_submit_frame();
        }

        uint32_t work_us = (uint32_t)(esp_timer_get_time() - frame_work_start_us);
        shell_profiler_on_frame(app->id ? app->id : "none",
                                (app->flags & SHELL_APP_FLAG_EXTERNAL) != 0,
                                SHELL_FRAME_BUDGET_US,
                                tick_us,
                                draw_us,
                                work_us);
        shell_wait_next_frame();
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
    {
        esp_err_t layout_err = led_layout_init();
        if (layout_err != ESP_OK) {
            ESP_LOGW(TAG, "led_layout init warning: %s", esp_err_to_name(layout_err));
        }
    }
    // (void)led_modes_start();     // temporarily disabled
    // (void)power_monitor_start(); // temporarily disabled
    shell_setup_link(); // TODO: better names for these setup stages? and sepaate components for each service init? 
    ESP_LOGI(TAG, "stage: link setup done");
    // gps_services_start(9600); // temporarily disabled for now
    shell_seed_initial_system_state();
    shell_init_input_and_apps();
    shell_profiler_init();
    shell_wait_for_logo();
    ESP_LOGI(TAG, "stage: input init and app init done");
    shell_run_loop();
}
