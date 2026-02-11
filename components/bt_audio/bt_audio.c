#include "bt_audio.h"
#include "bt_audio_internal.h"
#include "audio.h"
#include "audio_player.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "system_state.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "bta/bta_api.h"
#include "bta_dm_int.h"

// The public ESP-IDF API has no helper to toggle sniff for A2DP source.
// Use the Bluedroid primitives directly to force sniff off (avoids the
// unsniff deadlock seen on some sinks).
#ifndef HCI_ENABLE_MASTER_SLAVE_SWITCH
#define HCI_ENABLE_MASTER_SLAVE_SWITCH 0x0001
#endif
#ifndef HCI_ENABLE_HOLD_MODE
#define HCI_ENABLE_HOLD_MODE 0x0002
#endif
#ifndef HCI_ENABLE_SNIFF_MODE
#define HCI_ENABLE_SNIFF_MODE 0x0004
#endif
#ifndef HCI_ENABLE_PARK_MODE
#define HCI_ENABLE_PARK_MODE 0x0008
#endif
extern void BTM_SetDefaultLinkPolicy(uint16_t settings);
extern uint8_t BTM_SetLinkPolicy(uint8_t bd_addr[6], uint16_t *settings);

static const char *TAG = "bt_audio";

// ===== AVRCP hooks (weak, overridden by higher-level code like audio_player) =====
void __attribute__((weak)) bt_audio_on_play(void)   {}
void __attribute__((weak)) bt_audio_on_pause(void)  {}
void __attribute__((weak)) bt_audio_on_next(void)   {}
void __attribute__((weak)) bt_audio_on_prev(void)   {}

// ============================ State ============================
static bt_audio_state_t s_state         = BT_AUDIO_STATE_IDLE;
static bool             s_stack_ready   = false;
static bool             s_streaming     = false;
static bool             s_user_streaming = false;
static bool             s_media_cmd_pending = false;
static esp_a2d_media_ctrl_t s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
static esp_bd_addr_t    s_peer_bda      = {0};
static char             s_peer_name[32] = {0};
static int              s_peer_sample_rate = 44100;
static bt_audio_device_t s_devices[8];
static int               s_device_count = 0;
static bool              s_name_req_active = false;
static esp_bd_addr_t     s_name_req_bda = {0};
static bool              s_scanning = false;
static bt_audio_disconnect_cb_t s_disconnect_cb = NULL;
static bool              s_pending_connect = false;
static esp_bd_addr_t     s_pending_bda = {0};
static int               s_pending_attempts = 0;
static TimerHandle_t     s_pending_timer = NULL;
static bool bda_is_zero(const esp_bd_addr_t bda);
static esp_err_t start_connect(esp_bd_addr_t bda);
static esp_err_t start_discovery(void);
static void clear_pending_connect(void)
{
    s_pending_connect = false;
    memset(s_pending_bda, 0, sizeof(s_pending_bda));
    s_pending_attempts = 0;
    if (s_pending_timer) {
        xTimerStop(s_pending_timer, 0);
    }
}

static void pending_connect_timer_cb(TimerHandle_t tmr)
{
    (void)tmr;
    if (!s_pending_connect || bda_is_zero(s_pending_bda)) {
        clear_pending_connect();
        return;
    }
    if (s_state == BT_AUDIO_STATE_CONNECTING ||
        s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING) {
        return; // already in progress or satisfied
    }
    if (s_pending_attempts >= 3) {
        ESP_LOGW(TAG, "Pending connect retries exhausted");
        clear_pending_connect();
        (void)start_discovery();
        return;
    }
    s_pending_attempts++;
    ESP_LOGI(TAG, "Retry pending connect (%d/3)", s_pending_attempts);
    (void)start_connect(s_pending_bda);
}

static void schedule_pending_connect_retry(uint32_t delay_ms)
{
    if (!s_pending_timer) {
        s_pending_timer = xTimerCreate("bt_pend",
                                       pdMS_TO_TICKS(delay_ms),
                                       pdFALSE,
                                       NULL,
                                       pending_connect_timer_cb);
    }
    if (s_pending_timer) {
        xTimerStop(s_pending_timer, 0);
        xTimerChangePeriod(s_pending_timer, pdMS_TO_TICKS(delay_ms), 0);
        xTimerStart(s_pending_timer, 0);
    }
}

// Forward declarations for helper routines used before their definitions.
static void add_device(const esp_bd_addr_t bda, const char *name);
static void maybe_request_name(esp_bd_addr_t bda);
static void reset_devices(void);
static void avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                       esp_avrc_tg_cb_param_t *param);

static void set_state(bt_audio_state_t st){
    s_state = st;
}

static void set_connection(system_connection_t conn){
    system_state_set_connection(conn);
}

void bt_audio_set_disconnect_cb(bt_audio_disconnect_cb_t cb)
{
    s_disconnect_cb = cb;
}

void bt_audio_invoke_disconnect_cb(void)
{
    if (s_disconnect_cb) {
        s_disconnect_cb();
    }
}

static void set_pending_connect(const esp_bd_addr_t bda)
{
    memcpy(s_pending_bda, bda, sizeof(s_pending_bda));
    s_pending_connect = !bda_is_zero(s_pending_bda);
    s_pending_attempts = 0;
}

// ============================ Helpers ============================
static esp_err_t ensure_nvs(void){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_LOGW(TAG, "NVS full; erasing");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase nvs");
        err = nvs_flash_init();
    }
    return err;
}

static bool cod_is_audio(uint32_t cod){
    uint32_t major_service = (cod & 0x00FF0000) >> 16;
    uint32_t major_device  = (cod & 0x00001F00) >> 8;
    return (major_service & (ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO)) ||
           (major_device == ESP_BT_COD_MAJOR_DEV_AV);
}

// Device list helpers
static int find_device(const esp_bd_addr_t bda){
    for (int i = 0; i < s_device_count; ++i){
        if (memcmp(s_devices[i].bda, bda, sizeof(esp_bd_addr_t)) == 0) return i;
    }
    return -1;
}

static void add_device(const esp_bd_addr_t bda, const char *name){
    int idx = find_device(bda);
    if (idx < 0 && s_device_count < (int)(sizeof(s_devices)/sizeof(s_devices[0]))){
        idx = s_device_count++;
    }
    if (idx >= 0){
        memcpy(s_devices[idx].bda, bda, sizeof(esp_bd_addr_t));
        if (name && name[0]) {
            strncpy(s_devices[idx].name, name, sizeof(s_devices[idx].name) - 1);
            s_devices[idx].name[sizeof(s_devices[idx].name) - 1] = '\0';
        }
    }
}

static void reset_devices(void){
    s_device_count = 0;
    memset(s_devices, 0, sizeof(s_devices));
}

static bool bda_is_zero(const esp_bd_addr_t bda)
{
    if (!bda) return true;
    for (int i = 0; i < 6; ++i) {
        if (bda[i] != 0) return false;
    }
    return true;
}

static void disconnect_all_peers_except(const esp_bd_addr_t keep, bool use_keep)
{
    if (!s_stack_ready) {
        return;
    }

    for (int i = 0; i < s_device_count; ++i) {
        if (bda_is_zero(s_devices[i].bda)) {
            continue;
        }
        if (use_keep &&
            memcmp(keep, s_devices[i].bda, sizeof(esp_bd_addr_t)) == 0) {
            continue;
        }

        esp_err_t e = esp_a2d_source_disconnect(s_devices[i].bda);
        if (e != ESP_OK &&
            e != ESP_ERR_INVALID_STATE &&
            e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "disconnect dev %d failed: %s",
                     i, esp_err_to_name(e));
        }
    }

    // Also try the current peer directly if it's not the "keep" one
    if (!bda_is_zero(s_peer_bda) && (!use_keep ||
        memcmp(keep, s_peer_bda, sizeof(s_peer_bda)) != 0)) {
        esp_err_t e = esp_a2d_source_disconnect(s_peer_bda);
        if (e != ESP_OK &&
            e != ESP_ERR_INVALID_STATE &&
            e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "disconnect current peer failed: %s",
                     esp_err_to_name(e));
        }
    }
}

static void maybe_request_name(esp_bd_addr_t bda){
    if (s_name_req_active) return;
    int idx = find_device(bda);
    if (idx >= 0 && s_devices[idx].name[0]) return;
    esp_err_t e = esp_bt_gap_read_remote_name(bda);
    if (e == ESP_OK){
        s_name_req_active = true;
        memcpy(s_name_req_bda, bda, sizeof(s_name_req_bda));
    }
}

// ============================ A2DP callbacks ============================

static void a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    switch (event){
        case ESP_A2D_CONNECTION_STATE_EVT:{
            esp_a2d_connection_state_t st = param->conn_stat.state;
            ESP_LOGI(TAG, "A2DP conn state %d", st);
            if (st == ESP_A2D_CONNECTION_STATE_CONNECTED){
                memcpy(s_peer_bda, param->conn_stat.remote_bda, sizeof(s_peer_bda));
                clear_pending_connect(); // satisfied
                uint16_t nosniff_policy = HCI_ENABLE_MASTER_SLAVE_SWITCH | HCI_ENABLE_HOLD_MODE | HCI_ENABLE_PARK_MODE;
                (void)BTM_SetLinkPolicy(s_peer_bda, &nosniff_policy);
                ESP_LOGI(TAG, "Applied no-sniff link policy to peer");
                add_device(s_peer_bda, NULL);
                maybe_request_name(s_peer_bda);
                set_state(BT_AUDIO_STATE_CONNECTED);
                s_user_streaming = false;
                s_streaming = false;
                set_connection(SYS_CONN_CONNECTED);
            } else if (st == ESP_A2D_CONNECTION_STATE_CONNECTING){
                set_state(BT_AUDIO_STATE_CONNECTING);
                set_connection(SYS_CONN_CONNECTING);
            } else if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED ||
                       st == ESP_A2D_CONNECTION_STATE_DISCONNECTING){
                s_user_streaming = false;
                s_streaming = false;
                s_media_cmd_pending = false;
                s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
                set_state(BT_AUDIO_STATE_IDLE);
                set_connection(SYS_CONN_DISCONNECTED);
                memset(s_peer_bda, 0, sizeof(s_peer_bda));
                s_peer_name[0] = '\0';
                if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
                    break; // wait for DISCONNECTED to handle pending/restart
                }
                bool tried_pending = false;
                if (s_pending_connect && !bda_is_zero(s_pending_bda)) {
                    s_pending_attempts++;
                    if (s_pending_attempts > 3) {
                        ESP_LOGW(TAG, "Pending connect failed %d times, giving up", s_pending_attempts - 1);
                        clear_pending_connect();
                    } else {
                        ESP_LOGI(TAG, "Attempt pending connect after disconnect (%d/3)", s_pending_attempts);
                        tried_pending = (start_connect(s_pending_bda) == ESP_OK);
                        if (!tried_pending) {
                            schedule_pending_connect_retry(400);
                        }
                    }
                }
                if (!tried_pending) {
                    (void)start_discovery();
                }
            }

            break;
        }
        case ESP_A2D_AUDIO_STATE_EVT:{
            esp_a2d_audio_state_t st = param->audio_stat.state;
            ESP_LOGI(TAG, "A2DP audio state %d", st);
            if (st == ESP_A2D_AUDIO_STATE_STARTED){
                s_streaming = true;
                set_state(BT_AUDIO_STATE_STREAMING);
            } else {
                s_streaming = false;
                if (s_state == BT_AUDIO_STATE_STREAMING){
                    set_state(BT_AUDIO_STATE_CONNECTED);
                }
            }
            break;
        }
        case ESP_A2D_AUDIO_CFG_EVT:{
            uint8_t sf = param->audio_cfg.mcc.cie.sbc_info.samp_freq;
            int sample_rate = 16000;
            if (sf & ESP_A2D_SBC_CIE_SF_48K) sample_rate = 48000;
            else if (sf & ESP_A2D_SBC_CIE_SF_44K) sample_rate = 44100;
            else if (sf & ESP_A2D_SBC_CIE_SF_32K) sample_rate = 32000;
            s_peer_sample_rate = sample_rate;
            ESP_LOGI(TAG, "A2DP cfg: %d Hz", sample_rate);
            break;
        }
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:{
            esp_a2d_media_ctrl_t cmd = param->media_ctrl_stat.cmd;
            esp_a2d_media_ctrl_ack_t status = param->media_ctrl_stat.status;
            s_media_cmd_pending = false;
            s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
            if (status != ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGW(TAG, "A2DP media ctrl ack %d for cmd %d", status, cmd);
                if (cmd == ESP_A2D_MEDIA_CTRL_START) {
                    s_streaming = false;
                    if (s_state == BT_AUDIO_STATE_STREAMING) {
                        set_state(BT_AUDIO_STATE_CONNECTED);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================ GAP callback ============================
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event){
        case ESP_BT_GAP_DISC_RES_EVT:{
            bool audio_dev = false;
            char name[32] = {0};
            uint32_t cod = 0;
            for (int i = 0; i < param->disc_res.num_prop; ++i){
                esp_bt_gap_dev_prop_t *p = param->disc_res.prop + i;
                if (p->type == ESP_BT_GAP_DEV_PROP_COD && p->val){
                    cod = *(uint32_t*)p->val;
                    audio_dev = cod_is_audio(cod);
                } else if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME && p->val){
                    size_t n = p->len < sizeof(name) - 1 ? p->len : sizeof(name) - 1;
                    memcpy(name, p->val, n);
                    name[n] = 0;
                }
            }
            if (audio_dev){
                add_device(param->disc_res.bda, name[0] ? name : NULL);
                ESP_LOGI(TAG, "Found audio dev %s", name[0] ? name : "(unnamed)");
                if (!name[0]) {
                    maybe_request_name(param->disc_res.bda);
                }
                if (memcmp(param->disc_res.bda, s_peer_bda, sizeof(s_peer_bda)) == 0 && name[0]) {
                    strncpy(s_peer_name, name, sizeof(s_peer_name) - 1);
                    s_peer_name[sizeof(s_peer_name) - 1] = '\0';
                }
            }
            break;
        }
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:{
            s_name_req_active = false;
            if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS){
                add_device(param->read_rmt_name.bda, (const char*)param->read_rmt_name.rmt_name);
                ESP_LOGI(TAG, "Name resolved: %s", param->read_rmt_name.rmt_name);
                if (memcmp(param->read_rmt_name.bda, s_peer_bda, sizeof(s_peer_bda)) == 0) {
                    strncpy(s_peer_name, (const char*)param->read_rmt_name.rmt_name, sizeof(s_peer_name) - 1);
                    s_peer_name[sizeof(s_peer_name) - 1] = '\0';
                }
            }
            break;
        }
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:{
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                s_scanning = true;
                // Keep state/UI synchronized even if another callback briefly
                // forced us back to IDLE while inquiry is still active.
                set_state(BT_AUDIO_STATE_DISCOVERING);
                set_connection(SYS_CONN_CONNECTING);
            } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED){
                s_scanning = false;
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

                if (s_state == BT_AUDIO_STATE_DISCOVERING) {
                    set_state(BT_AUDIO_STATE_IDLE);
                    set_connection(SYS_CONN_DISCONNECTED);
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================ Start/Connect ============================
static esp_err_t start_connect(esp_bd_addr_t bda){
    if (!bda) return ESP_ERR_INVALID_ARG;
    // Stop any active discovery before initiating a link; otherwise the
    // controller can get congested and fail the new connection.
    (void)esp_bt_gap_cancel_discovery();
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    s_scanning = false;
    set_state(BT_AUDIO_STATE_CONNECTING);
    set_connection(SYS_CONN_CONNECTING);
    memcpy(s_peer_bda, bda, sizeof(s_peer_bda));
    esp_err_t err = esp_a2d_source_connect(bda);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "connect err %s", esp_err_to_name(err));
        set_state(BT_AUDIO_STATE_FAILED);
        set_connection(SYS_CONN_DISCONNECTED);
        if (s_pending_connect) {
            schedule_pending_connect_retry(400);
        } else {
            (void)start_discovery(); // recover by restarting inquiry
        }
    }
    return err;
}

static esp_err_t start_discovery(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING ||
        s_state == BT_AUDIO_STATE_CONNECTING) {
        ESP_LOGI(TAG, "start_discovery: busy (state=%s), skipping", bt_audio_state_name(s_state));
        return ESP_ERR_INVALID_STATE;
    }
    if (s_scanning) {
        ESP_LOGI(TAG, "start_discovery: already scanning");
        // Re-assert discovery state so UI/HUD do not remain in IDLE while
        // inquiry is active.
        set_state(BT_AUDIO_STATE_DISCOVERING);
        set_connection(SYS_CONN_CONNECTING);
        return ESP_OK;
    }
    s_scanning = true;
    set_state(BT_AUDIO_STATE_DISCOVERING);
    set_connection(SYS_CONN_CONNECTING);
    (void)esp_bt_gap_cancel_discovery();
    // Keep inquiry short to reduce controller airtime (less impact on streaming).
    esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "discovery start err %s", esp_err_to_name(err));
        s_scanning = false;
        set_state(BT_AUDIO_STATE_FAILED);
        set_connection(SYS_CONN_DISCONNECTED);
    }
    return err;
}

// ============================ Public API ============================
esp_err_t bt_audio_start(bool force_rescan){
    ESP_RETURN_ON_ERROR(ensure_nvs(), TAG, "nvs");

    if (!s_stack_ready){
        esp_err_t rel = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
        if (rel != ESP_OK && rel != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "release BLE %s", esp_err_to_name(rel));
            return rel;
        }
        esp_bt_controller_status_t ctrl_st = esp_bt_controller_get_status();
        if (ctrl_st == ESP_BT_CONTROLLER_STATUS_IDLE) {
            esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
            esp_err_t e = esp_bt_controller_init(&bt_cfg);
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "ctrl init %s", esp_err_to_name(e));
                return e;
            }
        }
        if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
            esp_bt_mode_t mode = ESP_BT_MODE_CLASSIC_BT;
            esp_err_t e = esp_bt_controller_enable(mode);
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "ctrl enable %s", esp_err_to_name(e));
                return e;
            }
        }

        esp_bluedroid_status_t bd_st = esp_bluedroid_get_status();
        if (bd_st == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "bd init");
            bd_st = esp_bluedroid_get_status();
        }
        if (bd_st != ESP_BLUEDROID_STATUS_ENABLED) {
            esp_err_t e = esp_bluedroid_enable();
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "bd enable %s", esp_err_to_name(e));
                return e;
            }
        }
        uint16_t nosniff_policy = HCI_ENABLE_MASTER_SLAVE_SWITCH | HCI_ENABLE_HOLD_MODE | HCI_ENABLE_PARK_MODE;
        BTM_SetDefaultLinkPolicy(nosniff_policy);
        ESP_LOGI(TAG, "Sniff mode disabled (policy=0x%04x)", nosniff_policy);
        
        ESP_RETURN_ON_ERROR(esp_avrc_tg_init(), TAG, "avrc tg init");
        ESP_RETURN_ON_ERROR(esp_avrc_tg_register_callback(avrc_tg_cb), TAG, "avrc tg cb");

        ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(gap_cb), TAG, "gap cb");
        ESP_RETURN_ON_ERROR(esp_a2d_register_callback(a2dp_cb), TAG, "a2dp cb");
        esp_err_t data_cb_err = esp_a2d_source_register_data_callback(bt_audio_a2dp_data_cb);
        if (data_cb_err != ESP_OK && data_cb_err != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(data_cb_err, TAG, "data cb");
        }
        esp_err_t a2d_init = esp_a2d_source_init();
        if (a2d_init != ESP_OK && a2d_init != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(a2d_init, TAG, "a2dp init");
        }
        ESP_RETURN_ON_ERROR(esp_bt_gap_set_device_name("Suit-BT"), TAG, "set name");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_bt_pin_code_t pin_code = { '0', '0', '0', '0' };
        esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
        s_stack_ready = true;
    }

    if (force_rescan){
        reset_devices();
        memset(s_peer_bda, 0, sizeof(s_peer_bda));
        s_peer_name[0] = '\0';
        return start_discovery();
    }
    return ESP_OK;
}

esp_err_t bt_audio_disconnect(void){
    if (!s_stack_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "bt_audio_disconnect: disconnect current peer");

    // Stop playback if needed
    if (s_streaming || s_user_streaming) {
        esp_err_t m = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        if (m != ESP_OK && m != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "media suspend before disconnect failed: %s", esp_err_to_name(m));
        }
        s_streaming = false;
        s_user_streaming = false;
    }

    bool have_peer = false;
    for (int i = 0; i < 6; ++i) {
        if (s_peer_bda[i] != 0) {
            have_peer = true;
            break;
        }
    }

    if (have_peer) {
        esp_err_t e = esp_a2d_source_disconnect(s_peer_bda);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE && e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "a2dp disconnect err: %s", esp_err_to_name(e));
        }
        // Let a2dp_cb() handle state when disconnect completes
        return e;
    }

    // No active link
    set_state(BT_AUDIO_STATE_IDLE);
    set_connection(SYS_CONN_DISCONNECTED);
    memset(s_peer_bda, 0, sizeof(s_peer_bda));
    s_peer_name[0] = '\0';
    return ESP_OK;
}

static void avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                       esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        // NOTE: in this ESP-IDF version the member is psth_cmd, not psth
        uint8_t key_code  = param->psth_cmd.key_code;
        uint8_t key_state = param->psth_cmd.key_state;

        ESP_LOGI(TAG, "AVRCP passthrough: code=%d state=%d", key_code, key_state);

        // Only act on key press, ignore key release/repeat
        if (key_state != ESP_AVRC_PT_CMD_STATE_PRESSED) {
            break;
        }

        switch (key_code) {
        case ESP_AVRC_PT_CMD_PLAY:
        case ESP_AVRC_PT_CMD_PAUSE:
            ESP_LOGI(TAG, "AVRCP: toggle play/pause");
            (void)audio_player_toggle_play_pause();
            break;

        case ESP_AVRC_PT_CMD_FORWARD:
            ESP_LOGI(TAG, "AVRCP: next track (unimplemented)");
            break;

        case ESP_AVRC_PT_CMD_BACKWARD:
            ESP_LOGI(TAG, "AVRCP: prev track (unimplemented)");
            break;

        case ESP_AVRC_PT_CMD_VOL_UP:
            ESP_LOGI(TAG, "AVRCP: vol up");
            bt_audio_volume_up();
            break;

        case ESP_AVRC_PT_CMD_VOL_DOWN:
            ESP_LOGI(TAG, "AVRCP: vol down");
            bt_audio_volume_down();
            break;

        default:
            ESP_LOGI(TAG, "AVRCP: unhandled key_code=%d", key_code);
            break;
        }
        break;
    }

    default:
        break;
    }
}

esp_err_t bt_audio_rescan(void){
    reset_devices();
    memset(s_peer_bda, 0, sizeof(s_peer_bda));
    s_peer_name[0] = '\0';
    s_user_streaming = false;
    return bt_audio_start(true);
}

esp_err_t bt_audio_scan(void){
    // Avoid scans while connected/streaming to prevent L2CAP congestion and audio glitches.
    if (s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING ||
        s_state == BT_AUDIO_STATE_CONNECTING) {
        ESP_LOGI(TAG, "skip scan: busy (state=%s)", bt_audio_state_name(s_state));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(bt_audio_start(false), TAG, "start");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    return start_discovery();
}

esp_err_t bt_audio_connect_index(int idx){
    if (idx < 0 || idx >= s_device_count) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_bd_addr_t target;
    memcpy(target, s_devices[idx].bda, sizeof(target));

    bool same_as_current =
        (memcmp(target, s_peer_bda, sizeof(target)) == 0) &&
        (s_state == BT_AUDIO_STATE_CONNECTED ||
         s_state == BT_AUDIO_STATE_STREAMING ||
         s_state == BT_AUDIO_STATE_CONNECTING);

    if (same_as_current) {
        ESP_LOGI(TAG, "Selected current device; disconnecting current peer");
        return bt_audio_disconnect();
    }

    bool busy = (s_state == BT_AUDIO_STATE_CONNECTED ||
                 s_state == BT_AUDIO_STATE_STREAMING ||
                 s_state == BT_AUDIO_STATE_CONNECTING);

    if (busy) {
        ESP_LOGI(TAG, "Switching peers: queue connect after disconnect");
        set_pending_connect(target);
        // Stop current stream before tearing down link to avoid congestion.
        (void)bt_audio_stop_stream();
        return bt_audio_disconnect();
    }

    ESP_LOGI(TAG, "Disconnecting other peers before connecting new one");
    disconnect_all_peers_except(target, true);

    s_user_streaming = false;
    s_streaming      = false;
    set_state(BT_AUDIO_STATE_IDLE);
    set_connection(SYS_CONN_DISCONNECTED);

    memcpy(s_peer_bda, target, sizeof(s_peer_bda));
    if (s_devices[idx].name[0]) {
        strncpy(s_peer_name, s_devices[idx].name, sizeof(s_peer_name) - 1);
        s_peer_name[sizeof(s_peer_name) - 1] = '\0';
    } else {
        s_peer_name[0] = '\0';
    }

    ESP_LOGI(TAG, "Connecting to device idx=%d", idx);
    return start_connect(target);
}

int bt_audio_get_devices(bt_audio_device_t *out, int max_out){
    if (!out || max_out <= 0) return 0;
    int n = s_device_count;
    if (n > max_out) n = max_out;
    memcpy(out, s_devices, n * sizeof(bt_audio_device_t));
    return n;
}

esp_err_t bt_audio_start_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_ERR_INVALID_STATE;
    }
    s_user_streaming = true;
    if (s_streaming) {
        set_state(BT_AUDIO_STATE_STREAMING);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), start skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_START;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
    if (err == ESP_OK){
        s_streaming = true;
        set_state(BT_AUDIO_STATE_STREAMING);
    } else {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
    }
    return err;
}

esp_err_t bt_audio_stop_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    bt_audio_stream_close();
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_OK;
    }
    s_user_streaming = false;
    if (!s_streaming) {
        set_state(BT_AUDIO_STATE_CONNECTED);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), suspend skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_SUSPEND;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    if (err != ESP_OK) {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
        return err;
    }
    s_streaming = false;
    set_state(BT_AUDIO_STATE_CONNECTED);
    return err;
}

esp_err_t bt_audio_pause_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_OK;
    }
    s_user_streaming = false;
    if (!s_streaming) {
        set_state(BT_AUDIO_STATE_CONNECTED);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), pause skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_SUSPEND;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    if (err != ESP_OK) {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
        return err;
    }
    s_streaming = false;
    set_state(BT_AUDIO_STATE_CONNECTED);
    return err;
}

int bt_audio_get_peer_sample_rate(void){
    return s_peer_sample_rate;
}

esp_err_t bt_audio_feed_pcm(const int16_t *stereo, size_t frames){
    (void)stereo; (void)frames;
    return ESP_ERR_NOT_SUPPORTED; // legacy API not used in file-backed mode
}

void bt_audio_get_status(bt_audio_status_t *out){
    if (!out) return;
    out->state = s_state;
    bool have_peer = false;
    for (int i = 0; i < 6; ++i) {
        if (s_peer_bda[i] != 0) { have_peer = true; break; }
    }
    out->has_peer = have_peer || (s_device_count > 0);
    memcpy(out->peer_bda, s_peer_bda, sizeof(out->peer_bda));
    strncpy(out->peer_name, s_peer_name, sizeof(out->peer_name) - 1);
    out->peer_name[sizeof(out->peer_name) - 1] = '\0';
    out->streaming = s_streaming;
    out->queued_frames = bt_audio_stream_queued_frames();
}

bool bt_audio_can_stream(void){
    return s_stack_ready &&
           (s_state == BT_AUDIO_STATE_CONNECTED || s_state == BT_AUDIO_STATE_STREAMING);
}

bool bt_audio_media_cmd_pending(void)
{
    return s_media_cmd_pending;
}

const char *bt_audio_state_name(bt_audio_state_t s){
    switch (s){
        case BT_AUDIO_STATE_DISABLED:     return "disabled";
        case BT_AUDIO_STATE_IDLE:         return "idle";
        case BT_AUDIO_STATE_STARTING:     return "starting";
        case BT_AUDIO_STATE_DISCOVERING:  return "scan";
        case BT_AUDIO_STATE_CONNECTING:   return "connecting";
        case BT_AUDIO_STATE_CONNECTED:    return "connected";
        case BT_AUDIO_STATE_STREAMING:    return "streaming";
        case BT_AUDIO_STATE_FAILED:       return "failed";
        default:                          return "?";
    }
}
