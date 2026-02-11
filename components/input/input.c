#include "input.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"

#include "hw.h"
#include "fft.h"

static const char *TAG = "input";
#define INPUT_LOG_EVENTS 0
#define INPUT_AUDIO_HOLE_FRAMES 6

static input_config_t s_cfg = {
    .long_press_ms = 800,
    .combo_mv = 0,         // disabled by default until ladder supports combo step
    .combo_tol_mv = 120,
    .combo_verify_ms = 40,
};

static input_button_t s_prev_btn = INPUT_BTN_NONE;
static TickType_t     s_prev_tick = 0;
static bool           s_long_sent = false;

static bool           s_has_buffered = false;
static input_event_t  s_buffered;
static bool           s_combo_pending = false;
static TickType_t     s_combo_pending_tick = 0;

static bool allow_combo_transition(input_button_t prev, input_button_t cur)
{
    if (prev == INPUT_BTN_A && cur == INPUT_BTN_TOP_COMBO) return true;
    if (prev == INPUT_BTN_B && cur == INPUT_BTN_TOP_COMBO) return true;
    return false;
}

static input_button_t decode_mv(int mv)
{
    int combo_tol = s_cfg.combo_tol_mv > 0 ? s_cfg.combo_tol_mv : 200;
    const int tol_single = 220;
    const int tol_fast = 140; // narrower so it doesn't steal B/C

    const struct {
        int mv;
        input_button_t btn;
        int tol;
    } targets[] = {
        { s_cfg.combo_mv > 0 ? s_cfg.combo_mv : 1100, INPUT_BTN_TOP_COMBO, combo_tol },
        { 2200, INPUT_BTN_FAST_FALL, tol_fast },
        { 3300, INPUT_BTN_D, tol_single },
        { 2475, INPUT_BTN_C, tol_single },
        { 1650, INPUT_BTN_B, tol_single },
        { 825,  INPUT_BTN_A, tol_single },
    };

    int best_diff = 1000000;
    input_button_t best = INPUT_BTN_NONE;
    for (size_t i = 0; i < sizeof(targets)/sizeof(targets[0]); ++i) {
        int diff = targets[i].mv - mv;
        if (diff < 0) diff = -diff;
        if (diff <= targets[i].tol && diff < best_diff) {
            best_diff = diff;
            best = targets[i].btn;
        }
    }
    return best;
}

bool input_sample(input_button_t *out_btn, int *out_mv)
{
    if (!out_btn || !out_mv) return false;
    int mv = 0;
    if (hw_adc1_read_mv(HW_BUTTONS_ADC_CH, &mv) != ESP_OK) {
        return false;
    }
    *out_mv = mv;
    *out_btn = decode_mv(mv);
    return true;
}

void input_init(const input_config_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
    }
    // Ensure ADC is ready for ladder sampling.
    (void)hw_adc1_init_default();
    s_prev_btn = INPUT_BTN_NONE;
    s_prev_tick = xTaskGetTickCount();
    s_long_sent = false;
    s_has_buffered = false;
    s_combo_pending = false;
    s_combo_pending_tick = 0;
}

static bool maybe_emit_pending(input_event_t *out_event)
{
    if (s_has_buffered && out_event) {
        *out_event = s_buffered;
        s_has_buffered = false;
        return true;
    }
    return false;
}

bool input_poll(input_event_t *out_event, TickType_t now_ticks)
{
    if (!out_event) return false;

    if (maybe_emit_pending(out_event)) {
        return true;
    }

    if (now_ticks == 0) {
        now_ticks = xTaskGetTickCount();
    }

    int mv = 0;
    input_button_t cur = INPUT_BTN_NONE;
    if (!input_sample(&cur, &mv)) {
        ESP_LOGW(TAG, "input_sample failed");
        return false;
    }
#if INPUT_LOG_EVENTS
    static uint32_t s_sample_count = 0;
    if ((s_sample_count++ % 15u) == 0u) { // ~2 Hz at 33 ms tick
        ESP_LOGI(TAG, "sample: mv=%d btn=%d prev=%d", mv, cur, s_prev_btn);
    }
#endif

    // Combo verification: require it to be stable before emitting
    if (cur == INPUT_BTN_TOP_COMBO && s_cfg.combo_verify_ms > 0) {
        TickType_t verify_ticks = pdMS_TO_TICKS(s_cfg.combo_verify_ms);
        if (!s_combo_pending && s_prev_btn != INPUT_BTN_TOP_COMBO) {
            s_combo_pending = true;
            s_combo_pending_tick = now_ticks;
        }
        if (s_combo_pending && (now_ticks - s_combo_pending_tick) < verify_ticks) {
            cur = s_prev_btn; // hold previous until verified
        } else {
            s_combo_pending = false;
        }
    } else {
        s_combo_pending = false;
    }

    // If we were in combo, require a return to NONE before other buttons can fire
    if (s_prev_btn == INPUT_BTN_TOP_COMBO && cur != INPUT_BTN_NONE) {
        cur = s_prev_btn;
    }

    // Require a return to NONE between distinct buttons so a held press can't
    // trigger another, except when A/B transitions into the verified top combo.
    if (s_prev_btn != INPUT_BTN_NONE &&
        cur != INPUT_BTN_NONE &&
        cur != s_prev_btn &&
        !allow_combo_transition(s_prev_btn, cur)) {
        cur = s_prev_btn;
    }


    if (cur != s_prev_btn) {
        // Any ladder transition means press/release activity; drop a short
        // novelty window so button noise does not pollute beat tracking.
        fft_punch_novelty_hole(INPUT_AUDIO_HOLE_FRAMES);

        // On change, emit release first, then buffer new press.
        s_prev_tick = now_ticks;
        s_long_sent = false;

        if (s_prev_btn != INPUT_BTN_NONE) {
            out_event->button   = s_prev_btn;
            out_event->type     = INPUT_EVENT_RELEASE;
            out_event->at_ticks = now_ticks;
#if INPUT_LOG_EVENTS
            ESP_LOGI(TAG, "event: release %d (mv=%d)", out_event->button, mv);
#endif
            s_prev_btn = cur;
            if (cur != INPUT_BTN_NONE) {
                s_buffered.button   = cur;
                s_buffered.type     = INPUT_EVENT_PRESS;
                s_buffered.at_ticks = now_ticks;
                s_has_buffered = true;
            }
            return true;
        }

        s_prev_btn = cur;
        if (cur != INPUT_BTN_NONE) {
            out_event->button   = cur;
            out_event->type     = INPUT_EVENT_PRESS;
            out_event->at_ticks = now_ticks;
#if INPUT_LOG_EVENTS
            ESP_LOGI(TAG, "event: press %d (mv=%d)", out_event->button, mv);
#endif
            return true;
        }
        return false;
    }

    if (cur != INPUT_BTN_NONE && !s_long_sent) {
        TickType_t held_ticks = now_ticks - s_prev_tick;
        TickType_t long_ticks = pdMS_TO_TICKS(s_cfg.long_press_ms);
        if (held_ticks >= long_ticks) {
            s_long_sent = true;
            out_event->button   = cur;
            out_event->type     = INPUT_EVENT_LONG_PRESS;
            out_event->at_ticks = now_ticks;
#if INPUT_LOG_EVENTS
            ESP_LOGI(TAG, "event: long %d (mv=%d)", out_event->button, mv);
#endif
            return true;
        }
    }

    return false;
}
