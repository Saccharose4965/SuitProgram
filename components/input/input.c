#include "input.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"

#include "hw.h"
#include "fft.h"

static const char *TAG = "input";
#define INPUT_LOG_EVENTS 0
#define INPUT_AUDIO_HOLE_FRAMES 6
#define INPUT_DEBOUNCE_MS 25
#define INPUT_LONG_PRESS_MS 1200
#define INPUT_LEVEL_TOL_MV 220
#define INPUT_LEVEL_A_MV 825
#define INPUT_LEVEL_AB_MV 1100
#define INPUT_LEVEL_B_MV 1650
#define INPUT_LEVEL_BC_MV 2200
#define INPUT_LEVEL_C_MV 2475
#define INPUT_LEVEL_D_MV 3300

static input_button_t s_prev_btn = INPUT_BTN_NONE;
static TickType_t     s_prev_tick = 0;
static bool           s_long_sent = false;

static input_button_t s_candidate_btn = INPUT_BTN_NONE;
static TickType_t     s_candidate_tick = 0;

static inline void consider_level(int mv,
                                  int target_mv,
                                  int target_tol,
                                  input_button_t target_btn,
                                  int *best_diff,
                                  input_button_t *best_btn)
{
    int diff = target_mv - mv;
    if (diff < 0) diff = -diff;
    if (diff <= target_tol && diff < *best_diff) {
        *best_diff = diff;
        *best_btn = target_btn;
    }
}

static input_button_t decode_mv(int mv)
{
    int best_diff = 1000000;
    input_button_t best = INPUT_BTN_NONE;

    consider_level(mv, INPUT_LEVEL_AB_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_AB_COMBO, &best_diff, &best);
    consider_level(mv, INPUT_LEVEL_BC_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_BC_COMBO, &best_diff, &best);
    consider_level(mv, INPUT_LEVEL_D_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_D, &best_diff, &best);
    consider_level(mv, INPUT_LEVEL_C_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_C, &best_diff, &best);
    consider_level(mv, INPUT_LEVEL_B_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_B, &best_diff, &best);
    consider_level(mv, INPUT_LEVEL_A_MV, INPUT_LEVEL_TOL_MV, INPUT_BTN_A, &best_diff, &best);

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

void input_init(void)
{
    // Ensure ADC is ready for ladder sampling.
    (void)hw_adc1_init_default();
    s_prev_btn = INPUT_BTN_NONE;
    s_prev_tick = xTaskGetTickCount();
    s_long_sent = false;
    s_candidate_btn = INPUT_BTN_NONE;
    s_candidate_tick = s_prev_tick;
}

bool input_poll(input_event_t *out_event, TickType_t now_ticks)
{
    if (!out_event) return false;

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

    // Debounce the decoded ladder level so brief ADC jitter doesn't create
    // synthetic event churn. This allows direct A->B transitions when
    // they are truly stable, without enforcing an intermediate NONE sample.
    TickType_t settle_ticks = pdMS_TO_TICKS(INPUT_DEBOUNCE_MS);
    if (cur != s_candidate_btn) {
        s_candidate_btn = cur;
        s_candidate_tick = now_ticks;
        cur = s_prev_btn;
    } else if ((now_ticks - s_candidate_tick) < settle_ticks) {
        cur = s_prev_btn;
    }


    if (cur != s_prev_btn) {
        // Any ladder transition means input activity; drop a short novelty
        // window so button noise does not pollute beat tracking.
        fft_punch_novelty_hole(INPUT_AUDIO_HOLE_FRAMES);

        // Long-press timing always restarts when the logical button changes.
        s_prev_tick = now_ticks;
        s_long_sent = false;

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
        TickType_t long_ticks = pdMS_TO_TICKS(INPUT_LONG_PRESS_MS);
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
