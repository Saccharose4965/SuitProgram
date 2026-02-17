#include "fft_render.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "oled.h"
#include "fft.h"   // NEW: for fft_get_confident_bpms & BPM range
#include "esp_timer.h"

static const float kDispPeakHold = 0.95f;  // decay for display peak tracking
static const float kDispPeakMin  = 1.0f;   // minimum display peak to avoid blowing up noise
static const float kHopRateNominalHz = (float)FFT_CFG_SAMPLE_RATE_HZ / (float)FFT_CFG_HOP_SAMPLES;

#ifndef FFT_RENDER_MAX_FPS
#define FFT_RENDER_MAX_FPS 20
#endif

// ------------------- Framebuffer helpers -------------------
static uint8_t g_fb[PANEL_W * PANEL_H / 8];
static inline void fb_clear(void){ memset(g_fb,0,sizeof(g_fb)); }
static inline void fb_pset(int x,int y){
    if ((unsigned)x>=PANEL_W||(unsigned)y>=PANEL_H) return;
    int idx = y*PANEL_W + x;
    g_fb[idx>>3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

// ------------------- Render state -------------------
static uint8_t g_spec_hist[PANEL_H][PANEL_W]; // spectrogram history (normalized 0..255)
static int     g_spec_col_idx = 0;
static float   g_disp_peak = 1.0f;     // running peak for display normalization (logmag)
static float   g_nov_disp_clean[PANEL_W];
static float   g_nov_disp_mean[PANEL_W];
static float   g_nov_disp_raw[PANEL_W];
static int     g_nov_disp_idx=0;
static int     g_nov_disp_count=0;
static float   g_tempo_spec_raw[PANEL_W];
static float   g_phase_corr[PANEL_W];
static float   g_phase_corr_peak = 1.0f;
static int64_t g_beat_flash_until_us = 0;
static volatile fft_view_t g_view_mode = FFT_VIEW_BPM_TEXT;
static bool g_render_enabled = true; // allow disabling OLED output
static fft_render_packet_t g_render_pkt; // avoid large per-task stack usage

static QueueHandle_t s_render_queue = NULL;
static TaskHandle_t s_render_task = NULL;
static SemaphoreHandle_t s_fb_mutex = NULL;

#define FFT_RENDER_CORE 1
#define FFT_RENDER_PRIO 3

// ------------------- Rendering -------------------
static void render_log_spectrum(const float *logmag);
static void render_spectrogram(void);
static void render_novelty_peaks(void);
static void render_flux_view(void);
static void render_tempo_raw(void);
static void render_beat_spectrum(const fft_render_packet_t *pkt);
static void render_phase_comb(void);
static void render_active_view(const fft_render_packet_t *pkt);
static void render_task(void *arg);

static void render_three_lines_fb(const char *l1, const char *l2, const char *l3){
    fb_clear();
    if (l1 && l1[0]) oled_draw_text3x5(g_fb, 0, 0, l1);
    if (l2 && l2[0]) oled_draw_text3x5(g_fb, 0, 10, l2);
    if (l3 && l3[0]) oled_draw_text3x5(g_fb, 0, 20, l3);
}

static void render_beat_spectrum(const fft_render_packet_t *pkt){
    char l1[32], l2[32], l3[32];

    // Beat flash marker (time-based so blink timing is independent of render FPS)
    char flash = ' ';
    if (esp_timer_get_time() <= g_beat_flash_until_us) flash = '*';

    // Try to get a list of confident BPMs
    float bpms[3];
    float confs[3];
    int n = fft_get_confident_bpms(bpms, confs, 3);

    if (n <= 0) {
        // --- Fallback: original single-BPM display ---

        float bpm_est = pkt ? pkt->bpm_est : 0.0f;
        if (!isfinite(bpm_est) || bpm_est < 0.0f) bpm_est = 0.0f;

        // Smooth the displayed BPM so it's readable
        static float s_bpm_display = 0.0f;
        const float alpha = 0.15f; // smaller = smoother, larger = more responsive

        if (s_bpm_display <= 0.0f)
            s_bpm_display = bpm_est;
        else
            s_bpm_display = (1.0f - alpha) * s_bpm_display + alpha * bpm_est;

        snprintf(l1, sizeof(l1), "BPM: %.1f", s_bpm_display);
        snprintf(l2, sizeof(l2), "conf:%.2f ph:%.2f",
                 pkt ? pkt->bpm_conf : 0.0f,
                 pkt ? pkt->bpm_phase : 0.0f);
        snprintf(l3, sizeof(l3), "B:%c", flash);

        render_three_lines_fb(l1, l2, l3);
        return;
    }

    // --- New: list of confident BPMs ---
    if (n > 3) n = 3;

    // Line 1: BPMs (ordered as provided: fundamental, double, special)
    size_t off = 0;
    off += snprintf(l1 + off, sizeof(l1) - off, "B:");
    for (int i = 0; i < n && off < sizeof(l1); ++i){
        off += snprintf(l1 + off, sizeof(l1) - off, " %5.1f", bpms[i]);
    }
    if (off < sizeof(l1) - 2){
        l1[off++] = ' ';
        l1[off++] = flash;
        l1[off]   = '\0';
    }

    // Line 2: confidences
    off = 0;
    off += snprintf(l2 + off, sizeof(l2) - off, "C:");
    for (int i = 0; i < n && off < sizeof(l2); ++i){
        off += snprintf(l2 + off, sizeof(l2) - off, " %4.2f", confs[i]);
    }
    l2[sizeof(l2) - 1] = '\0';

    // Line 3: phase + overall confidence
    snprintf(l3, sizeof(l3), "ph:%.2f conf:%.2f",
             pkt ? pkt->bpm_phase : 0.0f,
             pkt ? pkt->bpm_conf  : 0.0f);

    render_three_lines_fb(l1, l2, l3);
}

static void render_active_view(const fft_render_packet_t *pkt){
    switch (g_view_mode){
        case FFT_VIEW_SPECTRUM:       render_log_spectrum(pkt ? pkt->logmag : NULL); break;
        case FFT_VIEW_SPECTROGRAM:    render_spectrogram(); break;
        case FFT_VIEW_NOVELTY_PEAKS:  render_novelty_peaks(); break;
        case FFT_VIEW_FLUX:           render_flux_view(); break;
        case FFT_VIEW_TEMPO_RAW:      render_tempo_raw(); break;
        case FFT_VIEW_PHASE_COMB:     render_phase_comb(); break;
        case FFT_VIEW_BPM_TEXT:
        case FFT_VIEW_COUNT:
        default:                      render_beat_spectrum(pkt); break;
    }
}

// Draw a simple vertical bar graph for LOG magnitude spectrum (linear freq axis).
static void render_log_spectrum(const float *logmag){
    if (!logmag) return;

    static float cols[PANEL_W];
    float maxv = 1e-3f;
    const float step = ((float)(FFT_CFG_SIZE/2 + 1)) / (float)PANEL_W;
    for (int x=0; x<PANEL_W; ++x){
        int k0 = (int)floorf(x * step);
        int k1 = (int)floorf((x + 1) * step);
        if (k1 <= k0) k1 = k0 + 1;
        if (k1 > FFT_CFG_SIZE/2 + 1) k1 = FFT_CFG_SIZE/2 + 1;
        float acc=0.0f; int cnt=0;
        for (int k=k0; k<k1; ++k){ acc += logmag[k]; cnt++; }
        float v = (cnt>0) ? acc/(float)cnt : 0.0f;
        cols[x] = v;
        if (v > maxv) maxv = v;
    }
    fb_clear();
    float scale = g_disp_peak;
    if (scale < kDispPeakMin) scale = kDispPeakMin;
    for (int x=0; x<PANEL_W; ++x){
        float norm = cols[x] / scale;
        if (norm > 1.0f) norm = 1.0f;
        int h = (int)(norm * (float)(PANEL_H-1) + 0.5f);
        for (int y=0; y<=h; ++y) fb_pset(x, PANEL_H-1 - y);
    }
}

static void render_spectrogram(void){
    fb_clear();
    const uint8_t th = 128; // tweak for brightness vs noise (~0.5 in Q0.8)

    for (int x = 0; x < PANEL_W; ++x){
        int src = (g_spec_col_idx - 1 - x + PANEL_W) % PANEL_W;
        int px  = PANEL_W - 1 - x;

        for (int y = 0; y < PANEL_H; ++y){
            uint8_t vq = g_spec_hist[y][src];  // already downsampled for DISPLAY
            if (vq <= th) continue;

            int py = PANEL_H - 1 - y; // low freq at bottom
            fb_pset(px, py);
        }
    }
}

// Plot raw spectral flux as bars and its rolling mean as a dotted line (full 128px).
static void render_novelty_peaks(void){
    fb_clear();
    int groups = g_nov_disp_count;
    if (groups > PANEL_W) groups = PANEL_W;
    float maxv = 1e-3f;
    for (int g=0; g<groups; ++g){
        int idx = (g_nov_disp_idx - 1 - g + PANEL_W) % PANEL_W;
        float raw  = g_nov_disp_raw[idx];
        float mean = g_nov_disp_mean[idx];
        if (raw > maxv) maxv = raw;
        if (mean > maxv) maxv = mean;
    }
    float scale = (maxv > 1e-6f) ? maxv : 1e-6f;
    for (int g=0; g<groups; ++g){
        int x = PANEL_W - 1 - g; // newest on the right
        int idx = (g_nov_disp_idx - 1 - g + PANEL_W) % PANEL_W;
        float raw  = g_nov_disp_raw[idx];
        float mean = g_nov_disp_mean[idx];
        int y_mean  = PANEL_H-1 - (int)((mean / scale) * (float)(PANEL_H-1) + 0.5f);
        int h_raw   = (int)((raw  / scale) * (float)(PANEL_H-1) + 0.5f);
        int y_raw   = PANEL_H-1 - h_raw;
        if (y_mean  < 0) y_mean = 0;
        if (y_raw < 0)  y_raw  = 0;
        // Raw (positive novelty) as filled bar
        for (int y=PANEL_H-1; y>=y_raw; --y) fb_pset(x, y);
        // Local mean as a dotted marker (every other pixel)
        if ((g & 1) == 0) fb_pset(x, y_mean);
    }
}

// Plot cleaned novelty (flux): raw - mean, clamped at 0, as filled bars.
static void render_flux_view(void){
    fb_clear();

    int groups = g_nov_disp_count;
    if (groups > PANEL_W) groups = PANEL_W;

    // Find instantaneous max of cleaned flux over visible history
    float maxv = 1e-3f;
    for (int g = 0; g < groups; ++g){
        int idx   = (g_nov_disp_idx - 1 - g + PANEL_W) % PANEL_W;
        float clean = g_nov_disp_clean[idx];   // already raw - mean, clamped to zero in push_novelty
        if (clean > maxv) maxv = clean;
    }

    float scale = (maxv > 1e-6f) ? maxv : 1e-6f;

    // Draw cleaned flux as filled bars from the bottom (newest on the right)
    for (int g = 0; g < groups; ++g){
        int x   = PANEL_W - 1 - g; // newest on the right
        int idx = (g_nov_disp_idx - 1 - g + PANEL_W) % PANEL_W;

        float clean = g_nov_disp_clean[idx];
        int h_clean = (int)((clean / scale) * (float)(PANEL_H - 1) + 0.5f);
        int y_clean = PANEL_H - 1 - h_clean;
        if (y_clean < 0) y_clean = 0;

        // Filled bar for flux
        for (int y = PANEL_H - 1; y >= y_clean; --y){
            fb_pset(x, y);
        }
    }

}

// Show raw temporal FFT spectrum (no capacitor smoothing).
static void render_tempo_raw(void){
    fb_clear();
    float maxv = 1e-3f;
    for (int x=0; x<PANEL_W; ++x){
        float v = g_tempo_spec_raw[x];
        if (v > maxv) maxv = v;
    }
    for (int x=0; x<PANEL_W; ++x){
        float norm = (maxv > 0.0f) ? (g_tempo_spec_raw[x] / maxv) : 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        int h = (int)(norm * (float)(PANEL_H-1) + 0.5f);
        for (int y=0; y<=h; ++y) fb_pset(x, PANEL_H-1 - y);
    }
}

// Comb correlation vs phase: for each phase offset (0..1), correlate novelty with a smooth comb.
static void render_phase_comb(void){
    fb_clear();

    float maxv = g_phase_corr_peak;
    if (maxv < 1e-6f) return;

    // Static plot: phase 0..1 maps directly left→right
    for (int x = 0; x < PANEL_W; ++x){
        float norm = g_phase_corr[x] / maxv;
        if (norm > 1.0f) norm = 1.0f;
        if (norm < 0.0f) norm = 0.0f;
        int h = (int)(norm * (float)(PANEL_H - 1) + 0.5f);
        for (int y = 0; y <= h; ++y){
            fb_pset(x, PANEL_H - 1 - y);
        }
    }

}

static void render_task(void *arg){
    (void)arg;
    const int64_t min_frame_us = 1000000LL / (int64_t)FFT_RENDER_MAX_FPS;
    int64_t last_draw_us = 0;
    while (1){
        if (!s_render_queue){
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (xQueueReceive(s_render_queue, &g_render_pkt, portMAX_DELAY) == pdTRUE){
            int64_t now_us = esp_timer_get_time();
            if (last_draw_us != 0 && (now_us - last_draw_us) < min_frame_us){
                continue; // drop intermediate frames; queue is overwrite-based
            }
            if (g_render_enabled){
                if (s_fb_mutex) xSemaphoreTake(s_fb_mutex, portMAX_DELAY);
                render_active_view(&g_render_pkt);
                if (s_fb_mutex) xSemaphoreGive(s_fb_mutex);
                last_draw_us = esp_timer_get_time();
            }
        }
    }
}

// ------------------- Public interface -------------------
void fft_render_push_spectrogram(const float *logmag){
    if (!logmag) return;

    float scale = g_disp_peak;
    if (scale < kDispPeakMin) scale = kDispPeakMin;

    const int num_bins     = FFT_CFG_SIZE / 2 + 1; // full resolution
    const int bins_per_row = 4;                // DISPLAY: 2 FFT bins -> 1 pixel row
    const int max_covered  = PANEL_H * bins_per_row;    // max bins we can show
    const int usable_bins  = (num_bins < max_covered) ? num_bins : max_covered;
    int rows               = (usable_bins + bins_per_row - 1) / bins_per_row; // ceil
    if (rows > PANEL_H) rows = PANEL_H;

    for (int y = 0; y < rows; ++y){
        int b0 = y * bins_per_row;
        int b1 = b0 + bins_per_row - 1;
        if (b0 >= usable_bins) {
            g_spec_hist[y][g_spec_col_idx] = 0.0f;
            continue;
        }
        if (b1 >= usable_bins) b1 = usable_bins - 1;

        float acc = 0.0f;
        int   cnt = 0;
        for (int k = b0; k <= b1; ++k){
            acc += logmag[k];   // full-res data, only averaged for DISPLAY
            cnt++;
        }
        float v = (cnt > 0) ? acc / (float)cnt : 0.0f;
        float norm = (scale > 0.0f) ? (v / scale) : 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        if (norm < 0.0f) norm = 0.0f;
        uint8_t q = (uint8_t)(norm * 255.0f + 0.5f);
        g_spec_hist[y][g_spec_col_idx] = q;
    }

    // Clear any rows beyond the highest mapped one
    for (int y = rows; y < PANEL_H; ++y){
        g_spec_hist[y][g_spec_col_idx] = 0;
    }

    g_spec_col_idx = (g_spec_col_idx + 1) % PANEL_W;

    // Update running display peak for spectrum/spectrogram
    float frame_peak = 1e-3f;
    for (int k = 0; k <= FFT_CFG_SIZE/2; ++k){
        if (logmag[k] > frame_peak) frame_peak = logmag[k];
    }
    g_disp_peak *= kDispPeakHold;
    if (frame_peak > g_disp_peak) g_disp_peak = frame_peak;
    if (g_disp_peak < kDispPeakMin) g_disp_peak = kDispPeakMin;
}

void fft_render_push_novelty_display(float raw, float local_mean, float cleaned){
    g_nov_disp_raw[g_nov_disp_idx]   = raw;
    g_nov_disp_mean[g_nov_disp_idx]  = local_mean;
    g_nov_disp_clean[g_nov_disp_idx] = cleaned;

    g_nov_disp_idx = (g_nov_disp_idx + 1) % PANEL_W;
    if (g_nov_disp_count < PANEL_W) g_nov_disp_count++;
}

void fft_render_suppress_recent_novelty(int frames){
    if (frames <= 0 || g_nov_disp_count <= 0) return;
    if (frames > g_nov_disp_count) frames = g_nov_disp_count;

    for (int i = 0; i < frames; ++i){
        int idx = g_nov_disp_idx - 1 - i;
        while (idx < 0) idx += PANEL_W;
        g_nov_disp_raw[idx]   = 0.0f;
        g_nov_disp_mean[idx]  = 0.0f;
        g_nov_disp_clean[idx] = 0.0f;
    }
}

void fft_render_update_tempo_spectrum(const float *bpm_spec_bc){
    const int bpm_count = FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1;
    const int disp_bpm_count = FFT_CFG_TARGET_BPM_MAX - FFT_CFG_TARGET_BPM_MIN + 1;

    if (!bpm_spec_bc){
        memset(g_tempo_spec_raw, 0, sizeof(g_tempo_spec_raw));
        return;
    }

    for (int x=0; x < PANEL_W; ++x){
        // Display only 64..127 BPM; on 128px panels this gives exactly 2 px per BPM.
        int disp_idx = (PANEL_W > 0) ? (x * disp_bpm_count) / PANEL_W : 0;
        if (disp_idx < 0) disp_idx = 0;
        if (disp_idx >= disp_bpm_count) disp_idx = disp_bpm_count - 1;

        int bpm = FFT_CFG_TARGET_BPM_MIN + disp_idx;
        int bin = bpm - FFT_CFG_BPM_MIN;

        if (bin < 0) bin = 0;
        if (bin >= bpm_count) bin = bpm_count - 1;

        float val = bpm_spec_bc[bin];
        g_tempo_spec_raw[x] = val;
    }
}

void fft_render_update_phase_curve(const float *vals, int count){
    if (!vals || count <= 0){
        memset(g_phase_corr, 0, sizeof(g_phase_corr));
        g_phase_corr_peak = 1e-6f;
        return;
    }

    float maxv = 0.0f;
    for (int i = 0; i < count; ++i){
        if (vals[i] > maxv) maxv = vals[i];
    }
    if (maxv < 1e-6f) maxv = 1e-6f;
    g_phase_corr_peak = maxv;

    for (int x = 0; x < PANEL_W; ++x){
        float t = (PANEL_W > 1) ? ((float)x / (float)(PANEL_W - 1)) : 0.0f;
        float pos = t * (float)(count - 1);
        int i0 = (int)floorf(pos);
        int i1 = i0 + 1;
        if (i1 >= count) i1 = count - 1;
        float frac = pos - (float)i0;
        float v0 = vals[i0];
        float v1 = vals[i1];
        float v = v0 * (1.0f - frac) + v1 * frac;
        g_phase_corr[x] = v;
    }
}

void fft_render_trigger_flash(uint32_t flash_frames){
    if (flash_frames == 0) flash_frames = 1;

    int64_t now_us = esp_timer_get_time();
    int64_t dur_us = (int64_t)lroundf(((float)flash_frames * 1000000.0f) / kHopRateNominalHz);
    // Keep marker visible long enough so render FPS jitter doesn't drop beats visually.
    if (dur_us < 120000) dur_us = 120000;

    int64_t until_us = now_us + dur_us;
    if (until_us > g_beat_flash_until_us){
        g_beat_flash_until_us = until_us;
    }
}

void fft_render_set_view(fft_view_t view){
    if (view >= FFT_VIEW_COUNT) {
        g_view_mode = FFT_VIEW_BPM_TEXT;
        return;
    }
    g_view_mode = view;
}

void fft_render_set_display_enabled(bool enabled){
    g_render_enabled = enabled;
}

esp_err_t fft_render_init(void){
    if (!s_fb_mutex){
        s_fb_mutex = xSemaphoreCreateMutex();
        if (!s_fb_mutex) return ESP_ERR_NO_MEM;
    }
    if (!s_render_queue){
        s_render_queue = xQueueCreate(1, sizeof(fft_render_packet_t));
        if (!s_render_queue) return ESP_ERR_NO_MEM;
    }
    if (!s_render_task){
        BaseType_t rt = xTaskCreatePinnedToCore(render_task, "fft_render", 6144, NULL,
                                                FFT_RENDER_PRIO, &s_render_task, FFT_RENDER_CORE);
        if (rt != pdPASS) return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void fft_render_submit(const fft_render_packet_t *pkt){
    if (!pkt) return;

    if (s_render_queue){
        xQueueOverwrite(s_render_queue, pkt);
    } else if (g_render_enabled){
        if (s_fb_mutex) xSemaphoreTake(s_fb_mutex, portMAX_DELAY);
        render_active_view(pkt);
        if (s_fb_mutex) xSemaphoreGive(s_fb_mutex);
    }
}

void fft_render_copy_frame(uint8_t *dst_fb, size_t dst_len){
    if (!dst_fb || dst_len < sizeof(g_fb)) return;

    if (s_fb_mutex){
        if (xSemaphoreTake(s_fb_mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
    }
    memcpy(dst_fb, g_fb, sizeof(g_fb));
    if (s_fb_mutex){
        xSemaphoreGive(s_fb_mutex);
    }
}
