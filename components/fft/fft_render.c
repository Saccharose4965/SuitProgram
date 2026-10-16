#include "fft_render.h"

#include "esp_attr.h"
#include "freertos/semphr.h"
#include "oled.h"
#include <string.h>

_Static_assert(PANEL_W == FFT_DISPLAY_WIDTH && PANEL_H == FFT_DISPLAY_HEIGHT,
               "FFT framebuffer must match OLED");
static EXT_RAM_BSS_ATTR fft_display_t s_display;
// Drawing is shell-task-only. Capture only holds the mutex for a short copy.
static EXT_RAM_BSS_ATTR fft_display_t s_draw_snapshot;
static EXT_RAM_BSS_ATTR uint8_t s_cached_frame[FFT_DISPLAY_BYTES];
static uint32_t s_generation, s_draw_generation;
static fft_view_t s_draw_view;
static bool s_have_frame;
static StaticSemaphore_t s_mutex_storage;
static SemaphoreHandle_t s_mutex;
static portMUX_TYPE s_control_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_enabled;
static fft_view_t s_view = FFT_VIEW_SPECTRUM;

esp_err_t fft_render_init(void)
{
    if (!s_mutex) s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    return s_mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

void fft_render_reset(void)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fft_display_reset(&s_display);
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

void fft_render_update_detector(const detector_result_t *result)
{
    if (!s_mutex || !result) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_display.detector = *result;
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

void fft_render_submit(const fft_render_packet_t *pkt)
{
    if (!s_mutex || !pkt) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fft_display_push(&s_display, pkt);
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

void fft_render_set_view(fft_view_t view)
{
    portENTER_CRITICAL(&s_control_mux);
    s_view = (unsigned)view < FFT_VIEW_COUNT ? view : FFT_VIEW_SPECTRUM;
    portEXIT_CRITICAL(&s_control_mux);
}

void fft_render_set_display_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_control_mux);
    s_enabled = enabled;
    portEXIT_CRITICAL(&s_control_mux);
}

bool fft_render_is_display_enabled(void)
{
    portENTER_CRITICAL(&s_control_mux);
    bool enabled = s_enabled;
    portEXIT_CRITICAL(&s_control_mux);
    return enabled;
}

void fft_render_copy_frame(uint8_t *dst, size_t len)
{
    if (!dst || len < FFT_DISPLAY_BYTES || !s_mutex) return;
    portENTER_CRITICAL(&s_control_mux);
    fft_view_t view = s_view;
    portEXIT_CRITICAL(&s_control_mux);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t generation = s_generation;
    bool changed = !s_have_frame || generation != s_draw_generation || view != s_draw_view;
    if (changed) memcpy(&s_draw_snapshot, &s_display, sizeof(s_display));
    xSemaphoreGive(s_mutex);
    if (changed) {
        fft_display_draw(&s_draw_snapshot, view, s_cached_frame);
        s_draw_generation = generation;
        s_draw_view = view;
        s_have_frame = true;
    }
    memcpy(dst, s_cached_frame, FFT_DISPLAY_BYTES);
}
