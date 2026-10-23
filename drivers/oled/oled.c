#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "oled.h"
#include "hw.h"
#include "sdkconfig.h"


// ==== SSD130x low-level ====
static spi_device_handle_t s_oled;
static SemaphoreHandle_t s_oled_lock = NULL;
static bool s_oled_inited = false;
static bool s_oled_prev_valid = false;
static uint8_t s_oled_prev_pages[PANEL_H / 8][PANEL_W];

static inline void dc_cmd(void){
    gpio_set_level(PIN_DC_OLED, 0);
}
static inline void dc_data(void){
    gpio_set_level(PIN_DC_OLED, 1);
}

static void tx(const void* data, size_t len, bool is_data){
    if (!s_oled_lock) {
        static StaticSemaphore_t buf;
        s_oled_lock = xSemaphoreCreateMutexStatic(&buf);
    }
    if (s_oled_lock) xSemaphoreTake(s_oled_lock, portMAX_DELAY);

    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    if (is_data) dc_data(); else dc_cmd();
    (void)spi_device_transmit(s_oled, &t);

    if (s_oled_lock) xSemaphoreGive(s_oled_lock);
}
static inline void cmd1(uint8_t c){ tx(&c, 1, false); }
static inline void cmd2(uint8_t c, uint8_t v){ uint8_t b[2]={c,v}; tx(b, 2, false); }

static void hw_reset(void){
    if (PIN_RST_OLED == GPIO_NUM_NC) return;
    gpio_set_level(PIN_RST_OLED, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST_OLED, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static inline void begin_window(int x0, int x1, int page0, int page1){
    cmd1(0x21); cmd1(x0); cmd1(x1);
    cmd1(0x22); cmd1(page0); cmd1(page1);
}

// ==== Public API ====
void oled_init(void)
{
    if (s_oled_inited) return;

    // Shared SPI2 bus
    ESP_ERROR_CHECK(hw_spi2_init_once());
    vTaskDelay(pdMS_TO_TICKS(100));  // power-on settle

    // DC / CS (/RST) as outputs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_DC_OLED)
                      | (1ULL << PIN_CS_OLED)
                      | ((PIN_RST_OLED != GPIO_NUM_NC) ? (1ULL << PIN_RST_OLED) : 0),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(PIN_DC_OLED, 0);

    // Attach device on shared bus — mode 0
    spi_device_handle_t dev = NULL;
    ESP_ERROR_CHECK(hw_spi2_add_oled(&dev));
    s_oled = dev;

    hw_reset();

    // SSD1306/1309 init (128x64)
    cmd1(0xAE);                  // display off
    vTaskDelay(pdMS_TO_TICKS(2));
    cmd2(0xD5, 0x80);            // clock div
    cmd2(0xA8, PANEL_H - 1);     // multiplex
    cmd2(0xD3, 0x00);            // offset
    cmd1(0x40);                  // start line (0)
    cmd2(0x8D, 0x14);            // charge pump on (works for most SSD1306/1309 boards)
    cmd2(0x20, 0x00);            // memory mode: horizontal
    cmd1(0xA1);                  // seg remap
    cmd1(0xC8);                  // COM scan dec
    cmd2(0xDA, (PANEL_H==64)?0x12:0x02); // COM pins
    cmd2(0x81, 0xCF);            // contrast
    cmd2(0xD9, 0xF1);            // precharge
    cmd2(0xDB, 0x40);            // VCOM detect
    cmd1(0xA4);                  // resume from all-on
    cmd1(0xA6);                  // normal display
    cmd1(0xAF);                  // display ON
    vTaskDelay(pdMS_TO_TICKS(50)); // let panel wake up
    s_oled_prev_valid = false;
    s_oled_inited = true;
}


void oled_clear(void){
    begin_window(0, PANEL_W-1, 0, (PANEL_H/8)-1);
    static uint8_t zeros[128] = {0};
    for (int page=0; page<PANEL_H/8; ++page){
        tx(zeros, PANEL_W, true);
    }
    memset(s_oled_prev_pages, 0, sizeof(s_oled_prev_pages));
    s_oled_prev_valid = true;
}

// ---- Full-screen 128x64 blit (buf is PANEL_W*PANEL_H/8) ----
void oled_blit_full(const uint8_t* buf)
{
    if (!buf) return;
#ifndef CONFIG_SUIT_OLED_PARTIAL_UPDATES
    // Refresh the full panel, including unchanged/cleared pixels. One address
    // window and page-sized transfers avoid a large task-stack framebuffer.
    begin_window(0, PANEL_W-1, 0, PANEL_H/8-1);
#endif
    uint8_t line[PANEL_W];
    for (int p = 0; p < PANEL_H/8; ++p) {
        for (int x = 0; x < PANEL_W; ++x) {
            uint8_t b = 0;
            for (int bit = 0; bit < 8; ++bit) {
                int y = p*8 + bit;
                int idx = y*PANEL_W + x;
                uint8_t v = (buf[idx>>3] >> (7 - (idx & 7))) & 1u;
                b |= (uint8_t)(v << bit);
            }
            line[x] = b;
        }

#ifndef CONFIG_SUIT_OLED_PARTIAL_UPDATES
        tx(line, PANEL_W, true);
        continue;
#endif

        if (!s_oled_prev_valid) {
            begin_window(0, PANEL_W - 1, p, p);
            tx(line, PANEL_W, true);
            memcpy(s_oled_prev_pages[p], line, PANEL_W);
            continue;
        }

        int changed = 0;
        int runs = 0;
        bool in_run = false;
        for (int x = 0; x < PANEL_W; ++x) {
            bool diff = (line[x] != s_oled_prev_pages[p][x]);
            if (diff) {
                ++changed;
                if (!in_run) {
                    in_run = true;
                    ++runs;
                }
            } else {
                in_run = false;
            }
        }
        if (changed == 0) {
            continue;
        }

        // If a lot changed, send whole page (fewer SPI commands, less overhead).
        if (changed > (PANEL_W / 2) || runs > 10) {
            begin_window(0, PANEL_W - 1, p, p);
            tx(line, PANEL_W, true);
            memcpy(s_oled_prev_pages[p], line, PANEL_W);
            continue;
        }

        int run_start = -1;
        for (int x = 0; x < PANEL_W; ++x) {
            bool diff = (line[x] != s_oled_prev_pages[p][x]);
            if (diff) {
                s_oled_prev_pages[p][x] = line[x];
                if (run_start < 0) run_start = x;
            } else if (run_start >= 0) {
                int run_end = x - 1;
                begin_window(run_start, run_end, p, p);
                tx(&line[run_start], (size_t)(run_end - run_start + 1), true);
                run_start = -1;
            }
        }
        if (run_start >= 0) {
            int run_end = PANEL_W - 1;
            begin_window(run_start, run_end, p, p);
            tx(&line[run_start], (size_t)(run_end - run_start + 1), true);
        }
    }
#ifdef CONFIG_SUIT_OLED_PARTIAL_UPDATES
    s_oled_prev_valid = true;
#else
    s_oled_prev_valid = false;
#endif
}

// ---- 64x64 centered blit (buf is 64x64) ----
void oled_blit64_center(const uint8_t* buf){
    enum{W = 64, H = 64};
    int offX = (PANEL_W - W)/2; if (offX<0) offX=0;
    int offY = (PANEL_H - H)/2; if (offY<0) offY=0;
    int pageStart = offY/8;

    begin_window(offX, offX + W - 1, pageStart, pageStart + (H/8) - 1);

    uint8_t line[W];
    for (int p=0; p<H/8; ++p){
        for (int x=0; x<W; ++x){
            uint8_t b=0;
            for (int bit=0; bit<8; ++bit){
                int y = p*8 + bit;
                int idx = y*W + x;
                uint8_t v = (buf[idx>>3] >> (7-(idx&7))) & 1u;
                b |= (uint8_t)(v << bit);
            }
            line[x]=b;
        }
        tx(line, W, true);
    }
}

// ---- 64x64 blit with horizontal offset (buf is 64x64) ----
void oled_blit64_offset(const uint8_t* buf, int xoff_px){
    enum { W = 64, H = 64 };
    int baseX = (PANEL_W - W)/2;
    int colStart = baseX + xoff_px;
    int colEnd   = colStart + W - 1;
    int srcX0 = 0;
    int drawW = 0;

    if (!(colEnd < 0 || colStart > (PANEL_W - 1))) {
        if (colStart < 0) { srcX0 = -colStart; colStart = 0; }
        drawW = W - srcX0;
        if (colEnd >= PANEL_W) { drawW -= (colEnd - (PANEL_W - 1)); colEnd = PANEL_W - 1; }
        if (drawW < 0) drawW = 0;
    }

    int offY = (PANEL_H - H)/2;
    int pageStart = offY / 8;

    // Sliding needs explicit background clear behind the logo, so redraw the
    // full 128-column page span instead of only the current visible bbox.
    begin_window(0, PANEL_W - 1, pageStart, pageStart + (H/8) - 1);

    uint8_t line[PANEL_W];
    for (int p=0; p<H/8; ++p){
        memset(line, 0, sizeof(line));
        for (int x=0; x<drawW; ++x){
            int sx = srcX0 + x;
            uint8_t b=0;
            for (int bit=0; bit<8; ++bit){
                int y = p*8 + bit;
                int idx = y*W + sx;
                uint8_t v = (buf[idx>>3] >> (7-(idx&7))) & 1u;
                b |= (uint8_t)(v << bit);
            }
            line[colStart + x]=b;
        }
        tx(line, PANEL_W, true);
    }
}
