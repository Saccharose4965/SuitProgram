#include "audio_i2s.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_i2s";

static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;
static audio_i2s_config_t s_config;
static volatile int s_rate = 48000;
static volatile bool s_rate_changing;
static volatile uint32_t s_clock_revision;
static volatile uint32_t s_rx_overflows;
static bool s_tx_enabled;
static bool s_rx_enabled;
static bool s_configured;
static DRAM_ATTR int32_t s_prime_silence[256 * 2];

static i2s_clock_src_t clock_source(int hz)
{
    return hz % 11025 == 0 ? I2S_CLK_SRC_APLL : I2S_CLK_SRC_DEFAULT;
}

static bool IRAM_ATTR rx_overflow(i2s_chan_handle_t channel,
                                  i2s_event_data_t *event, void *user)
{
    (void)channel;
    (void)event;
    (void)user;
    ++s_rx_overflows;
    return false;
}

esp_err_t audio_i2s_init(const audio_i2s_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    if (s_tx || s_rx) return ESP_ERR_INVALID_STATE;

    s_config = *config;
    s_configured = true;
    s_rate = 48000;
    s_tx_enabled = false;
    s_rx_enabled = false;

    if (config->sd_en_pin >= 0) {
        gpio_reset_pin(config->sd_en_pin);
        gpio_set_direction(config->sd_en_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(config->sd_en_pin, 0);
    }

    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(
        config->i2s_id, I2S_ROLE_MASTER);
    channel.dma_desc_num = 8;
    channel.dma_frame_num = 256;
    channel.auto_clear = true;

    esp_err_t err = i2s_new_channel(&channel, &s_tx, &s_rx);
    if (err != ESP_OK) goto fail;

    i2s_std_config_t tx_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bclk_gpio,
            .ws = config->lrck_gpio,
            .dout = config->dout_gpio,
            .din = I2S_GPIO_UNUSED,
        },
    };
    tx_config.clk_cfg.clk_src = clock_source(s_rate);
    tx_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    tx_config.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    tx_config.slot_cfg.bit_shift = true;
    tx_config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    i2s_std_config_t rx_config = tx_config;
    rx_config.gpio_cfg.dout = I2S_GPIO_UNUSED;
    rx_config.gpio_cfg.din = config->din_gpio;

    err = i2s_channel_init_std_mode(s_tx, &tx_config);
    if (err != ESP_OK) goto fail;
    if (config->din_gpio >= 0) {
        err = i2s_channel_init_std_mode(s_rx, &rx_config);
        if (err != ESP_OK) goto fail;
        i2s_event_callbacks_t callbacks = {.on_recv_q_ovf = rx_overflow};
        err = i2s_channel_register_event_callback(s_rx, &callbacks, NULL);
        if (err != ESP_OK) goto fail;
    } else {
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }

    ESP_LOGI(TAG, "ready: fs=%d BCLK=%d LRCK=%d DOUT=%d DIN=%d", s_rate,
             config->bclk_gpio, config->lrck_gpio, config->dout_gpio,
             config->din_gpio);
    return ESP_OK;

fail:
    if (s_rx) {
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }
    if (s_tx) {
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    return err;
}

esp_err_t audio_i2s_release(void)
{
    esp_err_t err = audio_i2s_disable_all();
    if (err != ESP_OK) return err;
    if (s_rx) {
        err = i2s_del_channel(s_rx);
        if (err != ESP_OK) return err;
        s_rx = NULL;
    }
    if (s_tx) {
        err = i2s_del_channel(s_tx);
        if (err != ESP_OK) return err;
        s_tx = NULL;
    }
    ++s_clock_revision;
    return ESP_OK;
}

static esp_err_t restore_bus(void)
{
    if (s_tx) return ESP_OK;
    if (!s_configured) return ESP_ERR_INVALID_STATE;
    int wanted_rate = s_rate;
    esp_err_t err = audio_i2s_init(&s_config);
    if (err != ESP_OK) return err;
    return audio_i2s_set_rate(wanted_rate);
}

static esp_err_t reconfigure_clock(i2s_chan_handle_t channel, int hz)
{
    if (!channel) return ESP_OK;
    if ((channel == s_tx && s_tx_enabled) ||
        (channel == s_rx && s_rx_enabled)) {
        (void)i2s_channel_disable(channel);
        if (channel == s_tx) s_tx_enabled = false;
        else s_rx_enabled = false;
    }
    i2s_std_clk_config_t clock = {
        .clk_src = clock_source(hz),
        .sample_rate_hz = hz,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };
    return i2s_channel_reconfig_std_clock(channel, &clock);
}

esp_err_t audio_i2s_set_rate(int hz)
{
    if (hz <= 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = restore_bus();
    if (err != ESP_OK) return err;

    s_rate_changing = true;
    ++s_clock_revision;
    err = reconfigure_clock(s_tx, hz);
    if (err == ESP_OK) err = reconfigure_clock(s_rx, hz);
    if (err == ESP_OK) s_rate = hz;
    s_rate_changing = false;
    return err;
}

int audio_i2s_get_rate(void)
{
    return s_rate_changing ? 0 : s_rate;
}

uint32_t audio_i2s_get_clock_revision(void)
{
    return s_clock_revision;
}

uint32_t audio_i2s_get_rx_overflows(void)
{
    return s_rx_overflows;
}

esp_err_t audio_i2s_enable_tx(void)
{
    esp_err_t err = restore_bus();
    if (err != ESP_OK) return err;
    if (!s_tx) return ESP_ERR_INVALID_STATE;

    if (!s_tx_enabled) {
        if (s_config.sd_en_pin >= 0) gpio_set_level(s_config.sd_en_pin, 1);
        err = i2s_channel_enable(s_tx);
        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "TX enable timeout; retrying");
            (void)i2s_channel_disable(s_tx);
            vTaskDelay(pdMS_TO_TICKS(1));
            err = i2s_channel_enable(s_tx);
        }
        if (err == ESP_ERR_TIMEOUT && s_rx && !s_rx_enabled) {
            ESP_LOGW(TAG, "arming RX before final TX retry");
            esp_err_t rx_err = i2s_channel_enable(s_rx);
            if (rx_err == ESP_OK || rx_err == ESP_ERR_INVALID_STATE)
                s_rx_enabled = true;
            vTaskDelay(pdMS_TO_TICKS(1));
            err = i2s_channel_enable(s_tx);
        }
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            if (s_config.sd_en_pin >= 0) gpio_set_level(s_config.sd_en_pin, 0);
            return err;
        }
        s_tx_enabled = true;
    }

    if (s_rx && !s_rx_enabled) {
        esp_err_t rx_err = i2s_channel_enable(s_rx);
        if (rx_err == ESP_OK || rx_err == ESP_ERR_INVALID_STATE)
            s_rx_enabled = true;
        else
            ESP_LOGW(TAG, "paired RX enable failed: %s",
                     esp_err_to_name(rx_err));
    }

    size_t written = 0;
    err = i2s_channel_write(s_tx, s_prime_silence, sizeof(s_prime_silence),
                            &written, 40);
    if (err == ESP_ERR_TIMEOUT || written == 0) err = ESP_ERR_TIMEOUT;
    if (err != ESP_OK) {
        (void)audio_i2s_disable_tx();
        return err;
    }
    return ESP_OK;
}

esp_err_t audio_i2s_enable_rx(void)
{
    esp_err_t err = restore_bus();
    if (err != ESP_OK) return err;
    if (!s_rx) return ESP_ERR_INVALID_STATE;
    if (s_config.sd_en_pin >= 0) gpio_set_level(s_config.sd_en_pin, 0);

    if (!s_tx_enabled) {
        err = i2s_channel_enable(s_tx);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
        s_tx_enabled = true;
        size_t written = 0;
        (void)i2s_channel_write(s_tx, s_prime_silence,
                                sizeof(s_prime_silence), &written, 10);
    }
    if (!s_rx_enabled) {
        err = i2s_channel_enable(s_rx);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
        s_rx_enabled = true;
    }
    return ESP_OK;
}

esp_err_t audio_i2s_disable_tx(void)
{
    if (s_config.sd_en_pin >= 0 && s_configured)
        gpio_set_level(s_config.sd_en_pin, 0);
    if (!s_tx_enabled) return ESP_OK;
    esp_err_t err = i2s_channel_disable(s_tx);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        s_tx_enabled = false;
        return ESP_OK;
    }
    return err;
}

esp_err_t audio_i2s_disable_all(void)
{
    esp_err_t tx_err = audio_i2s_disable_tx();
    esp_err_t rx_err = ESP_OK;
    if (s_rx_enabled) {
        rx_err = i2s_channel_disable(s_rx);
        if (rx_err == ESP_OK || rx_err == ESP_ERR_INVALID_STATE) {
            s_rx_enabled = false;
            rx_err = ESP_OK;
        }
    }
    return tx_err != ESP_OK ? tx_err : rx_err;
}

i2s_chan_handle_t audio_i2s_tx_handle(void)
{
    return s_tx;
}

i2s_chan_handle_t audio_i2s_rx_handle(void)
{
    return s_rx;
}
