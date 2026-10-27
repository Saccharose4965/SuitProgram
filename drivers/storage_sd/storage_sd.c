#include "storage_sd.h"
#include "hw.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/unistd.h>   // fsync, rename, unlink
#include <sys/stat.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_idf_version.h"

// ---- choose ONE mount point here ----
#define STORAGE_MOUNT_POINT "/sdcard"
// -------------------------------------

static const char* TAG = "storage_sd";
static sdmmc_card_t *s_card = NULL;
static char s_mount_point[16] = STORAGE_MOUNT_POINT;
static StaticSemaphore_t s_lifecycle_mutex_buffer;
static SemaphoreHandle_t s_lifecycle_mutex = NULL;
static portMUX_TYPE s_lifecycle_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t storage_lifecycle_mutex(void)
{
    if (s_lifecycle_mutex) return s_lifecycle_mutex;
    taskENTER_CRITICAL(&s_lifecycle_mutex_init_lock);
    if (!s_lifecycle_mutex) {
        s_lifecycle_mutex = xSemaphoreCreateMutexStatic(&s_lifecycle_mutex_buffer);
    }
    taskEXIT_CRITICAL(&s_lifecycle_mutex_init_lock);
    return s_lifecycle_mutex;
}

static esp_err_t storage_lifecycle_lock(void)
{
    SemaphoreHandle_t mutex = storage_lifecycle_mutex();
    if (!mutex) return ESP_ERR_NO_MEM;
    return xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

static void storage_lifecycle_unlock(void)
{
    if (s_lifecycle_mutex) xSemaphoreGive(s_lifecycle_mutex);
}

bool storage_sd_is_mounted(void) { return s_card != NULL; }
const char* storage_sd_mount_point(void) { return s_mount_point; }

esp_err_t storage_sd_make_path(char* out, size_t outsz, const char* rel) {
    if (!out || outsz == 0) return ESP_ERR_INVALID_ARG;
    const char* base = storage_sd_mount_point();
    if (!rel || !*rel) rel = "";
    int n = snprintf(out, outsz, "%s/%s", base, rel);
    return (n > 0 && (size_t)n < outsz) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t storage_mount_sd(void)
{
    esp_err_t lock_err = storage_lifecycle_lock();
    if (lock_err != ESP_OK) return lock_err;
    if (s_card) {
        storage_lifecycle_unlock();
        return ESP_OK;
    }

    // Ensure SPI2 bus is up and all other CS lines are HIGH (idle).
    esp_err_t ret_hw = hw_spi2_init_once();
    if (ret_hw != ESP_OK) {
        storage_lifecycle_unlock();
        return ret_hw;
    }
    hw_spi2_idle_all_cs_high();

    // Pull-ups & drive strength
    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_SPI_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_SPI_CLK,  GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS_SD,    GPIO_PULLUP_ONLY);

    gpio_set_drive_capability(PIN_SPI_CLK,  GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PIN_SPI_MOSI, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PIN_CS_SD,    GPIO_DRIVE_CAP_3);

    // Host (SDSPI on SPI2)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = HW_SPI_HOST; // SPI2_HOST

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.host_id  = host.slot;
    slot_cfg.gpio_cs  = PIN_CS_SD;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 16 * 1024,
    #if ESP_IDF_VERSION_MAJOR >= 5
        .disk_status_check_enable = true,
    #endif
    };

    // Mounting at 20 MHz succeeded but sustained reads produced CRC/timeouts.
    // Leave margin on the shared wired bus; 10 MHz still exceeds PCM throughput.
    const int khz_steps[] = { 10000, 5000, 2000, 400 };
    const int n_steps = sizeof(khz_steps) / sizeof(khz_steps[0]);

    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < n_steps; ++i) {
        host.max_freq_khz = khz_steps[i];

        ESP_LOGI(TAG,
                 "Mounting SD at '%s' (CS=%d, MOSI=%d, MISO=%d, SCLK=%d, clk=%dkHz) [try %d/%d]",
                 STORAGE_MOUNT_POINT, PIN_CS_SD, PIN_SPI_MOSI, PIN_SPI_MISO,
                 PIN_SPI_CLK, host.max_freq_khz, i+1, n_steps);

        ret = esp_vfs_fat_sdspi_mount(STORAGE_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_card);
        if (ret == ESP_OK) {
            // Cache the exact mount point we used
            strncpy(s_mount_point, STORAGE_MOUNT_POINT, sizeof(s_mount_point));
            s_mount_point[sizeof(s_mount_point)-1] = '\0';

            sdmmc_card_print_info(stdout, s_card);
            storage_lifecycle_unlock();
            return ESP_OK;
        }

        ESP_LOGW(TAG, "sdspi_mount failed at %dkHz: %s",
                 host.max_freq_khz, esp_err_to_name(ret));

        // Ensure CS stays high before retry; tiny pause
        gpio_set_level(PIN_CS_SD, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    s_card = NULL;
    ESP_LOGE(TAG, "All SD mount attempts failed: %s", esp_err_to_name(ret));
    storage_lifecycle_unlock();
    return ret;
}

void storage_unmount_sd(void)
{
    if (storage_lifecycle_lock() != ESP_OK) return;
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(s_mount_point, s_card);
        s_card = NULL;
    }
    storage_lifecycle_unlock();
}

// --------------------- Optional: directory listing ---------------------
esp_err_t storage_sd_list_dir(const char* rel_or_null)
{
    char path[128];
    ESP_RETURN_ON_ERROR(storage_sd_make_path(path, sizeof(path),
                        rel_or_null ? rel_or_null : ""), TAG, "path build");

    DIR *d = opendir(path);
    if (!d) {
        ESP_LOGE(TAG, "opendir(%s) failed", path);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Listing %s:", path);
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        printf("  %s\n", e->d_name);
    }
    closedir(d);
    return ESP_OK;
}
