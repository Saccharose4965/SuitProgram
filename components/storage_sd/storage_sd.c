#include "storage_sd.h"
#include "hw.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/unistd.h>   // fsync, rename, unlink
#include <sys/stat.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
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
static volatile bool s_mounting = false;

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
    if (s_card) return ESP_OK;

    // If another task is mounting, wait a bit for it to finish.
    if (s_mounting) {
        for (int i = 0; i < 20; ++i) { // up to ~1s
            vTaskDelay(pdMS_TO_TICKS(50));
            if (s_card) return ESP_OK;
        }
        // If still not mounted, fall through and try ourselves.
    }
    s_mounting = true;

    // Ensure SPI2 bus is up and all other CS lines are HIGH (idle).
    esp_err_t ret_hw = hw_spi2_init_once();
    if (ret_hw != ESP_OK) { s_mounting = false; return ret_hw; }
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

    // Keep SDSPI probing at supported full-duplex rates on this wiring/driver.
    // 40 MHz is rejected by SPI master on ESP32 here, so start at 20 MHz.
    const int khz_steps[] = { 20000, 10000, 5000, 2000, 400 };
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
            s_mounting = false;
            return ESP_OK;
        }

        ESP_LOGW(TAG, "sdspi_mount failed at %dkHz: %s",
                 host.max_freq_khz, esp_err_to_name(ret));

        // Ensure CS stays high before retry; tiny pause
        gpio_set_level(PIN_CS_SD, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    s_card = NULL;
    s_mounting = false;
    ESP_LOGE(TAG, "All SD mount attempts failed: %s", esp_err_to_name(ret));
    return ret;
}

void storage_unmount_sd(void)
{
    if (!s_card) return;
    esp_vfs_fat_sdcard_unmount(s_mount_point, s_card);
    s_card = NULL;
}

// ---------- Atomic boot counter (binary 4-byte file) ----------
esp_err_t storage_boot_counter_increment(uint32_t *out_count)
{
    if (out_count) *out_count = 0;

    esp_err_t m = storage_mount_sd();
    if (m != ESP_OK) {
        ESP_LOGE(TAG, "SD not mounted (%s)", esp_err_to_name(m));
        return m;
    }

    // Open/create the counter file
    FILE *f = fopen(HW_SD_COUNTER_PATH, "r+");
    if (!f) {
        f = fopen(HW_SD_COUNTER_PATH, "w+");
        if (!f) {
            ESP_LOGE(TAG, "Failed to create %s", HW_SD_COUNTER_PATH);
            return ESP_FAIL;
        }
        uint32_t zero = 0;
        if (fwrite(&zero, 1, sizeof(zero), f) != sizeof(zero)) {
            ESP_LOGE(TAG, "Init write failed");
            fclose(f);
            return ESP_FAIL;
        }
        fflush(f);
        int fd = fileno(f);
        if (fd >= 0) fsync(fd);
        fseek(f, 0, SEEK_SET);
    }

    uint32_t n = 0;
    size_t rd = fread(&n, 1, sizeof(n), f);
    if (rd != sizeof(n)) {
        n = 0;
        ESP_LOGW(TAG, "Counter file corrupt; resetting");
    }

    if (n == UINT32_MAX) n = 0; else n += 1;

    fseek(f, 0, SEEK_SET);
    if (fwrite(&n, 1, sizeof(n), f) != sizeof(n)) {
        ESP_LOGE(TAG, "Counter write failed");
        fclose(f);
        return ESP_FAIL;
    }
    fflush(f);
    int fd = fileno(f);
    if (fd >= 0) fsync(fd);
    fclose(f);

    if (out_count) *out_count = n;
    return ESP_OK;
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
