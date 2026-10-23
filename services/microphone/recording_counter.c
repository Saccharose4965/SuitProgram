#include "recording_counter.h"
#include "storage_sd.h"
#include "esp_log.h"
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "recording_counter";

// ---------- Legacy in-place boot counter (binary 4-byte file) ----------
esp_err_t recording_counter_next(uint32_t *out_count)
{
    if (out_count) *out_count = 0;

    esp_err_t m = storage_mount_sd();
    if (m != ESP_OK) {
        ESP_LOGE(TAG, "SD not mounted (%s)", esp_err_to_name(m));
        return m;
    }

    char path[128];
    esp_err_t e = storage_sd_make_path(path, sizeof(path), ".rec_counter");
    if (e != ESP_OK) return e;

    // Open/create the counter file
    FILE *f = fopen(path, "r+");
    if (!f) {
        f = fopen(path, "w+");
        if (!f) {
            ESP_LOGE(TAG, "Failed to create %s", path);
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
