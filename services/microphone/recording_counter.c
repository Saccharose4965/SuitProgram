#include "recording_counter.h"

#include "storage_sd.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "recording_counter";

typedef struct {
    char primary[128];
    char temporary[136];
    char backup[136];
} counter_paths_t;

static esp_err_t build_paths(counter_paths_t *paths)
{
    if (!paths) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(storage_sd_make_path(paths->primary, sizeof(paths->primary),
                                             ".rec_counter"), TAG, "counter path");
    int n = snprintf(paths->temporary, sizeof(paths->temporary), "%s.tmp", paths->primary);
    if (n <= 0 || (size_t)n >= sizeof(paths->temporary)) return ESP_ERR_NO_MEM;
    n = snprintf(paths->backup, sizeof(paths->backup), "%s.bak", paths->primary);
    if (n <= 0 || (size_t)n >= sizeof(paths->backup)) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static esp_err_t read_counter(const char *path, uint32_t *value, bool *exists)
{
    *value = 0;
    *exists = false;
    FILE *file = fopen(path, "rb");
    if (!file) return errno == ENOENT ? ESP_OK : ESP_FAIL;
    *exists = true;

    uint32_t stored = 0;
    size_t count = fread(&stored, 1, sizeof(stored), file);
    int extra = fgetc(file);
    bool ok = count == sizeof(stored) && extra == EOF && !ferror(file);
    if (fclose(file) != 0) ok = false;
    if (!ok) return ESP_ERR_INVALID_RESPONSE;
    *value = stored;
    return ESP_OK;
}

static esp_err_t write_counter(const counter_paths_t *paths, uint32_t value)
{
    FILE *file = fopen(paths->temporary, "wb");
    if (!file) return ESP_FAIL;
    bool ok = fwrite(&value, 1, sizeof(value), file) == sizeof(value);
    if (ok && fflush(file) != 0) ok = false;
    if (ok) {
        int fd = fileno(file);
        if (fd < 0 || fsync(fd) != 0) ok = false;
    }
    if (fclose(file) != 0) ok = false;
    if (!ok) {
        (void)unlink(paths->temporary);
        return ESP_FAIL;
    }

    uint32_t ignored = 0;
    bool primary_exists = false;
    esp_err_t primary_status = read_counter(paths->primary, &ignored, &primary_exists);
    bool primary_valid = primary_status == ESP_OK && primary_exists;
    if (primary_exists && !primary_valid) {
        if (unlink(paths->primary) != 0) goto fail;
    } else if (primary_valid) {
        if (unlink(paths->backup) != 0 && errno != ENOENT) goto fail;
        if (rename(paths->primary, paths->backup) != 0) goto fail;
    } else if (primary_status != ESP_OK) {
        goto fail;
    }

    if (rename(paths->temporary, paths->primary) == 0) return ESP_OK;
    if (primary_valid && rename(paths->backup, paths->primary) != 0) {
        ESP_LOGE(TAG, "counter install and backup restore failed");
    }
fail:
    (void)unlink(paths->temporary);
    return ESP_FAIL;
}

static esp_err_t promote_temporary(const counter_paths_t *paths)
{
    uint32_t ignored = 0;
    bool primary_exists = false;
    esp_err_t status = read_counter(paths->primary, &ignored, &primary_exists);
    bool primary_valid = status == ESP_OK && primary_exists;
    if (primary_exists && !primary_valid) {
        if (unlink(paths->primary) != 0) return ESP_FAIL;
    } else if (primary_valid) {
        if (unlink(paths->backup) != 0 && errno != ENOENT) return ESP_FAIL;
        if (rename(paths->primary, paths->backup) != 0) return ESP_FAIL;
    } else if (status != ESP_OK) {
        return status;
    }
    if (rename(paths->temporary, paths->primary) == 0) return ESP_OK;
    if (primary_valid && rename(paths->backup, paths->primary) != 0) {
        ESP_LOGE(TAG, "counter recovery and backup restore failed");
    }
    return ESP_FAIL;
}

esp_err_t recording_counter_next(uint32_t *out_count)
{
    if (out_count) *out_count = 0;
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount SD");

    counter_paths_t paths;
    ESP_RETURN_ON_ERROR(build_paths(&paths), TAG, "build paths");
    const char *copies[] = {paths.primary, paths.backup, paths.temporary};
    uint32_t highest = 0;
    bool have_valid = false;
    bool have_invalid = false;
    size_t highest_copy = 0;

    for (size_t i = 0; i < sizeof(copies) / sizeof(copies[0]); ++i) {
        uint32_t value = 0;
        bool exists = false;
        esp_err_t err = read_counter(copies[i], &value, &exists);
        if (err != ESP_OK && err != ESP_ERR_INVALID_RESPONSE) return err;
        if (err == ESP_ERR_INVALID_RESPONSE) have_invalid = true;
        if (err == ESP_OK && exists && (!have_valid || value > highest)) {
            highest = value;
            have_valid = true;
            highest_copy = i;
        }
    }

    if (!have_valid && have_invalid) {
        ESP_LOGW(TAG, "counter copies corrupt; restarting with collision checks");
    }
    if (have_valid && highest_copy == 2) {
        ESP_RETURN_ON_ERROR(promote_temporary(&paths), TAG, "recover temporary counter");
    }
    if (have_valid && highest == UINT32_MAX) return ESP_ERR_INVALID_STATE;
    uint32_t next = have_valid ? highest + 1u : 0u;
    ESP_RETURN_ON_ERROR(write_counter(&paths, next), TAG, "save counter");
    if (out_count) *out_count = next;
    return ESP_OK;
}
