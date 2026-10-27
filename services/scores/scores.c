#include "scores.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "storage_sd.h"

static const char *TAG = "scores";

typedef struct {
    char primary[128];
    char temporary[136];
    char backup[136];
} score_paths_t;

static esp_err_t validate_game_key(const char *game_key)
{
    if (!game_key || !game_key[0]) return ESP_ERR_INVALID_ARG;
    size_t len = 0;
    for (const char *p = game_key; *p; ++p) {
        unsigned char c = (unsigned char)*p;
        if (!(isalnum(c) || c == '_' || c == '-')) return ESP_ERR_INVALID_ARG;
        if (++len > 40) return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t ensure_scores_dir(void)
{
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount sd");
    char dir_path[96];
    ESP_RETURN_ON_ERROR(storage_sd_make_path(dir_path, sizeof(dir_path), "scores"),
                        TAG, "build dir path");
    struct stat st = {0};
    if (stat(dir_path, &st) == 0) {
        return S_ISDIR(st.st_mode) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (mkdir(dir_path, 0775) == 0 || errno == EEXIST) return ESP_OK;
    return ESP_FAIL;
}

static esp_err_t build_score_paths(const char *game_key, score_paths_t *paths)
{
    if (!paths) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(validate_game_key(game_key), TAG, "bad key");
    char relative[80];
    int n = snprintf(relative, sizeof(relative), "scores/%s.txt", game_key);
    if (n <= 0 || (size_t)n >= sizeof(relative)) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(storage_sd_make_path(paths->primary, sizeof(paths->primary), relative),
                        TAG, "build score path");
    n = snprintf(paths->temporary, sizeof(paths->temporary), "%s.tmp", paths->primary);
    if (n <= 0 || (size_t)n >= sizeof(paths->temporary)) return ESP_ERR_NO_MEM;
    n = snprintf(paths->backup, sizeof(paths->backup), "%s.bak", paths->primary);
    if (n <= 0 || (size_t)n >= sizeof(paths->backup)) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static esp_err_t read_score_file(const char *path, uint32_t *score, bool *exists)
{
    if (!path || !score || !exists) return ESP_ERR_INVALID_ARG;
    *score = 0;
    *exists = false;
    FILE *file = fopen(path, "rb");
    if (!file) return errno == ENOENT ? ESP_OK : ESP_FAIL;
    *exists = true;

    char text[64];
    size_t length = fread(text, 1, sizeof(text) - 1, file);
    bool io_ok = !ferror(file);
    int extra = io_ok ? fgetc(file) : EOF;
    if (fclose(file) != 0) io_ok = false;
    if (!io_ok) return ESP_FAIL;
    if (extra != EOF || memchr(text, '\0', length) != NULL) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    text[length] = '\0';

    size_t pos = 0;
    while (pos < length && isspace((unsigned char)text[pos])) pos++;
    if (pos == length || !isdigit((unsigned char)text[pos])) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint32_t value = 0;
    while (pos < length && isdigit((unsigned char)text[pos])) {
        unsigned digit = (unsigned)(text[pos] - '0');
        if (value > (UINT32_MAX - digit) / 10u) return ESP_ERR_INVALID_RESPONSE;
        value = value * 10u + digit;
        pos++;
    }
    while (pos < length && isspace((unsigned char)text[pos])) pos++;
    if (pos != length) return ESP_ERR_INVALID_RESPONSE;
    *score = value;
    return ESP_OK;
}

static esp_err_t write_temporary(const char *path, uint32_t score)
{
    FILE *file = fopen(path, "wb");
    if (!file) return ESP_FAIL;
    bool ok = fprintf(file, "%lu\n", (unsigned long)score) > 0;
    if (ok && fflush(file) != 0) ok = false;
    if (ok) {
        int fd = fileno(file);
        if (fd < 0 || fsync(fd) != 0) ok = false;
    }
    if (fclose(file) != 0) ok = false;
    if (ok) return ESP_OK;
    (void)unlink(path);
    return ESP_FAIL;
}

static esp_err_t scores_save_high(const char *game_key, uint32_t score)
{
    ESP_RETURN_ON_ERROR(ensure_scores_dir(), TAG, "ensure dir");
    score_paths_t paths;
    ESP_RETURN_ON_ERROR(build_score_paths(game_key, &paths), TAG, "build paths");
    ESP_RETURN_ON_ERROR(write_temporary(paths.temporary, score), TAG, "write temporary");

    uint32_t old_score = 0;
    bool primary_exists = false;
    esp_err_t primary_status = read_score_file(paths.primary, &old_score, &primary_exists);
    bool primary_valid = primary_status == ESP_OK && primary_exists;
    if (primary_exists && !primary_valid) {
        if (unlink(paths.primary) != 0) goto fail;
    } else if (primary_valid) {
        if (unlink(paths.backup) != 0 && errno != ENOENT) goto fail;
        if (rename(paths.primary, paths.backup) != 0) goto fail;
    } else if (primary_status != ESP_OK) {
        goto fail;
    }

    if (rename(paths.temporary, paths.primary) == 0) return ESP_OK;
    if (primary_valid && rename(paths.backup, paths.primary) != 0) {
        ESP_LOGE(TAG, "score install and backup restore failed for '%s'", game_key);
    }
fail:
    (void)unlink(paths.temporary);
    return ESP_FAIL;
}

static esp_err_t promote_temporary(const score_paths_t *paths, const char *game_key)
{
    (void)game_key;
    uint32_t ignored = 0;
    bool primary_exists = false;
    esp_err_t status = read_score_file(paths->primary, &ignored, &primary_exists);
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
        ESP_LOGE(TAG, "recovery install and backup restore failed for '%s'", game_key);
    }
    return ESP_FAIL;
}

esp_err_t scores_load_high(const char *game_key, uint32_t *out_high)
{
    if (!out_high) return ESP_ERR_INVALID_ARG;
    *out_high = 0;
    ESP_RETURN_ON_ERROR(ensure_scores_dir(), TAG, "ensure dir");
    score_paths_t paths;
    ESP_RETURN_ON_ERROR(build_score_paths(game_key, &paths), TAG, "build paths");

    const char *copies[] = {paths.primary, paths.backup, paths.temporary};
    uint32_t best = 0;
    bool have_valid = false;
    bool have_any = false;
    size_t best_copy = 0;
    for (size_t i = 0; i < sizeof(copies) / sizeof(copies[0]); ++i) {
        uint32_t value = 0;
        bool exists = false;
        esp_err_t err = read_score_file(copies[i], &value, &exists);
        if (err != ESP_OK && err != ESP_ERR_INVALID_RESPONSE) return err;
        have_any |= exists;
        if (err == ESP_OK && exists && (!have_valid || value > best)) {
            best = value;
            have_valid = true;
            best_copy = i;
        }
    }
    if (!have_valid) return have_any ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
    *out_high = best;
    if (best_copy != 0) {
        esp_err_t repair = best_copy == 2
            ? promote_temporary(&paths, game_key)
            : scores_save_high(game_key, best);
        if (repair != ESP_OK) {
            ESP_LOGW(TAG, "using recovered score for '%s'; primary repair failed", game_key);
        }
    }
    return ESP_OK;
}

esp_err_t scores_update_high(const char *game_key, uint32_t candidate, uint32_t *out_high)
{
    uint32_t current = 0;
    ESP_RETURN_ON_ERROR(scores_load_high(game_key, &current), TAG, "load");
    if (candidate > current) {
        ESP_RETURN_ON_ERROR(scores_save_high(game_key, candidate), TAG, "save");
        current = candidate;
    }
    if (out_high) *out_high = current;
    return ESP_OK;
}
