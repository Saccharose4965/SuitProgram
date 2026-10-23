#include "scores.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_check.h"
#include "storage_sd.h"

static esp_err_t validate_game_key(const char *game_key){
    if (!game_key || !game_key[0]) return ESP_ERR_INVALID_ARG;

    size_t len = 0;
    for (const char *p = game_key; *p; ++p){
        unsigned char c = (unsigned char)*p;
        if (!(isalnum(c) || c == '_' || c == '-')){
            return ESP_ERR_INVALID_ARG;
        }
        len++;
        if (len > 40) return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t ensure_scores_dir(void){
    ESP_RETURN_ON_ERROR(storage_mount_sd(), "scores", "mount sd");

    char dir_path[96];
    ESP_RETURN_ON_ERROR(
        storage_sd_make_path(dir_path, sizeof(dir_path), "scores"),
        "scores",
        "build dir path"
    );

    struct stat st = {0};
    if (stat(dir_path, &st) == 0){
        if (S_ISDIR(st.st_mode)) return ESP_OK;
        return ESP_ERR_INVALID_STATE;
    }

    if (mkdir(dir_path, 0775) == 0 || errno == EEXIST){
        return ESP_OK;
    }

    return ESP_FAIL;
}

static esp_err_t build_score_path(const char *game_key, char *out, size_t out_sz){
    if (!out || out_sz == 0) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(validate_game_key(game_key), "scores", "bad key");

    char rel_path[80];
    int n = snprintf(rel_path, sizeof(rel_path), "scores/%s.txt", game_key);
    if (n <= 0 || (size_t)n >= sizeof(rel_path)) return ESP_ERR_NO_MEM;

    return storage_sd_make_path(out, out_sz, rel_path);
}

static esp_err_t scores_save_high(const char *game_key, uint32_t high){
    ESP_RETURN_ON_ERROR(ensure_scores_dir(), "scores", "ensure dir");

    char path[128];
    ESP_RETURN_ON_ERROR(build_score_path(game_key, path, sizeof(path)), "scores", "path");

    char tmp_path[136];
    int tn = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    if (tn <= 0 || (size_t)tn >= sizeof(tmp_path)) return ESP_ERR_NO_MEM;

    FILE *f = fopen(tmp_path, "w");
    if (!f) return ESP_FAIL;

    bool ok = true;
    if (fprintf(f, "%lu\n", (unsigned long)high) <= 0) ok = false;
    if (ok && fflush(f) != 0) ok = false;
    if (ok){
        int fd = fileno(f);
        if (fd >= 0 && fsync(fd) != 0) ok = false;
    }
    if (fclose(f) != 0) ok = false;

    if (!ok){
        unlink(tmp_path);
        return ESP_FAIL;
    }

    if (rename(tmp_path, path) != 0){
        unlink(tmp_path);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t scores_load_high(const char *game_key, uint32_t *out_high){
    if (!out_high) return ESP_ERR_INVALID_ARG;
    *out_high = 0;

    ESP_RETURN_ON_ERROR(ensure_scores_dir(), "scores", "ensure dir");

    char path[128];
    ESP_RETURN_ON_ERROR(build_score_path(game_key, path, sizeof(path)), "scores", "path");

    FILE *f = fopen(path, "r");
    if (!f){
        if (errno == ENOENT){
            return ESP_OK;
        }
        return ESP_FAIL;
    }

    unsigned long v = 0;
    if (fscanf(f, "%lu", &v) == 1){
        *out_high = (uint32_t)v;
    }

    fclose(f);
    return ESP_OK;
}

esp_err_t scores_update_high(const char *game_key, uint32_t candidate, uint32_t *out_high){
    uint32_t current = 0;
    ESP_RETURN_ON_ERROR(scores_load_high(game_key, &current), "scores", "load");

    if (candidate > current){
        ESP_RETURN_ON_ERROR(scores_save_high(game_key, candidate), "scores", "save");
        current = candidate;
    }

    if (out_high) *out_high = current;
    return ESP_OK;
}
