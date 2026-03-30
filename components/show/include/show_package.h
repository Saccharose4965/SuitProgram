#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "show_format.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t role_id;
    uint32_t bucket_count;
    uint32_t clip_count;
} show_package_role_info_t;

typedef struct {
    uint64_t show_uid;
    uint32_t duration_ms;
    uint32_t file_bytes;
    uint16_t version_major;
    uint16_t version_minor;
    uint16_t role_count;
    uint16_t fps_hint;
    uint16_t bucket_ms;
    uint16_t palette_count;
    uint16_t group_count;
    bool crc_checked;
    bool crc_valid;
    show_package_role_info_t roles[SHOW_MAX_ROLES];
} show_package_info_t;

esp_err_t show_package_build_path(const char *show_slug, char *out, size_t out_sz);
esp_err_t show_package_load_info(const char *show_slug, show_package_info_t *out);
esp_err_t show_package_load_info_from_path(const char *path, show_package_info_t *out);

#ifdef __cplusplus
}
#endif
