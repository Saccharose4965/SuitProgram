#include "show_package.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "storage_sd.h"

static const char *TAG = "show_package";

static bool show_header_looks_valid(const show_file_header_v1_t *hdr)
{
    if (!hdr) return false;
    if (hdr->magic != SHOW_FILE_MAGIC_V1) return false;
    if (hdr->version_major != SHOW_FILE_VERSION_MAJOR_V1) return false;
    if (hdr->header_bytes < sizeof(*hdr)) return false;
    if (hdr->file_bytes < hdr->header_bytes) return false;
    if (hdr->role_count == 0 || hdr->role_count > SHOW_MAX_ROLES) return false;
    return true;
}

static esp_err_t show_package_read_role_table(FILE *f,
                                              const show_file_header_v1_t *hdr,
                                              show_role_program_v1_t *roles,
                                              size_t role_cap)
{
    if (!f || !hdr || !roles) return ESP_ERR_INVALID_ARG;
    if (hdr->role_count > role_cap) return ESP_ERR_INVALID_SIZE;
    if (fseek(f, (long)hdr->role_table_offset, SEEK_SET) != 0) {
        return ESP_FAIL;
    }
    size_t want = hdr->role_count;
    size_t got = fread(roles, sizeof(show_role_program_v1_t), want, f);
    if (got != want) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t show_package_build_path(const char *show_slug, char *out, size_t out_sz)
{
    if (!show_slug || !*show_slug || !out || out_sz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    char rel[96];
    int rel_len = snprintf(rel, sizeof(rel), "shows/%s/show.bin", show_slug);
    if (rel_len <= 0 || (size_t)rel_len >= sizeof(rel)) {
        return ESP_ERR_NO_MEM;
    }
    return storage_sd_make_path(out, out_sz, rel);
}

esp_err_t show_package_load_info_from_path(const char *path, show_package_info_t *out)
{
    if (!path || !*path || !out) return ESP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGW(TAG, "open failed: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    show_file_header_v1_t hdr = {0};
    size_t got = fread(&hdr, 1, sizeof(hdr), f);
    if (got != sizeof(hdr)) {
        fclose(f);
        return ESP_FAIL;
    }

    if (!show_header_looks_valid(&hdr)) {
        fclose(f);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return ESP_FAIL;
    }
    long file_size = ftell(f);
    if (file_size < 0) {
        fclose(f);
        return ESP_FAIL;
    }
    if ((uint32_t)file_size != hdr.file_bytes) {
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    if (hdr.role_table_offset + hdr.role_count * sizeof(show_role_program_v1_t) > hdr.file_bytes) {
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    show_role_program_v1_t roles[SHOW_MAX_ROLES] = {0};
    esp_err_t err = show_package_read_role_table(f, &hdr, roles, SHOW_MAX_ROLES);
    fclose(f);
    if (err != ESP_OK) {
        return err;
    }

    out->show_uid = hdr.show_uid;
    out->duration_ms = hdr.duration_ms;
    out->file_bytes = hdr.file_bytes;
    out->version_major = hdr.version_major;
    out->version_minor = hdr.version_minor;
    out->role_count = hdr.role_count;
    out->fps_hint = hdr.fps_hint;
    out->bucket_ms = hdr.bucket_ms;
    out->palette_count = hdr.palette_count;
    out->group_count = hdr.group_count;
    out->crc_checked = false;
    out->crc_valid = false;
    for (uint16_t i = 0; i < hdr.role_count && i < SHOW_MAX_ROLES; ++i) {
        out->roles[i].role_id = roles[i].role_id;
        out->roles[i].bucket_count = roles[i].bucket_count;
        out->roles[i].clip_count = roles[i].clip_count;
    }
    return ESP_OK;
}

esp_err_t show_package_load_info(const char *show_slug, show_package_info_t *out)
{
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount sd");

    char path[128];
    ESP_RETURN_ON_ERROR(show_package_build_path(show_slug, path, sizeof(path)),
                        TAG, "path");
    return show_package_load_info_from_path(path, out);
}
