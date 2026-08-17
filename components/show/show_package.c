#include "show_package.h"

#include <ctype.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "show_layout_ids.h"
#include "storage_sd.h"

static const char *TAG = "show_package";

typedef struct {
    show_file_header_v1_t header;
    show_file_timing_v1_1_t timing;
    show_role_program_v1_t roles[SHOW_MAX_ROLES];
    uint32_t bucket_total;
    uint32_t clip_total;
} show_validated_file_t;

static bool show_slug_valid(const char *slug)
{
    if (!slug || !slug[0] || slug[0] == '.') return false;
    size_t len = 0;
    for (const unsigned char *p = (const unsigned char *)slug; *p; ++p) {
        if (!(isalnum(*p) || *p == '-' || *p == '_' || *p == '.')) return false;
        if (++len >= 32u) return false;
    }
    return strstr(slug, "..") == NULL;
}

static bool show_range_valid(uint32_t offset, uint32_t count, size_t item_bytes,
                             uint32_t limit)
{
    if (offset > limit || item_bytes == 0) return false;
    return count <= ((size_t)(limit - offset) / item_bytes);
}

static uint32_t show_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return crc;
}

static esp_err_t show_read_exact(FILE *f, uint32_t offset, void *dst, size_t bytes);

static esp_err_t show_crc32_file(FILE *f, uint32_t file_bytes, uint32_t *out)
{
    if (!f || !out) return ESP_ERR_INVALID_ARG;
    if (fseek(f, 0, SEEK_SET) != 0) return ESP_FAIL;

    uint8_t buf[512];
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    uint32_t file_pos = 0;
    const uint32_t crc_pos = (uint32_t)offsetof(show_file_header_v1_t, crc32);
    while (file_pos < file_bytes) {
        size_t want = file_bytes - file_pos;
        if (want > sizeof(buf)) want = sizeof(buf);
        if (fread(buf, 1, want, f) != want) return ESP_FAIL;
        for (size_t i = 0; i < want; ++i) {
            uint32_t absolute = file_pos + (uint32_t)i;
            if (absolute >= crc_pos && absolute < crc_pos + sizeof(uint32_t)) {
                buf[i] = 0;
            }
        }
        crc = show_crc32_update(crc, buf, want);
        file_pos += (uint32_t)want;
    }
    *out = crc ^ UINT32_C(0xFFFFFFFF);
    return ESP_OK;
}

static size_t show_effect_base_param_bytes(uint8_t effect_kind)
{
    if (effect_kind == SHOW_EFFECT_SOLID) {
        return sizeof(show_param_solid_v1_t);
    }
    switch (effect_kind) {
        case SHOW_EFFECT_BLINK:
        case SHOW_EFFECT_PULSE:
        case SHOW_EFFECT_SWEEP:
        case SHOW_EFFECT_MIRROR_SWEEP:
        case SHOW_EFFECT_CHASE:
        case SHOW_EFFECT_SPARKLE:
        case SHOW_EFFECT_FANOUT:
        case SHOW_EFFECT_GLOBAL_SWEEP:
        case SHOW_EFFECT_TRAVELING_ORB:
        case SHOW_EFFECT_GROUND_ENERGY:
        case SHOW_EFFECT_RADIAL_RAY:
            return sizeof(show_param_spatial_v1_t);
        default:
            return 0;
    }
}

static bool show_effect_param_size_valid(uint8_t effect_kind, uint16_t param_bytes)
{
    size_t base_bytes = show_effect_base_param_bytes(effect_kind);
    if (base_bytes == 0) return false;
    if (param_bytes == base_bytes) return true;
    if (param_bytes < base_bytes + sizeof(show_color_anim_v1_1_t)) return false;
    size_t stop_bytes = param_bytes - base_bytes - sizeof(show_color_anim_v1_1_t);
    return stop_bytes > 0 && (stop_bytes % sizeof(show_color_stop_v1_1_t)) == 0 &&
           stop_bytes / sizeof(show_color_stop_v1_1_t) <= 16u;
}

static bool show_color_extension_valid(FILE *f, const show_clip_v1_t *clip)
{
    size_t base_bytes = show_effect_base_param_bytes(clip->effect_kind);
    if (clip->param_bytes == base_bytes) return true;

    show_color_anim_v1_1_t animation;
    if (show_read_exact(f, clip->param_offset + (uint32_t)base_bytes,
                        &animation, sizeof(animation)) != ESP_OK ||
        animation.mode < SHOW_COLOR_MODE_LINEAR ||
        animation.mode > SHOW_COLOR_MODE_CYCLE ||
        (animation.flags & ~(SHOW_COLOR_FLAG_TEMPO_SYNC |
                             SHOW_COLOR_FLAG_FIT_CLIP)) != 0 ||
        animation.stop_count == 0 || animation.stop_count > 16u ||
        clip->param_bytes != base_bytes + sizeof(animation) +
                             animation.stop_count * sizeof(show_color_stop_v1_1_t)) {
        return false;
    }

    uint16_t previous_position = 0;
    for (uint8_t i = 0; i < animation.stop_count; ++i) {
        show_color_stop_v1_1_t stop;
        uint32_t offset = clip->param_offset + (uint32_t)base_bytes +
                          sizeof(animation) + i * sizeof(stop);
        if (show_read_exact(f, offset, &stop, sizeof(stop)) != ESP_OK ||
            stop.position_1024 > 1024u ||
            (i > 0 && stop.position_1024 < previous_position)) {
            return false;
        }
        previous_position = stop.position_1024;
    }
    return true;
}

static bool show_clip_valid(const show_file_header_v1_t *hdr,
                            const show_clip_v1_t *clip)
{
    if (!hdr || !clip) return false;
    if (clip->start_ms >= clip->end_ms || clip->end_ms > hdr->duration_ms) return false;
    if (clip->blend_mode > SHOW_BLEND_MAX) return false;
    if (!show_effect_param_size_valid(clip->effect_kind, clip->param_bytes)) return false;
    if (clip->param_offset < hdr->param_blob_offset ||
        !show_range_valid(clip->param_offset, clip->param_bytes, 1,
                          hdr->string_table_offset)) {
        return false;
    }

    switch (clip->target_kind) {
        case SHOW_TARGET_KIND_ALL:
            return clip->target_id == 0;
        case SHOW_TARGET_KIND_SECTION:
            return show_section_name((show_section_id_t)clip->target_id) != NULL;
        case SHOW_TARGET_KIND_GROUP:
            return show_group_name((show_group_id_t)clip->target_id) != NULL;
        default:
            return false;
    }
}

static esp_err_t show_read_exact(FILE *f, uint32_t offset, void *dst, size_t bytes)
{
    if (!f || (!dst && bytes > 0)) return ESP_ERR_INVALID_ARG;
    if (bytes == 0) return ESP_OK;
    if (offset > (uint32_t)LONG_MAX || fseek(f, (long)offset, SEEK_SET) != 0) {
        return ESP_FAIL;
    }
    return fread(dst, 1, bytes, f) == bytes ? ESP_OK : ESP_FAIL;
}

static esp_err_t show_validate_file(FILE *f, show_validated_file_t *out,
                                    show_package_info_t *info)
{
    if (!f || !out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));
    if (info) memset(info, 0, sizeof(*info));

    ESP_RETURN_ON_ERROR(show_read_exact(f, 0, &out->header, sizeof(out->header)),
                        TAG, "read header");
    const show_file_header_v1_t *hdr = &out->header;
    if (hdr->magic != SHOW_FILE_MAGIC_V1 ||
        hdr->version_major != SHOW_FILE_VERSION_MAJOR_V1 ||
        hdr->version_minor > SHOW_FILE_VERSION_MINOR_V1 ||
        hdr->header_bytes < sizeof(*hdr) ||
        hdr->file_bytes < hdr->header_bytes ||
        hdr->role_count == 0 || hdr->role_count > SHOW_MAX_ROLES ||
        hdr->duration_ms == 0 || hdr->bucket_ms == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (hdr->version_minor >= 1u) {
        if (hdr->header_bytes < sizeof(*hdr) + sizeof(out->timing)) {
            return ESP_ERR_INVALID_SIZE;
        }
        ESP_RETURN_ON_ERROR(show_read_exact(f, sizeof(*hdr), &out->timing,
                                            sizeof(out->timing)),
                            TAG, "read timing");
    }

    if (fseek(f, 0, SEEK_END) != 0) return ESP_FAIL;
    long actual_size = ftell(f);
    if (actual_size < 0 || (uint32_t)actual_size != hdr->file_bytes) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (!(hdr->header_bytes <= hdr->role_table_offset &&
          hdr->role_table_offset <= hdr->palette_table_offset &&
          hdr->palette_table_offset <= hdr->group_table_offset &&
          hdr->group_table_offset <= hdr->bucket_table_offset &&
          hdr->bucket_table_offset <= hdr->clip_table_offset &&
          hdr->clip_table_offset <= hdr->param_blob_offset &&
          hdr->param_blob_offset <= hdr->string_table_offset &&
          hdr->string_table_offset <= hdr->file_bytes)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!show_range_valid(hdr->role_table_offset, hdr->role_count,
                          sizeof(show_role_program_v1_t),
                          hdr->palette_table_offset)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (hdr->palette_count != 0 || hdr->group_count != 0 ||
        hdr->palette_table_offset != hdr->group_table_offset ||
        hdr->group_table_offset != hdr->bucket_table_offset) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint32_t bucket_bytes = hdr->clip_table_offset - hdr->bucket_table_offset;
    uint32_t clip_bytes = hdr->param_blob_offset - hdr->clip_table_offset;
    if ((bucket_bytes % sizeof(show_bucket_v1_t)) != 0 ||
        (clip_bytes % sizeof(show_clip_v1_t)) != 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    out->bucket_total = bucket_bytes / sizeof(show_bucket_v1_t);
    out->clip_total = clip_bytes / sizeof(show_clip_v1_t);

    ESP_RETURN_ON_ERROR(show_read_exact(f, hdr->role_table_offset, out->roles,
                                        hdr->role_count * sizeof(out->roles[0])),
                        TAG, "read roles");
    uint8_t role_seen = 0;
    for (uint16_t i = 0; i < hdr->role_count; ++i) {
        const show_role_program_v1_t *role = &out->roles[i];
        if (role->role_id >= SHOW_MAX_ROLES || (role_seen & (1u << role->role_id)) ||
            role->bucket_first > out->bucket_total ||
            role->bucket_count > out->bucket_total - role->bucket_first ||
            role->clip_first > out->clip_total ||
            role->clip_count > out->clip_total - role->clip_first) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        role_seen |= (uint8_t)(1u << role->role_id);

        for (uint32_t b = 0; b < role->bucket_count; ++b) {
            show_bucket_v1_t bucket;
            uint32_t index = role->bucket_first + b;
            uint32_t offset = hdr->bucket_table_offset + index * sizeof(bucket);
            ESP_RETURN_ON_ERROR(show_read_exact(f, offset, &bucket, sizeof(bucket)),
                                TAG, "read bucket");
            uint64_t expected_start = (uint64_t)b * hdr->bucket_ms;
            if (expected_start >= hdr->duration_ms ||
                bucket.bucket_start_ms != (uint32_t)expected_start ||
                bucket.clip_first > role->clip_count ||
                bucket.clip_count > role->clip_count - bucket.clip_first) {
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    for (uint32_t i = 0; i < out->clip_total; ++i) {
        show_clip_v1_t clip;
        uint32_t offset = hdr->clip_table_offset + i * sizeof(clip);
        ESP_RETURN_ON_ERROR(show_read_exact(f, offset, &clip, sizeof(clip)),
                            TAG, "read clip");
        if (!show_clip_valid(hdr, &clip) || !show_color_extension_valid(f, &clip)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    uint32_t crc = 0;
    ESP_RETURN_ON_ERROR(show_crc32_file(f, hdr->file_bytes, &crc), TAG, "crc");
    if (crc != hdr->crc32) return ESP_ERR_INVALID_CRC;

    if (info) {
        info->show_uid = hdr->show_uid;
        info->duration_ms = hdr->duration_ms;
        info->file_bytes = hdr->file_bytes;
        info->version_major = hdr->version_major;
        info->version_minor = hdr->version_minor;
        info->role_count = hdr->role_count;
        info->fps_hint = hdr->fps_hint;
        info->bucket_ms = hdr->bucket_ms;
        info->palette_count = hdr->palette_count;
        info->group_count = hdr->group_count;
        info->tempo_millibpm = out->timing.tempo_millibpm;
        info->beat_offset_ms = out->timing.beat_offset_ms;
        info->crc_checked = true;
        info->crc_valid = true;
        for (uint16_t i = 0; i < hdr->role_count; ++i) {
            uint8_t id = out->roles[i].role_id;
            info->roles[id].role_id = id;
            info->roles[id].bucket_count = out->roles[i].bucket_count;
            info->roles[id].clip_count = out->roles[i].clip_count;
        }
    }
    return ESP_OK;
}

static void *show_alloc(size_t bytes)
{
    if (bytes == 0) return NULL;
    void *ptr = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return ptr ? ptr : heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
}

esp_err_t show_package_build_path(const char *show_slug, char *out, size_t out_sz)
{
    if (!show_slug_valid(show_slug) || !out || out_sz == 0) return ESP_ERR_INVALID_ARG;
    char rel[96];
    int len = snprintf(rel, sizeof(rel), "shows/%s/show.bin", show_slug);
    if (len <= 0 || (size_t)len >= sizeof(rel)) return ESP_ERR_INVALID_SIZE;
    return storage_sd_make_path(out, out_sz, rel);
}

esp_err_t show_package_load_info_from_path(const char *path, show_package_info_t *out)
{
    if (!path || !path[0] || !out) return ESP_ERR_INVALID_ARG;
    FILE *f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;
    show_validated_file_t valid;
    esp_err_t err = show_validate_file(f, &valid, out);
    fclose(f);
    return err;
}

esp_err_t show_package_load_info(const char *show_slug, show_package_info_t *out)
{
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount sd");
    char path[128];
    ESP_RETURN_ON_ERROR(show_package_build_path(show_slug, path, sizeof(path)), TAG, "path");
    return show_package_load_info_from_path(path, out);
}

void show_package_program_free(show_package_program_t *program)
{
    if (!program) return;
    heap_caps_free(program->clips);
    heap_caps_free(program->buckets);
    heap_caps_free(program->param_blob);
    memset(program, 0, sizeof(*program));
}

esp_err_t show_package_load_program_from_path(const char *path, uint8_t role_id,
                                              show_package_program_t *out)
{
    if (!path || !path[0] || !out || role_id >= SHOW_MAX_ROLES) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    FILE *f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;

    show_validated_file_t valid;
    esp_err_t err = show_validate_file(f, &valid, &out->info);
    if (err != ESP_OK) {
        fclose(f);
        return err;
    }

    const show_role_program_v1_t *selected = NULL;
    for (uint16_t i = 0; i < valid.header.role_count; ++i) {
        if (valid.roles[i].role_id == role_id) {
            selected = &valid.roles[i];
            break;
        }
    }
    if (!selected) {
        fclose(f);
        return ESP_ERR_NOT_FOUND;
    }
    out->role = *selected;

    size_t buckets_bytes = (size_t)selected->bucket_count * sizeof(show_bucket_v1_t);
    size_t clips_bytes = (size_t)selected->clip_count * sizeof(show_clip_v1_t);
    size_t param_bytes = valid.header.string_table_offset - valid.header.param_blob_offset;
    out->buckets = show_alloc(buckets_bytes);
    out->clips = show_alloc(clips_bytes);
    out->param_blob = show_alloc(param_bytes);
    if ((buckets_bytes && !out->buckets) || (clips_bytes && !out->clips) ||
        (param_bytes && !out->param_blob)) {
        fclose(f);
        show_package_program_free(out);
        return ESP_ERR_NO_MEM;
    }

    uint32_t buckets_offset = valid.header.bucket_table_offset +
                              selected->bucket_first * sizeof(show_bucket_v1_t);
    uint32_t clips_offset = valid.header.clip_table_offset +
                            selected->clip_first * sizeof(show_clip_v1_t);
    err = show_read_exact(f, buckets_offset, out->buckets, buckets_bytes);
    if (err == ESP_OK) {
        err = show_read_exact(f, clips_offset, out->clips, clips_bytes);
    }
    if (err == ESP_OK) {
        err = show_read_exact(f, valid.header.param_blob_offset,
                              out->param_blob, param_bytes);
    }
    fclose(f);
    if (err != ESP_OK) {
        show_package_program_free(out);
        return err;
    }
    out->param_blob_bytes = param_bytes;
    out->param_blob_file_offset = valid.header.param_blob_offset;
    return ESP_OK;
}

esp_err_t show_package_load_program(const char *show_slug, uint8_t role_id,
                                    show_package_program_t *out)
{
    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "mount sd");
    char path[128];
    ESP_RETURN_ON_ERROR(show_package_build_path(show_slug, path, sizeof(path)), TAG, "path");
    return show_package_load_program_from_path(path, role_id, out);
}

const void *show_package_program_param(const show_package_program_t *program,
                                       const show_clip_v1_t *clip,
                                       size_t expected_bytes)
{
    if (!program || !clip || !program->param_blob ||
        clip->param_bytes < expected_bytes ||
        clip->param_offset < program->param_blob_file_offset) {
        return NULL;
    }
    size_t offset = clip->param_offset - program->param_blob_file_offset;
    if (offset > program->param_blob_bytes ||
        expected_bytes > program->param_blob_bytes - offset) {
        return NULL;
    }
    return program->param_blob + offset;
}
