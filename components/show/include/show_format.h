#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHOW_FILE_MAGIC_V1 UINT32_C(0x31535753) /* 'SWS1' little-endian */
#define SHOW_FILE_VERSION_MAJOR_V1 1u
#define SHOW_FILE_VERSION_MINOR_V1 1u
#define SHOW_MAX_ROLES 3u

typedef enum {
    SHOW_ROLE_A = 0,
    SHOW_ROLE_B = 1,
    SHOW_ROLE_C = 2,
} show_role_id_t;

typedef enum {
    SHOW_TARGET_KIND_INVALID = 0,
    SHOW_TARGET_KIND_ALL = 1,
    SHOW_TARGET_KIND_SECTION = 2,
    SHOW_TARGET_KIND_GROUP = 3,
} show_target_kind_t;

typedef enum {
    SHOW_BLEND_REPLACE = 0,
    SHOW_BLEND_ALPHA = 1,
    SHOW_BLEND_ADD = 2,
    SHOW_BLEND_MAX = 3,
} show_blend_mode_t;

typedef enum {
    SHOW_CLIP_FLAG_TEMPO_SYNC = (1u << 0),
} show_clip_flag_t;

typedef enum {
    SHOW_EFFECT_INVALID = 0,
    SHOW_EFFECT_SOLID = 1,
    SHOW_EFFECT_BLINK = 2,
    SHOW_EFFECT_PULSE = 3,
    SHOW_EFFECT_STROBE = 4,  // legacy reserved slot; strobe authoring support removed
    SHOW_EFFECT_FADE = 5,  // legacy reserved slot; fade authoring support removed
    SHOW_EFFECT_SWEEP = 6,
    SHOW_EFFECT_MIRROR_SWEEP = 7,
    SHOW_EFFECT_CHASE = 8,
    SHOW_EFFECT_GRADIENT = 9,  // legacy reserved slot; gradient authoring support removed
    SHOW_EFFECT_SPARKLE = 10,
    SHOW_EFFECT_FANOUT = 11,
    SHOW_EFFECT_PALETTE_CYCLE = 12,  // legacy reserved slot; palette-cycle authoring support removed
    SHOW_EFFECT_GLOBAL_SWEEP = 13,
    SHOW_EFFECT_TRAVELING_ORB = 14,
    SHOW_EFFECT_RING_BURST = 15,  // legacy reserved slot; ring-burst authoring support removed
    SHOW_EFFECT_GROUND_ENERGY = 16,
    SHOW_EFFECT_RADIAL_RAY = 17,
} show_effect_kind_t;

typedef enum {
    SHOW_AXIS_Y = 0,
    SHOW_AXIS_X = 1,
    SHOW_AXIS_Z = 2,
    SHOW_AXIS_SECTION_U = 3,
    SHOW_AXIS_RADIAL = 4,
    SHOW_AXIS_RANDOM_XY = 5,
} show_axis_kind_t;

typedef enum {
    SHOW_SPATIAL_OPTION_RANDOM_CROSS_X = (1u << 15),
} show_spatial_option_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version_major;
    uint16_t version_minor;
    uint32_t header_bytes;
    uint32_t file_bytes;
    uint32_t crc32;
    uint64_t show_uid;
    uint32_t duration_ms;
    uint16_t role_count;
    uint16_t fps_hint;
    uint16_t bucket_ms;
    uint16_t palette_count;
    uint16_t group_count;
    uint16_t reserved0;
    uint32_t role_table_offset;
    uint32_t palette_table_offset;
    uint32_t group_table_offset;
    uint32_t bucket_table_offset;
    uint32_t clip_table_offset;
    uint32_t param_blob_offset;
    uint32_t string_table_offset;
} show_file_header_v1_t;

/* Appended between the base header and role table when version_minor >= 1. */
typedef struct __attribute__((packed)) {
    uint32_t tempo_millibpm;
    int32_t beat_offset_ms;
} show_file_timing_v1_1_t;

typedef struct __attribute__((packed)) {
    uint8_t role_id;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t bucket_first;
    uint32_t bucket_count;
    uint32_t clip_first;
    uint32_t clip_count;
} show_role_program_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t bucket_start_ms;
    uint32_t clip_first;
    uint16_t clip_count;
    uint16_t reserved0;
} show_bucket_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t start_ms;
    uint32_t end_ms;
    uint16_t layer;
    uint8_t effect_kind;
    uint8_t blend_mode;
    uint8_t target_kind;
    uint8_t reserved0;
    uint32_t target_id;
    uint16_t palette_id;
    uint16_t flags;
    uint16_t fade_in_ms_div10;
    uint16_t fade_out_ms_div10;
    uint32_t param_offset;
    uint16_t param_bytes;
    uint16_t seed;
} show_clip_v1_t;

typedef struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
    uint16_t intensity_1024;
    uint16_t reserved0;
} show_param_solid_v1_t;

typedef struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
    uint8_t to_r;
    uint8_t to_g;
    uint8_t to_b;
    uint8_t to_a;
    uint8_t axis_kind;
    uint8_t reverse;
    uint16_t intensity_1024;
    uint16_t width_1024;
    uint16_t softness_1024;
    uint16_t frequency_1024;
    uint16_t phase_1024;
    uint16_t repeats;
    uint16_t duty_cycle_1024;
    uint16_t min_intensity_1024;
    uint16_t max_intensity_1024;
    uint16_t axis_mix_1024;
} show_param_spatial_v1_t;

typedef enum {
    SHOW_COLOR_MODE_HOLD = 0,
    SHOW_COLOR_MODE_LINEAR = 1,
    SHOW_COLOR_MODE_SMOOTH = 2,
    SHOW_COLOR_MODE_CYCLE = 3,
} show_color_mode_t;

typedef enum {
    SHOW_COLOR_FLAG_TEMPO_SYNC = (1u << 0),
    SHOW_COLOR_FLAG_FIT_CLIP = (1u << 1),
} show_color_flag_t;

/* Optional suffix after a v1 solid/spatial parameter block. */
typedef struct __attribute__((packed)) {
    uint8_t mode;
    uint8_t flags;
    uint8_t stop_count;
    uint8_t reserved0;
    uint32_t rate_million;
} show_color_anim_v1_1_t;

typedef struct __attribute__((packed)) {
    uint16_t position_1024;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
} show_color_stop_v1_1_t;

#ifdef __cplusplus
}
#endif
