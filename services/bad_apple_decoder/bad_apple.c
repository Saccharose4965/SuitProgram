#include "bad_apple.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_log.h"

#include "heatshrink_decoder.h"

#define BAD_APPLE_DEFAULT_FPS 30
#define BAD_APPLE_MIN_FPS 1
#define BAD_APPLE_MAX_FPS 60
#define BAD_APPLE_MAX_CATCHUP_FRAMES 2

#define HS_IN_BUF_SIZE  2048
#define HS_OUT_BUF_SIZE 4096

typedef enum {
    STEP_FRAME_READY = 0,
    STEP_EOF,
    STEP_ERROR,
} step_result_t;

typedef struct {
    FILE *file;

    bad_apple_source_type_t source_type;
    bool loop;
    bool playing;
    bool paused;
    bool eof;
    bool frame_valid;

    uint8_t target_fps;
    float frame_period_sec;
    float frame_accum_sec;

    uint32_t frame_index;
    size_t file_size_bytes;
    esp_err_t last_err;
    char path[128];

    uint8_t frame[BAD_APPLE_FRAME_BYTES];
    size_t frame_fill;

    int32_t rle_marker;
    int32_t rle_runlength;
    uint8_t repeat_byte;
    uint32_t repeat_remaining;

    heatshrink_decoder hsd;
    bool hs_input_done;
    bool hs_done;
    uint8_t hs_in_buf[HS_IN_BUF_SIZE];
    size_t hs_in_len;
    size_t hs_in_pos;
    uint8_t hs_out_buf[HS_OUT_BUF_SIZE];
    size_t hs_out_len;
    size_t hs_out_pos;
} bad_apple_ctx_t;

static const char *TAG = "bad_apple";
static bad_apple_ctx_t s_ba = {
    .target_fps = BAD_APPLE_DEFAULT_FPS,
    .frame_period_sec = 1.0f / (float)BAD_APPLE_DEFAULT_FPS,
    .rle_marker = -1,
    .rle_runlength = -1,
    .last_err = ESP_OK,
};

static uint8_t clamp_fps(uint8_t fps)
{
    if (fps < BAD_APPLE_MIN_FPS) return BAD_APPLE_DEFAULT_FPS;
    if (fps > BAD_APPLE_MAX_FPS) return BAD_APPLE_MAX_FPS;
    return fps;
}

const char *bad_apple_source_type_name(bad_apple_source_type_t type)
{
    switch (type) {
        case BAD_APPLE_SOURCE_HEATSHRINK_RLE: return "hs+rle";
        case BAD_APPLE_SOURCE_RLE:            return "rle";
        case BAD_APPLE_SOURCE_RAW:            return "raw";
        case BAD_APPLE_SOURCE_AUTO:           return "auto";
        default:                              return "unknown";
    }
}

static bad_apple_source_type_t detect_source_type(const char *path)
{
    if (!path) return BAD_APPLE_SOURCE_HEATSHRINK_RLE;
    const char *dot = strrchr(path, '.');
    if (!dot) return BAD_APPLE_SOURCE_HEATSHRINK_RLE;

    if (strcasecmp(dot, ".hs") == 0) {
        return BAD_APPLE_SOURCE_HEATSHRINK_RLE;
    }
    if (strcasecmp(dot, ".rle") == 0) {
        return BAD_APPLE_SOURCE_RLE;
    }
    if (strcasecmp(dot, ".raw") == 0 ||
        strcasecmp(dot, ".bin") == 0 ||
        strcasecmp(dot, ".uc") == 0) {
        return BAD_APPLE_SOURCE_RAW;
    }

    return BAD_APPLE_SOURCE_HEATSHRINK_RLE;
}

static void reset_decode_state(void)
{
    s_ba.frame_fill = 0;
    s_ba.rle_marker = -1;
    s_ba.rle_runlength = -1;
    s_ba.repeat_byte = 0;
    s_ba.repeat_remaining = 0;

    heatshrink_decoder_reset(&s_ba.hsd);
    s_ba.hs_input_done = false;
    s_ba.hs_done = false;
    s_ba.hs_in_len = 0;
    s_ba.hs_in_pos = 0;
    s_ba.hs_out_len = 0;
    s_ba.hs_out_pos = 0;
}

static void close_file(void)
{
    if (s_ba.file) {
        fclose(s_ba.file);
        s_ba.file = NULL;
    }
}

static esp_err_t open_path(const char *path, bad_apple_source_type_t type)
{
    if (!path || !path[0]) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        return ESP_ERR_NOT_FOUND;
    }

    close_file();
    s_ba.file = f;

    strncpy(s_ba.path, path, sizeof(s_ba.path) - 1);
    s_ba.path[sizeof(s_ba.path) - 1] = '\0';

    s_ba.source_type = (type == BAD_APPLE_SOURCE_AUTO) ? detect_source_type(path) : type;

    struct stat st;
    if (stat(path, &st) == 0 && st.st_size >= 0) {
        s_ba.file_size_bytes = (size_t)st.st_size;
    } else {
        s_ba.file_size_bytes = 0;
    }

    reset_decode_state();
    s_ba.eof = false;
    s_ba.frame_valid = false;
    s_ba.frame_index = 0;
    s_ba.frame_accum_sec = 0.0f;
    s_ba.last_err = ESP_OK;

    ESP_LOGI(TAG, "opened %s (%s, %u bytes)",
             s_ba.path,
             bad_apple_source_type_name(s_ba.source_type),
             (unsigned)s_ba.file_size_bytes);

    return ESP_OK;
}

static esp_err_t reopen_current_path(void)
{
    if (!s_ba.path[0]) {
        return ESP_ERR_INVALID_STATE;
    }
    return open_path(s_ba.path, s_ba.source_type);
}

static int file_next_byte(uint8_t *out)
{
    if (!s_ba.file || !out) return -1;

    size_t n = fread(out, 1, 1, s_ba.file);
    if (n == 1) return 1;
    if (ferror(s_ba.file)) return -1;
    return 0;
}

static int hs_next_rle_byte(uint8_t *out)
{
    if (!out || !s_ba.file) return -1;

    for (;;) {
        if (s_ba.hs_out_pos < s_ba.hs_out_len) {
            *out = s_ba.hs_out_buf[s_ba.hs_out_pos++];
            return 1;
        }

        size_t produced = 0;
        HSD_poll_res pres = heatshrink_decoder_poll(
            &s_ba.hsd,
            s_ba.hs_out_buf,
            sizeof(s_ba.hs_out_buf),
            &produced);
        if (pres < 0) {
            return -1;
        }
        if (produced > 0) {
            s_ba.hs_out_pos = 0;
            s_ba.hs_out_len = produced;
            continue;
        }

        if (!s_ba.hs_input_done) {
            if (s_ba.hs_in_pos >= s_ba.hs_in_len) {
                s_ba.hs_in_len = fread(s_ba.hs_in_buf, 1, sizeof(s_ba.hs_in_buf), s_ba.file);
                s_ba.hs_in_pos = 0;
                if (s_ba.hs_in_len == 0) {
                    if (ferror(s_ba.file)) {
                        return -1;
                    }
                    s_ba.hs_input_done = true;
                }
            }

            if (!s_ba.hs_input_done) {
                size_t sunk = 0;
                HSD_sink_res sres = heatshrink_decoder_sink(
                    &s_ba.hsd,
                    &s_ba.hs_in_buf[s_ba.hs_in_pos],
                    s_ba.hs_in_len - s_ba.hs_in_pos,
                    &sunk);
                if (sres < 0) {
                    return -1;
                }
                s_ba.hs_in_pos += sunk;
                continue;
            }
        }

        if (!s_ba.hs_done) {
            HSD_finish_res fres = heatshrink_decoder_finish(&s_ba.hsd);
            if (fres < 0) {
                return -1;
            }
            if (fres == HSDR_FINISH_DONE) {
                s_ba.hs_done = true;
            }
            continue;
        }

        return 0;
    }
}

static int next_rle_stream_byte(uint8_t *out)
{
    if (!out) return -1;
    if (s_ba.source_type == BAD_APPLE_SOURCE_RLE) {
        return file_next_byte(out);
    }
    if (s_ba.source_type == BAD_APPLE_SOURCE_HEATSHRINK_RLE) {
        return hs_next_rle_byte(out);
    }
    return -1;
}

static bool emit_frame_byte(uint8_t value)
{
    if (s_ba.frame_fill >= BAD_APPLE_FRAME_BYTES) {
        s_ba.frame_fill = 0;
    }

    s_ba.frame[s_ba.frame_fill++] = value;
    if (s_ba.frame_fill == BAD_APPLE_FRAME_BYTES) {
        s_ba.frame_fill = 0;
        s_ba.frame_index++;
        s_ba.frame_valid = true;
        return true;
    }

    return false;
}

static step_result_t decode_next_frame_raw(void)
{
    if (!s_ba.file) {
        return STEP_ERROR;
    }

    size_t total = 0;
    while (total < BAD_APPLE_FRAME_BYTES) {
        size_t n = fread(&s_ba.frame[total], 1, BAD_APPLE_FRAME_BYTES - total, s_ba.file);
        if (n == 0) {
            if (ferror(s_ba.file)) {
                return STEP_ERROR;
            }
            return STEP_EOF;
        }
        total += n;
    }

    s_ba.frame_index++;
    s_ba.frame_valid = true;
    return STEP_FRAME_READY;
}

static step_result_t decode_next_frame_stream(void)
{
    s_ba.frame_fill = 0;

    for (;;) {
        while (s_ba.repeat_remaining > 0) {
            s_ba.repeat_remaining--;
            if (emit_frame_byte(s_ba.repeat_byte)) {
                return STEP_FRAME_READY;
            }
        }

        uint8_t c = 0;
        int rc = next_rle_stream_byte(&c);
        if (rc < 0) {
            return STEP_ERROR;
        }
        if (rc == 0) {
            if (s_ba.frame_fill == 0 &&
                s_ba.repeat_remaining == 0 &&
                s_ba.rle_marker < 0 &&
                s_ba.rle_runlength < 0) {
                return STEP_EOF;
            }
            return STEP_ERROR;
        }

        if (s_ba.rle_marker < 0) {
            if (c == 0x55 || c == 0xaa) {
                s_ba.rle_marker = (int32_t)c;
            } else if (emit_frame_byte(c)) {
                return STEP_FRAME_READY;
            }
            continue;
        }

        if (s_ba.rle_runlength < 0) {
            if (c == 0) {
                uint8_t literal = (uint8_t)s_ba.rle_marker;
                s_ba.rle_marker = -1;
                if (emit_frame_byte(literal)) {
                    return STEP_FRAME_READY;
                }
            } else if ((c & 0x80u) == 0) {
                s_ba.repeat_byte = (s_ba.rle_marker == 0x55) ? 0x00 : 0xFF;
                s_ba.repeat_remaining = (uint32_t)c;
                s_ba.rle_marker = -1;
            } else {
                s_ba.rle_runlength = (int32_t)(c & 0x7f);
            }
        } else {
            s_ba.repeat_byte = (s_ba.rle_marker == 0x55) ? 0x00 : 0xFF;
            s_ba.repeat_remaining = (uint32_t)(s_ba.rle_runlength | ((int32_t)c << 7));
            s_ba.rle_marker = -1;
            s_ba.rle_runlength = -1;
        }
    }
}

static step_result_t decode_next_frame_once(void)
{
    switch (s_ba.source_type) {
        case BAD_APPLE_SOURCE_RAW:
            return decode_next_frame_raw();
        case BAD_APPLE_SOURCE_RLE:
        case BAD_APPLE_SOURCE_HEATSHRINK_RLE:
            return decode_next_frame_stream();
        case BAD_APPLE_SOURCE_AUTO:
        default:
            return STEP_ERROR;
    }
}

static step_result_t decode_next_frame(void)
{
    for (int attempt = 0; attempt < 2; ++attempt) {
        step_result_t r = decode_next_frame_once();
        if (r != STEP_EOF) {
            return r;
        }
        if (!s_ba.loop) {
            return STEP_EOF;
        }

        esp_err_t err = reopen_current_path();
        if (err != ESP_OK) {
            s_ba.last_err = err;
            return STEP_ERROR;
        }
    }

    return STEP_EOF;
}

esp_err_t bad_apple_start(const bad_apple_config_t *cfg)
{
    if (!cfg || !cfg->path || !cfg->path[0]) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = open_path(cfg->path, cfg->type);
    if (err != ESP_OK) {
        s_ba.last_err = err;
        return err;
    }

    s_ba.target_fps = clamp_fps(cfg->target_fps);
    s_ba.frame_period_sec = 1.0f / (float)s_ba.target_fps;
    s_ba.loop = cfg->loop;
    s_ba.playing = true;
    s_ba.paused = false;
    s_ba.eof = false;

    step_result_t step = decode_next_frame();
    if (step == STEP_FRAME_READY) {
        return ESP_OK;
    }

    if (step == STEP_EOF) {
        s_ba.last_err = ESP_ERR_INVALID_SIZE;
    } else {
        if (s_ba.last_err == ESP_OK) {
            s_ba.last_err = ESP_FAIL;
        }
    }

    s_ba.playing = false;
    s_ba.eof = true;
    close_file();
    return s_ba.last_err;
}

void bad_apple_stop(void)
{
    close_file();
    s_ba.playing = false;
    s_ba.paused = false;
    s_ba.eof = false;
    s_ba.frame_valid = false;
    s_ba.frame_accum_sec = 0.0f;
}

esp_err_t bad_apple_restart(void)
{
    esp_err_t err = reopen_current_path();
    if (err != ESP_OK) {
        s_ba.last_err = err;
        return err;
    }

    s_ba.playing = true;
    s_ba.paused = false;
    s_ba.eof = false;
    s_ba.frame_valid = false;
    s_ba.frame_accum_sec = 0.0f;

    step_result_t step = decode_next_frame();
    if (step == STEP_FRAME_READY) {
        s_ba.last_err = ESP_OK;
        return ESP_OK;
    }

    if (step == STEP_EOF) {
        s_ba.last_err = ESP_ERR_INVALID_SIZE;
    } else if (s_ba.last_err == ESP_OK) {
        s_ba.last_err = ESP_FAIL;
    }

    s_ba.playing = false;
    s_ba.eof = true;
    close_file();
    return s_ba.last_err;
}

void bad_apple_tick(float dt_sec)
{
    if (!s_ba.playing || s_ba.paused || !s_ba.file) return;
    if (dt_sec <= 0.0f) return;

    s_ba.frame_accum_sec += dt_sec;

    int stepped = 0;
    while (s_ba.frame_accum_sec >= s_ba.frame_period_sec && stepped < BAD_APPLE_MAX_CATCHUP_FRAMES) {
        s_ba.frame_accum_sec -= s_ba.frame_period_sec;

        step_result_t step = decode_next_frame();
        if (step == STEP_FRAME_READY) {
            stepped++;
            continue;
        }

        if (step == STEP_EOF) {
            s_ba.eof = true;
            s_ba.playing = false;
        } else {
            if (s_ba.last_err == ESP_OK) s_ba.last_err = ESP_FAIL;
            s_ba.playing = false;
            s_ba.eof = true;
        }
        break;
    }

    if (stepped == BAD_APPLE_MAX_CATCHUP_FRAMES && s_ba.frame_accum_sec > (s_ba.frame_period_sec * 2.0f)) {
        s_ba.frame_accum_sec = s_ba.frame_period_sec;
    }
}

esp_err_t bad_apple_copy_frame(uint8_t *dst, size_t dst_size)
{
    if (!dst || dst_size < BAD_APPLE_FRAME_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ba.frame_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(dst, s_ba.frame, BAD_APPLE_FRAME_BYTES);
    return ESP_OK;
}

void bad_apple_set_paused(bool paused)
{
    s_ba.paused = paused;
}

void bad_apple_toggle_pause(void)
{
    s_ba.paused = !s_ba.paused;
}

bool bad_apple_is_paused(void)
{
    return s_ba.paused;
}

void bad_apple_set_target_fps(uint8_t fps)
{
    s_ba.target_fps = clamp_fps(fps);
    s_ba.frame_period_sec = 1.0f / (float)s_ba.target_fps;
}

uint8_t bad_apple_get_target_fps(void)
{
    return s_ba.target_fps;
}

void bad_apple_get_status(bad_apple_status_t *out)
{
    if (!out) return;

    memset(out, 0, sizeof(*out));
    out->playing = s_ba.playing;
    out->paused = s_ba.paused;
    out->eof = s_ba.eof;
    out->frame_valid = s_ba.frame_valid;
    out->loop = s_ba.loop;
    out->target_fps = s_ba.target_fps;
    out->frame_index = s_ba.frame_index;
    out->file_size_bytes = s_ba.file_size_bytes;
    out->source_type = s_ba.source_type;
    out->last_err = s_ba.last_err;
    strncpy(out->path, s_ba.path, sizeof(out->path) - 1);
    out->path[sizeof(out->path) - 1] = '\0';
}
