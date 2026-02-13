#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t bt_audio_a2dp_data_cb(uint8_t *data, int32_t len);
void bt_audio_stream_close(void);
size_t bt_audio_stream_queued_frames(void);
void bt_audio_stream_set_paused(bool paused);
void bt_audio_invoke_disconnect_cb(void);

#ifdef __cplusplus
}
#endif
