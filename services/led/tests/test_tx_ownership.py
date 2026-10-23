"""Check that a new logical frame cannot overwrite frames being transmitted."""

from pathlib import Path
import subprocess
import tempfile
import unittest


class TxOwnershipTest(unittest.TestCase):
    def test_pending_frame_is_separate_from_driver_frames(self):
        source = (Path(__file__).resolve().parents[1] / "led.c").read_text()
        start = source.index("static esp_err_t led_flush_frame")
        end = source.index("static void led_tx_task", start)
        functions = source[start:end]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define LED_OUTPUT_COUNT 2
#define LED_COUNT 4
#define ESP_RETURN_ON_ERROR(expr,...) do { int e=(expr); if(e) return e; } while(0)
static uint8_t physical[2][12], pending[12], next_frame[12];
static uint8_t *s_output_frames[2] = {physical[0], physical[1]};
static uint8_t *s_frame = pending;
static size_t s_output_lengths[2] = {4, 4}, s_tx_len_bytes;
static bool s_inited = true, tx_locked, inject;
static int s_tx_task = 1, notifications, writes;
static esp_err_t led_submit_async_locked(const uint8_t *, size_t);
static int led_tx_lock_take(void) { assert(!tx_locked); tx_locked = true; return 0; }
static void led_tx_lock_give(void) { assert(tx_locked); tx_locked = false; }
static void led_apply_audio_brightness_inplace(uint8_t *p, size_t n) {(void)p;(void)n;}
static void xTaskNotifyGive(int task) {(void)task; ++notifications;}
static void led_split_frame(const uint8_t *p, size_t pixels) {
    assert(tx_locked);
    for (int i = 0; i < 2; ++i) memcpy(physical[i], p, pixels * 3);
}
static int led_strip_write(const uint8_t *frames[2], const size_t lengths[2]) {
    assert(tx_locked && frames[0] == physical[0] && lengths[0] == 4);
    uint8_t saved[2][12]; memcpy(saved, physical, sizeof(saved));
    if (inject) {
        assert(led_submit_async_locked(next_frame, sizeof(next_frame)) == ESP_OK);
        assert(memcmp(saved, physical, sizeof(saved)) == 0);
        inject = false;
    }
    ++writes;
    return ESP_OK;
}
FUNCTIONS
int main(void) {
    uint8_t first[12];
    memset(first, 17, sizeof(first));
    memset(next_frame, 99, sizeof(next_frame));
    inject = true;
    assert(led_flush_frame(first, sizeof(first)) == ESP_OK);
    assert(writes == 1 && notifications == 1 && !tx_locked);
    assert(physical[0][0] == 17 && pending[0] == 99);
    assert(led_flush_frame(pending, sizeof(pending)) == ESP_OK);
    assert(physical[0][0] == 99 && writes == 2);
}
'''.replace("FUNCTIONS", functions)
        with tempfile.TemporaryDirectory(prefix="suit-led-tx-") as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(harness)
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", str(path / "test.c"),
                "-o", str(path / "test")
            ], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
