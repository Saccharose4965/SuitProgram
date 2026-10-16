"""Compile production LED flush/submit functions with deterministic RMT mocks."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class TxOwnershipTest(unittest.TestCase):
    def test_pending_frame_does_not_wait_for_wire(self):
        source = (Path(__file__).resolve().parents[1] / "led.c").read_text()
        start = source.index("static esp_err_t led_drain_outputs_locked")
        end = source.index("static void led_tx_task", start)
        functions = source[start:end]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_INVALID_ARG 3
#define LED_OUTPUT_COUNT 2
#define LED_COUNT 4
#define LED_RMT_TX_TIMEOUT_MS 200
#define ESP_RETURN_ON_ERROR(expr,...) do { int e=(expr); if(e) return e; } while(0)
#define ESP_LOGW(...) ((void)0)
typedef struct { int loop_count; } rmt_transmit_config_t;
static struct { int chan, encoder; } s_outputs[2] = {{1,1},{2,2}};
static uint8_t physical[2][12], pending[12], next_frame[12];
static uint8_t *s_output_frames[2]={physical[0],physical[1]}, *s_frame=pending;
static size_t s_output_lengths[2]={4,4}, s_tx_len_bytes;
static bool s_inited=true, tx_locked, inject, timeout_once, stuck;
static bool inflight[2];
static int s_tx_task=1, notifications, splits, fail_submit;
static esp_err_t led_submit_async_locked(const uint8_t *, size_t);
static int led_tx_lock_take(void) { assert(!tx_locked); tx_locked=true; return 0; }
static void led_tx_lock_give(void) { assert(tx_locked); tx_locked=false; }
static void led_apply_audio_brightness_inplace(uint8_t *p, size_t n) { (void)p; (void)n; }
static void xTaskNotifyGive(int task) { (void)task; ++notifications; }
static void led_split_frame(const uint8_t *p, size_t n) {
    assert(tx_locked && !inflight[0] && !inflight[1]);
    ++splits;
    for(int i=0;i<2;++i) memcpy(physical[i],p,n*3);
}
static int rmt_transmit(int ch, int encoder, const void *p, size_t n, const rmt_transmit_config_t *cfg) {
    (void)encoder; (void)p; (void)n; (void)cfg;
    assert(tx_locked && !inflight[ch-1]);
    if(ch==fail_submit) return ESP_ERR_INVALID_STATE;
    inflight[ch-1]=true; return 0;
}
static int rmt_tx_wait_all_done(int ch, int timeout_ms) {
    assert(tx_locked && timeout_ms==LED_RMT_TX_TIMEOUT_MS);
    if (inflight[ch-1] && inject) {
        uint8_t saved[2][12]; memcpy(saved,physical,sizeof(saved));
        // Model the producer running while the TX task waits: it needs no TX lock.
        assert(led_submit_async_locked(next_frame,sizeof(next_frame))==0);
        assert(memcmp(saved,physical,sizeof(saved))==0);
        assert(memcmp(pending,next_frame,sizeof(pending))==0);
        inject=false;
    }
    if (inflight[ch-1] && (timeout_once || stuck)) {
        timeout_once=false; return ESP_ERR_TIMEOUT;
    }
    inflight[ch-1]=false; return 0;
}
static int rmt_disable(int ch) { inflight[ch-1]=false; return 0; }
static int rmt_enable(int ch) { (void)ch; return 0; }
FUNCTIONS
int main(void) {
    uint8_t first[12]; memset(first,17,sizeof(first)); memset(next_frame,99,sizeof(next_frame));
    inject=true;
    assert(led_flush_frame(first,sizeof(first))==0);
    assert(notifications==1 && !tx_locked && !inflight[0] && !inflight[1]);
    assert(physical[0][0]==17 && pending[0]==99); // in-flight and pending stay distinct
    assert(led_flush_frame(pending,sizeof(pending))==0);
    assert(physical[0][0]==99);
    fail_submit=2;
    assert(led_flush_frame(first,sizeof(first))==ESP_ERR_INVALID_STATE);
    assert(!inflight[0] && !inflight[1] && !tx_locked); // drain after partial submit
    fail_submit=0; timeout_once=true;
    assert(led_flush_frame(first,sizeof(first))==0); // bounded timeout recovery
    assert(!tx_locked && !inflight[0] && !inflight[1]);
    stuck=true; inflight[0]=true;
    int before=splits;
    assert(led_flush_frame(next_frame,sizeof(next_frame))==ESP_ERR_TIMEOUT);
    assert(splits==before && physical[0][0]==17 && !tx_locked); // no overwrite while busy
}
'''.replace("FUNCTIONS", functions)
        # No led_lock stub: the flush path must not acquire the shared state mutex.
        with tempfile.TemporaryDirectory(prefix="suit-led-tx-") as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(harness)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", str(path / "test.c"),
                            "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
