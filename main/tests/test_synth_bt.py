"""Exercise production Bluetooth generator bridge without the Bluetooth stack."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

class SynthBluetoothTest(unittest.TestCase):
    def test_generator_bridge(self):
        source = (ROOT / 'components/bt_audio/bt_audio_stream_file.c').read_text()
        bridge = source[source.index('static portMUX_TYPE s_generator_mux'):source.index('// ============================ File-backed')]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int esp_err_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
typedef size_t (*bt_audio_generator_t)(int16_t *, size_t, int, void *);
static bool connected=true;
static size_t queued;
static int rate=44100, volume=100, calls;
static bool short_read;
static bool bt_audio_can_stream(void){return connected;}
static size_t bt_audio_stream_queued_frames(void){return queued;}
static int bt_audio_get_peer_sample_rate(void){return rate;}
static int bt_audio_volume_get_percent(void){return volume;}
static void vTaskDelay(int n){(void)n;assert(0);}
BRIDGE
static size_t generate(int16_t *out,size_t n,int hz,void *user){
    assert(hz==rate && user==&calls && n<=128);
    calls++;
    if(short_read)n/=2;
    for(size_t i=0;i<n;i++)out[i]=20000;
    return n;
}
int main(void){
    uint8_t buffer[1025];
    assert(!generate_live(buffer,sizeof(buffer)));
    connected=false;assert(bt_audio_set_generator(generate,&calls)!=ESP_OK);
    connected=true;queued=5;assert(bt_audio_set_generator(generate,&calls)!=ESP_OK);
    queued=0;assert(bt_audio_set_generator(generate,&calls)==ESP_OK);
    assert(bt_audio_set_generator(generate,&calls)!=ESP_OK);
    for(int k=0;k<2;k++){
        rate=k?48000:44100;
        memset(buffer,0xff,sizeof(buffer));
        assert(generate_live(buffer,sizeof(buffer)));
        for(int i=0;i<1024;i+=2){int16_t x;memcpy(&x,buffer+i,2);assert(x==20000);}
        assert(buffer[1024]==0);
    }
    volume=200;generate_live(buffer,sizeof(buffer));
    int16_t x;memcpy(&x,buffer,2);assert(x==32767);
    short_read=true;generate_live(buffer,sizeof(buffer));
    for(int i=256;i<1025;i++)assert(buffer[i]==0);
    rate=0;int before=calls;generate_live(buffer,sizeof(buffer));
    assert(before==calls);
    for(int i=0;i<1025;i++)assert(buffer[i]==0);
    assert(bt_audio_set_generator(NULL,NULL)==ESP_OK);
    assert(!generate_live(buffer,sizeof(buffer)));
}
'''.replace('BRIDGE', bridge)
        with tempfile.TemporaryDirectory(prefix='suit-synth-bt-') as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(harness)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', str(path / 'test.c'),
                            '-o', str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], check=True)

if __name__ == '__main__':
    unittest.main()
