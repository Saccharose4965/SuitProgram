from pathlib import Path
import subprocess
import tempfile
import unittest


class CallPacketsTest(unittest.TestCase):
    def test_packet_validation_and_audio_ownership(self):
        code = (Path(__file__).resolve().parents[1] / 'app_call.c').read_text()
        receive = code[code.index('void call_app_receive('):code.index('static void call_worker(')]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define LINK_MSG_CALL 5
#define CALL_SAMPLES 92
typedef int link_msg_type_t;
typedef struct { uint8_t mac[6];uint8_t data[196]; } call_packet_t;
static bool s_active,s_talk;static int s_packets=1,copies;
static call_packet_t last;
static int xQueueSend(int q,const void*p,int t){(void)q;(void)t;memcpy(&last,p,sizeof(last));++copies;return 1;}
FUNCTION
int main(void){
    uint8_t data[196]={0},mac[6]={1};
    memcpy(data,"CALL",4);uint16_t bytes=184;memcpy(data+8,&bytes,2);
    call_app_receive(5,mac,data,sizeof(data));assert(!copies);
    s_active=true;
    call_app_receive(5,mac,data,sizeof(data)-1);assert(!copies);
    call_app_receive(4,mac,data,sizeof(data));assert(!copies);
    call_app_receive(5,NULL,data,sizeof(data));assert(!copies);
    call_app_receive(5,mac,data,sizeof(data));assert(copies==1 && last.mac[0]==1);
    s_talk=true;call_app_receive(5,mac,data,sizeof(data));assert(copies==1);
    s_talk=false;data[8]=255;call_app_receive(5,mac,data,sizeof(data));assert(copies==1);
}
'''.replace('FUNCTION', receive)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(harness)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', str(path / 'test.c'),
                            '-o', str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], check=True)


if __name__ == '__main__':
    unittest.main()
