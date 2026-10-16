"""Run production direct-message receive state machine against host queues/files."""
from pathlib import Path
import subprocess
import tempfile
import unittest

STUBS = r'''
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/stat.h>
#include <assert.h>
#include <setjmp.h>
typedef int esp_err_t;
typedef int link_msg_type_t;
typedef void *QueueHandle_t;
typedef unsigned TickType_t;
typedef void (*telemetry_progress_cb_t)(size_t,size_t,void *);
typedef struct {unsigned data_size,data_offset;} audio_wav_info_t;
#define ESP_OK 0
#define ESP_FAIL 1
#define ESP_ERR_NO_MEM 2
#define ESP_ERR_NOT_FOUND 3
#define ESP_ERR_INVALID_SIZE 4
#define ESP_ERR_TIMEOUT 5
#define LINK_MSG_MESSAGE 2
#define LINK_PATH_ESPNOW 1
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define pdPASS 1
static int link_active_path(void){return 1;}
static unsigned esp_random(void){return 42;}
static unsigned xTaskGetTickCount(void){return 1;}
static int storage_mount_sd(void){return 0;}
static int audio_wav_parse_file(FILE *f,audio_wav_info_t *i){i->data_size=5;i->data_offset=4;char h[4];return fread(h,1,4,f)==4 && !memcmp(h,"RIFF",4)?0:1;}
static const char *base(const char *p){const char *b=strrchr(p,'/');return b?b+1:p;}
static FILE *host_open(const char *p,const char *mode){return fopen(base(p),mode);}
static int host_remove(const char *p){return remove(base(p));}
static int host_rename(const char *a,const char *b){return rename(base(a),base(b));}
static int host_stat(const char *p,struct stat *s){return stat(base(p),s);}
#define fopen host_open
#define remove host_remove
#define rename host_rename
#define stat(p,s) host_stat(p,s)
#define mkdir(...) ((void)0)
static QueueHandle_t xQueueCreate(int n,size_t s){(void)n;(void)s;return (void *)1;}
static void vQueueDelete(QueueHandle_t q){(void)q;}
static void xQueueReset(QueueHandle_t q){(void)q;}
static int xTaskCreatePinnedToCore(void(*f)(void*),const char*n,int s,void*a,int p,void*t,int c)
{(void)f;(void)n;(void)s;(void)a;(void)p;(void)t;(void)c;return 1;}
static int xQueueSend(QueueHandle_t q,const void*p,int t){(void)q;(void)p;(void)t;return 1;}
static int xQueueReceive(QueueHandle_t q,void*p,int t);
static int link_send_frame_to(const uint8_t*m,int type,const uint8_t*p,size_t n,bool a);
'''

TEST = r'''
static packet_t input[8];static unsigned index_in, count_in, acknowledgements;
static uint8_t last_status;static uint32_t last_value;static jmp_buf done;
static int xQueueReceive(QueueHandle_t q,void*p,int t) {
    (void)q;(void)t;if(index_in==count_in)longjmp(done,1);
    memcpy(p,&input[index_in++],sizeof(packet_t));return 1;
}
static int link_send_frame_to(const uint8_t*m,int type,const uint8_t*p,size_t n,bool a) {
    (void)m;(void)type;(void)n;(void)a;++acknowledgements;last_status=p[3];last_value=value(p+8);return 0;
}
int main(void) {
    const uint8_t bytes[]="RIFFtest";
    for(unsigned bad=0;bad<2;bad++) {
        uint32_t sid=42+bad;
        memset(input,0,sizeof(input));
        for(unsigned i=0;i<5;i++){input[i].mac[0]=1;input[i].len=12;}
        header(input[0].wire,START,sid,sizeof(bytes));
        header(input[1].wire,DATA,sid,0);input[1].len=12+sizeof(bytes);
        memcpy(input[1].wire+12,bytes,sizeof(bytes));
        input[2]=input[1]; // duplicate chunk after lost ACK must not write twice
        header(input[3].wire,END,sid,hash_bytes(2166136261u,bytes,sizeof(bytes))+bad);
        input[4]=input[3]; // repeat END after lost final ACK
        index_in=0;count_in=bad?4:5;acknowledgements=0;
        if(!setjmp(done))receiver(NULL);
        assert(acknowledgements==count_in && last_status==bad);
        char name[128];snprintf(name,sizeof(name),"010000000000_%08x.wav",sid);
        FILE*f=fopen(name,"rb");
        if(bad){assert(!f);} else {
            uint8_t actual[sizeof(bytes)];assert(f && fread(actual,1,sizeof(actual),f)==sizeof(actual));
            assert(fgetc(f)==EOF && !memcmp(actual,bytes,sizeof(bytes)));fclose(f);
            assert(last_value==sizeof(bytes));
        }
    }
}
'''


class DirectMessageTest(unittest.TestCase):
    def test_duplicate_chunks_final_ack_and_checksum(self):
        source = (Path(__file__).resolve().parents[1] / 'message_direct.c').read_text()
        source = source[source.index('enum { START'):]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(STUBS + source + TEST)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra',
                            '-fsanitize=address,undefined', str(path / 'test.c'),
                            '-o', str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], cwd=tmp, check=True)


if __name__ == '__main__':
    unittest.main()
