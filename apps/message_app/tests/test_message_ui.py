"""Production message navigation/drawing against audio and display stubs."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class MessageUITest(unittest.TestCase):
    def test_recording_names(self):
        source=(ROOT/'apps/message_app/message_app.c').read_text()
        parse=source[source.index('static bool message_parse_outbox_number('):source.index('static esp_err_t message_build_outbox_path_from_name(')]
        build=source[source.index('static esp_err_t message_build_record_path('):source.index('static void message_capture_last_path(')]
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
typedef int esp_err_t;
enum {ESP_OK,ESP_ERR_INVALID_ARG,ESP_ERR_INVALID_STATE,ESP_ERR_NOT_FOUND,ESP_ERR_NO_MEM,ESP_FAIL};
static struct {bool sd_ready;char outbox_dir[128];}s_message={true,"."};
typedef struct {uint64_t latest_num;} message_outbox_scan_t;
static bool message_prepare_dirs(void){return true;}
static int message_scan_outbox(message_outbox_scan_t *s){s->latest_num=1;return ESP_OK;}
PARSE
BUILD
int main(void){
    uint64_t n;
    assert(message_parse_outbox_number("msg_123.wav",&n) && n==123);
    assert(!message_parse_outbox_number("msg18446744073709551616.wav",&n));
    assert(!message_parse_outbox_number("msg12.wav.part",&n));
    FILE *f=fopen("msg2.wav","wb");assert(f);fputs("unfinished",f);fclose(f);
    char path[128];assert(message_build_record_path(path,sizeof(path))==ESP_OK);
    assert(!strcmp(path,"./msg3.wav"));
    struct stat st;assert(stat("msg2.wav",&st)==0 && st.st_size==10);
    assert(remove("msg2.wav")==0);
    f=fopen("msg2.wav.part","wb");assert(f);fputs("partial",f);fclose(f);
    assert(message_build_record_path(path,sizeof(path))==ESP_OK);
    assert(!strcmp(path,"./msg3.wav"));
    assert(stat("msg2.wav.part",&st)==0 && st.st_size==7);
    assert(message_build_record_path(path,3)==ESP_ERR_NO_MEM);
}
'''.replace('PARSE',parse).replace('BUILD',build)
        with tempfile.TemporaryDirectory(prefix='suit-message-names-') as tmp:
            path=Path(tmp)
            (path/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined',str(path/'test.c'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],cwd=path,check=True)

    def test_controls_and_bounds(self):
        source = (ROOT/'apps/message_app/message_app.c').read_text()
        handlers = source[source.index('void message_app_handle_input('):]
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
enum { MSG_MENU,MSG_NEW,MSG_OUTBOX,MSG_INBOX,MSG_FILE_LIMIT=96 };
enum {INPUT_EVENT_PRESS,INPUT_EVENT_LONG_PRESS};
enum {INPUT_BTN_A,INPUT_BTN_B,INPUT_BTN_C,INPUT_BTN_D};
#define ESP_OK 0
#define PANEL_W 128
#define PANEL_H 64
static bool s_record_graph=true;
static size_t mic_rec_envelope(uint8_t *out,size_t n){memset(out,255,n);return n;}
typedef struct {int type,button;} input_event_t;
typedef struct {void(*request_switch)(const char *,void *);void *request_user_data;} shell_app_context_t;
static int s_page,s_choice,records,stopped,plays,sends,list_opens;
static size_t s_file_count,s_file_selected;
static bool s_list_full,recording;
static char s_files[96][64];
static struct {bool send_active;int last_err;int64_t rec_started_us;char last_path[128];} s_message;
static bool mic_rec_is_recording(void){return recording;}
static void message_new(void){s_page=MSG_NEW;}
static void message_open_list(int page){s_page=page;s_file_count=96;s_file_selected=0;list_opens++;}
static int message_start_recording(void){records++;recording=true;return 0;}
static int message_stop_recording(void){stopped++;recording=false;return 0;}
static int message_start_send(void){sends++;return 0;}
static int message_toggle_playback(void){plays++;return 0;}
static void message_play_selected(void){if(s_file_selected)plays++;}
static void message_update_legend(void){}
static int64_t esp_timer_get_time(void){return 1000000;}
static const char *basename_or_self(const char *s){return s;}
static const char *esp_err_to_name(int err){(void)err;return "ESP_ERR_INVALID_STATE";}
static int width,height;
static void oled_draw_text3x5(uint8_t *fb,int x,int y,const char *s){
    (void)fb;assert(x>=0 && y>=0 && x+(int)strlen(s)*4<=width && y+5<=height);
}
HANDLERS
static void press(int b){input_event_t e={INPUT_EVENT_PRESS,b};message_app_handle_input(NULL,&e);}
int main(void){
    press(INPUT_BTN_D);assert(s_page==MSG_NEW);
    press(INPUT_BTN_C);assert(records==1 && recording);
    press(INPUT_BTN_A);assert(!recording && stopped==1 && s_page==MSG_MENU);
    press(INPUT_BTN_B);press(INPUT_BTN_D);assert(s_page==MSG_OUTBOX && list_opens==1);
    press(INPUT_BTN_B);press(INPUT_BTN_C);assert(plays==1 && s_file_selected==1);
    press(INPUT_BTN_D);assert(s_page==MSG_NEW);
    press(INPUT_BTN_D);assert(plays==2);
    press(INPUT_BTN_B);assert(sends==1);
    s_message.send_active=true;press(INPUT_BTN_A);assert(s_page==MSG_NEW);
    s_message.send_active=false;press(INPUT_BTN_A);
    s_choice=2;press(INPUT_BTN_D);assert(s_page==MSG_INBOX);
    press(INPUT_BTN_B);press(INPUT_BTN_D);assert(plays==3 && s_page==MSG_INBOX);
    press(INPUT_BTN_A);press(INPUT_BTN_D);assert(s_page==MSG_MENU);
    input_event_t e={INPUT_EVENT_LONG_PRESS,INPUT_BTN_C};
    message_app_handle_input(NULL,&e);assert(records==1); // no hidden long-C action
    uint8_t fb[1024];
    memset(s_files,'x',sizeof(s_files));
    for(int i=0;i<96;i++)s_files[i][63]=0;
    int opens=list_opens;
    for(width=0;width<=128;width++)for(height=0;height<=64;height++){
        for(s_page=0;s_page<4;s_page++){
            s_file_selected=96;message_app_draw(NULL,fb,0,0,width,height);
            s_message.last_err=1;message_app_draw(NULL,fb,0,0,width,height);s_message.last_err=0;
        }
    }
    assert(opens==list_opens); // drawing cannot scan/open files
}
'''.replace('HANDLERS',handlers)
        with tempfile.TemporaryDirectory(prefix='suit-message-ui-') as tmp:
            path=Path(tmp)
            (path/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined',str(path/'test.c'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

if __name__=='__main__':
    unittest.main()
