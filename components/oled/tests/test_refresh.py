"""Production OLED blitter against an emulated page-addressed panel."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class RefreshTest(unittest.TestCase):
    def test_full_and_partial(self):
        source = (ROOT / 'components/oled/oled.c').read_text()
        function = source[source.index('void oled_blit_full('):source.index('// ---- 64x64 centered')]
        code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define PANEL_W 128
#define PANEL_H 64
static bool s_oled_prev_valid;
static uint8_t s_oled_prev_pages[8][128], panel[8][128];
static int x0,x1,p0,p1,x,p,windows,bytes;
static void begin_window(int a,int b,int c,int d){
    x0=a;x1=b;p0=c;p1=d;x=a;p=c;windows++;
}
static void tx(const void *data,size_t len,bool is_data){
    assert(is_data);const uint8_t *src=data;bytes+=len;
    while(len--){panel[p][x]=*src++;if(++x>x1){x=x0;if(++p>p1)p=p0;}}
}
FUNCTION
static void verify(const uint8_t *fb){
    for(int y=0;y<64;y++)for(int x=0;x<128;x++){
        int bit=y*128+x;
        assert(((panel[y/8][x]>>(y%8))&1)==((fb[bit/8]>>(7-bit%8))&1));
    }
}
int main(void){
    uint8_t fb[1024]={0};
    for(int frame=0;frame<30;frame++){
        fb[(frame*37)%1024]^=0x85;
        bytes=windows=0;oled_blit_full(fb);verify(fb);
#ifndef CONFIG_SUIT_OLED_PARTIAL_UPDATES
        assert(bytes==1024 && windows==1);
#endif
    }
    bytes=windows=0;oled_blit_full(fb);verify(fb);
#ifdef CONFIG_SUIT_OLED_PARTIAL_UPDATES
    assert(bytes==0 && windows==0);
#else
    assert(bytes==1024 && windows==1);
    memset(panel,0xff,sizeof(panel)); // full refresh repairs stale panel contents
    oled_blit_full(fb);verify(fb);
#endif
    memset(fb,0,sizeof(fb));oled_blit_full(fb);verify(fb);
    oled_blit_full(NULL);
}
'''.replace('FUNCTION', function)
        with tempfile.TemporaryDirectory(prefix='suit-oled-') as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(code)
            for partial in (False, True):
                flags = ['-DCONFIG_SUIT_OLED_PARTIAL_UPDATES=1'] if partial else []
                subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                '-fsanitize=address,undefined', *flags,
                                str(path / 'test.c'), '-o', str(path / 'test')], check=True)
                subprocess.run([str(path / 'test')], check=True)

if __name__ == '__main__':
    unittest.main()
