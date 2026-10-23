"""Pixel equality against the pre-extraction implementation at checkpoint 224f2cd."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[3]

class DrawingTest(unittest.TestCase):
    def test_pixels(self):
        previous=subprocess.run(['git','show','224f2cd:components/oled/oled.c'],
                                cwd=ROOT,text=True,capture_output=True,check=True).stdout
        previous=previous[previous.index('// ---- Scroll curve'):]
        current=(ROOT/'apps/ui_shell/ui_draw.c').read_text()
        # Compile separate translation units to keep private glyph names private.
        prefix='''
#include <stdint.h>
#include <string.h>
#include <math.h>
#define PANEL_W 128
#define PANEL_H 64
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
void oled_blit_full(const uint8_t *p);
'''
        harness=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
static uint8_t actual[1024],expected[1024];
void oled_blit_full(const uint8_t *p){memcpy(actual,p,1024);}
void old_blit(const uint8_t *p){memcpy(expected,p,1024);}
void oled_draw_text3x5(uint8_t*,int,int,const char*);
void old_draw(uint8_t*,int,int,const char*);
void oled_render_three_lines(const char*,const char*,const char*);
void old_lines(const char*,const char*,const char*);
int oled_calc_scroll_px_slick(int,int);
int old_scroll(int,int);
int main(void){
    char text[2]={0};
    for(int c=1;c<128;c++)for(int x=-5;x<132;x+=17)for(int y=-5;y<69;y+=9){
        uint8_t a[1026]={0},b[1026]={0};text[0]=c;
        oled_draw_text3x5(a+1,x,y,text);old_draw(b+1,x,y,text);
        assert(memcmp(a,b,sizeof(a))==0);assert(a[0]==0 && a[1025]==0);
    }
    oled_draw_text3x5(0,0,0,"test");oled_draw_text3x5(actual,0,0,0);
    const char *lines[]={"", "Hello world", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"};
    for(int i=0;i<3;i++){
        oled_render_three_lines(lines[i],lines[(i+1)%3],lines[(i+2)%3]);
        old_lines(lines[i],lines[(i+1)%3],lines[(i+2)%3]);
        assert(memcmp(actual,expected,1024)==0);
    }
    for(int n=0;n<90;n++)for(int i=0;i<=n;i++)assert(oled_calc_scroll_px_slick(i,n)==old_scroll(i,n));
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-draw-') as tmp:
            p=Path(tmp)
            previous=previous.replace('oled_blit_full','old_blit').replace('oled_draw_text3x5','old_draw').replace('oled_render_three_lines','old_lines').replace('oled_calc_scroll_px_slick','old_scroll')
            (p/'old.c').write_text(prefix.replace('oled_blit_full','old_blit')+previous)
            (p/'new.c').write_text(prefix+'\n'.join(l for l in current.splitlines() if l!='#include "ui_draw.h"'))
            (p/'test.c').write_text(harness)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            str(p/'old.c'),str(p/'new.c'),str(p/'test.c'),'-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
