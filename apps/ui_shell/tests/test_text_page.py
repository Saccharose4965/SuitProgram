"""The shared status page clips horizontally and respects its viewport height."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class TextPageTest(unittest.TestCase):
    def test_bounds(self):
        source = (ROOT / "apps/ui_shell/shell_ui.c").read_text()
        start = source.index("static void draw_clipped_text")
        end = source.index("void shell_ui_draw_overlay", start)
        implementation = source[start:end]
        harness = r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct { const char *title; const char *const *lines; size_t line_count; } shell_text_page_t;
static int calls, xs[8], ys[8]; static char shown[8][48];
static void oled_draw_text3x5(uint8_t *fb,int x,int y,const char *text) {
    (void)fb; xs[calls]=x;ys[calls]=y;snprintf(shown[calls],48,"%s",text);calls++;
}
'''
        test = r'''
int main(void) {
    uint8_t fb[1]={0}; const char *lines[]={"123456", "second"};
    shell_text_page_t page={.title="ABCDEFG",.lines=lines,.line_count=2};
    shell_ui_draw_text_page(fb,10,20,20,21,&page);
    assert(calls==2 && xs[0]==12 && ys[0]==22 && ys[1]==30);
    assert(!strcmp(shown[0],"ABCD") && !strcmp(shown[1],"1234"));
    calls=0; page.title=NULL;
    shell_ui_draw_text_page(fb,0,0,128,13,&page);
    assert(calls==1 && ys[0]==2 && !strcmp(shown[0],"123456"));
    calls=0; shell_ui_draw_text_page(fb,0,0,4,40,&page); assert(calls==0);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="suit-text-page-") as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(harness + implementation + test)
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", str(path / "test.c"),
                "-o", str(path / "test"),
            ], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
