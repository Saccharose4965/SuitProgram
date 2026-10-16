"""Differential tests of production uniform blending and RGB packing."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


def function(source, name):
    start = source.rfind("\nstatic ", 0, source.index(name + "(")) + 1
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class UniformRenderTests(unittest.TestCase):
    def test_exact_output_and_reused_work(self):
        source = (ROOT / "components/show/show_renderer.c").read_text()
        prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define SHOW_BLEND_REPLACE 0
#define SHOW_BLEND_ADD 1
#define SHOW_BLEND_MAX 2
typedef struct { float r,g,b,a; } show_color_t;
typedef struct { size_t led_count; show_color_t colors[128]; } show_renderer_t;
typedef struct { uint8_t blend_mode; unsigned mask; } show_clip_v1_t;
static bool clip_targets_led(const show_renderer_t *r, const show_clip_v1_t *c, size_t i)
{ (void)r; return (c->mask >> (i%8)) & 1; }
static void led_set_pixel_rgb(uint8_t *p,size_t i,uint8_t r,uint8_t g,uint8_t b)
{ p[3*i]=r; p[3*i+1]=g; p[3*i+2]=b; }
static unsigned blends, rounds;
static long counted_round(float x) { ++rounds; return lroundf(x); }
'''
        code = prefix + function(source, "clamp01")
        code += function(source, "blend_pixel").replace("r = clamp01(r);", "++blends; r = clamp01(r);")
        code += function(source, "render_uniform")
        code += function(source, "pack_colors").replace("lroundf(", "counted_round(")
        code += r'''
int main(void) {
    srand(17);
    for (unsigned test=0; test<2000; ++test) {
        show_renderer_t actual={.led_count=128}, expected=actual;
        show_clip_v1_t clip={test%4, test%3 ? (unsigned)rand()&255 : 255};
        float scale=test%7 ? (rand()%1025)/1024.0f : 0;
        float r=.73f,g=.27f,b=.95f,a=.63f;
        for (unsigned i=0;i<128;++i) {
            show_color_t c={(rand()%1025)/1024.0f,(rand()%1025)/1024.0f,
                            (rand()%1025)/1024.0f,(rand()%1025)/1024.0f};
            if (test%2 && i%16) c=actual.colors[i-1];
            actual.colors[i]=expected.colors[i]=c;
        }
        for (unsigned i=0;i<128;++i)
            if (clip_targets_led(&expected,&clip,i))
                blend_pixel(&expected.colors[i],clip.blend_mode,r*scale,g*scale,b*scale,a*scale);
        render_uniform(&actual,&clip,r,g,b,a,scale);
        assert(!memcmp(actual.colors,expected.colors,sizeof(actual.colors)));
        uint8_t got[384], want[384];
        uint8_t brightness=test%256;
        float factor=brightness/255.0f;
        for (unsigned i=0;i<128;++i) {
            show_color_t c=expected.colors[i];
            led_set_pixel_rgb(want,i,lroundf(clamp01(c.r)*255*factor),
                lroundf(clamp01(c.g)*255*factor),lroundf(clamp01(c.b)*255*factor));
        }
        pack_colors(actual.colors,128,brightness,got);
        assert(!memcmp(got,want,sizeof(got)));
    }
    for (unsigned mode=0;mode<4;++mode) {
        show_renderer_t uniform={.led_count=128};
        show_clip_v1_t clip={mode,255};
        uint8_t output[384];
        blends=rounds=0;
        render_uniform(&uniform,&clip,.8f,.5f,.1f,1,.7f);
        pack_colors(uniform.colors,128,255,output);
        assert(blends==1 && rounds==3);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "test.c").write_text(code)
            subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", str(path / "test.c"),
                            "-lm", "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
