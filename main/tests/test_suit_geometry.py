"""Canonical pose lookup and ring statistics, using production geometry."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class GeometryTest(unittest.TestCase):
    def test_shared_space(self):
        code = r'''
#include "suit_geometry.h"
#include <assert.h>
#include <math.h>
#include <string.h>
static led_point_t raw[6];
bool led_layout_get_from_config(const led_layout_config_t *c,int i,led_point_t *p) {
    if(i<0 || i>=c->total_leds)return false;
    *p=raw[i];return true;
}
int main(void) {
    led_layout_config_t c={.total_leds=6,.section_count=3};
    strcpy(c.sections[0].name,"front_left_ring");
    c.sections[0].length=4;c.sections[0].geom_kind=LED_LAYOUT_GEOM_ARC;
    strcpy(c.sections[1].name,"back_left_belt");
    c.sections[1].length=1;c.sections[1].logical_start=4;
    strcpy(c.sections[2].name,"left_upper_arm");
    c.sections[2].length=1;c.sections[2].logical_start=5;
    raw[0]=(led_point_t){4,10,0};raw[1]=(led_point_t){0,14,0};
    raw[2]=(led_point_t){-4,10,0};raw[3]=(led_point_t){0,6,0};
    raw[4]=(led_point_t){10,0,0};raw[5]=(led_point_t){0,0,0};
    led_point_t sum={0},p;
    for(int i=0;i<6;i++) {
        assert(suit_geometry_get(&c,i,&p));
        int section=i<4?0:i-3;
        show_section_id_t id;
        assert(show_section_id_from_name(c.sections[section].name,&id));
        led_point_t expected=show_geometry_pose(id,raw[i],i<4?i/3.0f:.5f);
        assert(fabsf(p.x-expected.x)<1e-6f && fabsf(p.y-expected.y)<1e-6f && fabsf(p.z-expected.z)<1e-6f);
        if(i<4){assert(p.z==12);sum.x+=p.x;sum.y+=p.y;sum.z+=p.z;}
        if(i==4)assert(p.z==-12 && p.y==33.5f);
    }
    led_point_t center;float radius;
    assert(suit_geometry_ring(&c,&center,&radius));
    assert(fabsf(center.x-sum.x/4)<1e-6f && center.y==10 && center.z==12 && radius>0);
    assert(!suit_geometry_get(&c,-1,&p));assert(!suit_geometry_get(&c,6,&p));
    assert(!suit_geometry_get(NULL,0,&p));assert(!suit_geometry_get(&c,0,NULL));
    raw[0].x=NAN;assert(!suit_geometry_get(&c,0,&p));
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-geometry-') as tmp:
            p=Path(tmp)
            (p/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n')
            (p/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined','-I',tmp,
                            '-I',str(ROOT/'services/suit_geometry/include'),
                            '-I',str(ROOT/'services/led_layout/include'),str(p/'test.c'),
                            *[str(ROOT/'services/suit_geometry'/f) for f in
                              ('suit_geometry.c','show_geometry.c','show_layout_ids.c')],
                            '-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)


if __name__=='__main__':unittest.main()
