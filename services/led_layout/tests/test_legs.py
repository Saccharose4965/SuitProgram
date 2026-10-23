"""Compile the real layout loader; zero legs must not move existing pixels."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class LegsTest(unittest.TestCase):
    def test_defaults_migration_and_roundtrip(self):
        headers = {
            'esp_err.h': '#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_FAIL -1\n#define ESP_ERR_INVALID_STATE 1\n#define ESP_ERR_INVALID_ARG 2\n#define ESP_ERR_NOT_FOUND 3\n',
            'esp_attr.h': '#define EXT_RAM_BSS_ATTR\n',
            'esp_log.h': '#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGI(...) ((void)0)\n',
            'esp_check.h': '#define ESP_RETURN_ON_ERROR(e,...) do {int r=(e); if(r) return r;} while(0)\n',
            'freertos/FreeRTOS.h': '#pragma once\n#define portMAX_DELAY 0\n#define pdTRUE 1\n',
            'freertos/semphr.h': '#pragma once\ntypedef int StaticSemaphore_t; typedef int* SemaphoreHandle_t;\nstatic inline int* xSemaphoreCreateMutexStatic(int*p){return p;}\nstatic inline int xSemaphoreTake(int*p,int t){(void)p;(void)t;return 1;}\nstatic inline void xSemaphoreGive(int*p){(void)p;}\n',
            'storage_sd.h': '#include <stdio.h>\nstatic inline int storage_mount_sd(void){return 0;}\nstatic inline int storage_sd_make_path(char*p,size_t n,const char*f){snprintf(p,n,"%s",f);return 0;}\n',
            'sys/unistd.h': '#include <unistd.h>\n',
        }
        code = r'''
#include <assert.h>
#include "SOURCE"
static led_layout_config_t old, now, saved;
int main(void) {
    layout_fill_default(&now); layout_finalize(&now);
    assert(now.section_count == 33 && now.total_leds == 721);
    FILE *bad=fopen("led_layout.txt","w"); assert(bad);
    fputs("strip_count=256\nsection0_length=65536\nsection0_point_count=256\nsection0_radius=nan\n",bad);
    fclose(bad);
    assert(layout_load_from_sd_locked(&saved));
    assert(saved.strip_count==2 && saved.sections[0].length==27);
    assert(saved.sections[0].point_count<=LED_LAYOUT_MAX_SECTION_POINTS);
    assert(isfinite(saved.sections[0].radius));
    unlink("led_layout.txt");
    for (unsigned i=0;i<now.section_count;i++) {
        const led_layout_section_t *s=&now.sections[i];
        if (strstr(s->name,"thigh") || strstr(s->name,"shin")) {
            assert(!s->length && s->point_count >= 4 && s->points[0].y > 30);
        } else old.sections[old.section_count++]=*s;
    }
    old.strip_count=2; layout_finalize(&old);
    assert(old.section_count==21);
    assert(layout_save_to_sd_locked(&old)==ESP_OK);
    assert(layout_load_from_sd_locked(&now));
    assert(now.section_count==33 && now.total_leds==old.total_leds);
    for (unsigned i=0;i<old.section_count;i++) {
        const led_layout_section_t *a=&old.sections[i], *b=layout_find_section(&now,a->name);
        assert(b && a->length==b->length && a->physical_start==b->physical_start && a->reversed==b->reversed);
    }
    for (unsigned i=0;i<old.total_leds;i++) {
        led_layout_mapping_t a,b;
        assert(led_layout_map_logical_from_config(&old,i,&a));
        assert(led_layout_map_logical_from_config(&now,i,&b));
        assert(a.strip==b.strip && a.physical_index==b.physical_index);
    }
    assert(layout_save_to_sd_locked(&now)==ESP_OK);
    assert(layout_load_from_sd_locked(&saved));
    assert(saved.section_count==33 && saved.total_leds==now.total_leds);
    for (unsigned i=0;i<33;i++) {
        assert(!strcmp(now.sections[i].name,saved.sections[i].name));
        assert(now.sections[i].length==saved.sections[i].length);
        for(unsigned p=0;p<now.sections[i].point_count;p++) {
            assert(fabsf(now.sections[i].points[p].x-saved.sections[i].points[p].x)<1e-4f);
            assert(fabsf(now.sections[i].points[p].y-saved.sections[i].points[p].y)<1e-4f);
            assert(fabsf(now.sections[i].points[p].z-saved.sections[i].points[p].z)<1e-4f);
        }
    }
    // Allow a future leg to consume spare capacity, then disable it again.
    s_layout=now; s_ready=true;
    unsigned leg=5; assert(!strcmp(s_layout.sections[leg].name,"left_thigh_front"));
    assert(led_layout_set_section_length(0,17)==ESP_OK);
    unsigned back=layout_find_section(&s_layout,"back_left_belt")->physical_start;
    assert(led_layout_set_section_length(leg,10)==ESP_OK);
    assert(layout_find_section(&s_layout,"back_left_belt")->physical_start==back+10);
    assert(led_layout_set_section_length(leg,0)==ESP_OK);
    assert(layout_find_section(&s_layout,"back_left_belt")->physical_start==back);
}
'''.replace('SOURCE', str(ROOT / 'services/led_layout/led_layout.c'))
        with tempfile.TemporaryDirectory(prefix='suit-legs-') as tmp:
            p=Path(tmp)
            for name, text in headers.items():
                path=p/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_text(text)
            (p/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-D_DEFAULT_SOURCE','-Wall','-Wextra',
                '-Wno-unused-variable','-fsanitize=address,undefined','-I',tmp,
                '-I',str(ROOT/'services/led_layout/include'),str(p/'test.c'),'-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],cwd=tmp,check=True)

if __name__=='__main__': unittest.main()
