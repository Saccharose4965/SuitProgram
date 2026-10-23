#include "suit_geometry.h"
#include <math.h>
#include <string.h>

bool suit_geometry_get(const led_layout_config_t *cfg, int index, led_point_t *out)
{
    if (!cfg || !out || index < 0 || index >= cfg->total_leds) return false;
    for (size_t s=0; s<cfg->section_count; ++s) {
        const led_layout_section_t *section=&cfg->sections[s];
        if (index < section->logical_start ||
            (size_t)(index-section->logical_start) >= section->length) continue;
        led_point_t raw;
        if (!led_layout_get_from_config(cfg,index,&raw)) return false;
        if (!isfinite(raw.x) || !isfinite(raw.y) || !isfinite(raw.z)) return false;
        show_section_id_t id=SHOW_SECTION_INVALID;
        (void)show_section_id_from_name(section->name,&id);
        float u=section->length>1 ? (float)(index-section->logical_start)/(section->length-1) : .5f;
        *out=show_geometry_pose(id,raw,u);
        return isfinite(out->x) && isfinite(out->y) && isfinite(out->z);
    }
    return false;
}

bool suit_geometry_ring(const led_layout_config_t *cfg, led_point_t *center, float *radius)
{
    if (!cfg || !center || !radius) return false;
    for (size_t s=0;s<cfg->section_count;++s) {
        const led_layout_section_t *sec=&cfg->sections[s];
        if (sec->geom_kind!=LED_LAYOUT_GEOM_ARC || !strstr(sec->name,"ring")) continue;
        led_point_t sum={0},p; size_t count=0;
        for (size_t j=0;j<sec->length;++j)
            if(suit_geometry_get(cfg,sec->logical_start+j,&p)) {
                sum.x+=p.x;sum.y+=p.y;sum.z+=p.z;++count;
            }
        if(!count) continue;
        *center=(led_point_t){sum.x/count,sum.y/count,sum.z/count};
        float total=0;
        for(size_t j=0;j<sec->length;++j)
            if(suit_geometry_get(cfg,sec->logical_start+j,&p))
                total+=hypotf(p.x-center->x,p.y-center->y);
        *radius=total/count;
        return true;
    }
    return false;
}
