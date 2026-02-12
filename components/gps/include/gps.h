#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool   valid;
    double lat_deg, lon_deg;

    int    sats_used, sats_view;
    float  hdop, pdop, vdop;
    int    fix_type;

    float  altitude_m;
    float  speed_kmh;
    float  course_deg;

    // UTC date/time from NMEA (if provided)
    bool   time_valid;
    bool   date_valid;
    int    year, month, day;   // year is full (e.g., 2025)
    int    hour, min, sec;
} gps_fix_t;

// Service API (moved out of app_main.c)
void gps_service_start(int baud);
void gps_services_start(int baud);
bool gps_is_good_fix(const gps_fix_t *f);
gps_fix_t gps_cached_fix(void);
void gps_system_time_bridge_start(void);
void gps_ui_start(void);
void gps_display_time(uint8_t *fb);
#ifdef __cplusplus
}
#endif
