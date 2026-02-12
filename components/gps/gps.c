// components/gps/gps.c
#include "gps.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#define GPS_UART     UART_NUM_0
#define GPS_TX_GPIO  1
#define GPS_RX_GPIO  3

#include "oled.h"  // for oled_render_three_lines()
#include "system_state.h"

#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#define BG_TASK_CORE 0
#else
#define APP_TASK_CORE 0
#define BG_TASK_CORE 0
#endif

// ---------------- shared state ----------------
static gps_fix_t s_fix = {0};
static SemaphoreHandle_t s_fix_mux = NULL;

static inline void fix_lock(void){ if (s_fix_mux) xSemaphoreTake(s_fix_mux, portMAX_DELAY); }
static inline void fix_unlock(void){ if (s_fix_mux) xSemaphoreGive(s_fix_mux); }
static inline gps_fix_t fix_copy(void){ gps_fix_t f; fix_lock(); f = s_fix; fix_unlock(); return f; }
static inline void fix_commit(const gps_fix_t* f){ fix_lock(); s_fix = *f; fix_unlock(); }

// -------------- helpers --------------
static uint8_t hex2nib(char c){
    if (c>='0' && c<='9') return (uint8_t)(c-'0');
    c = (char)toupper((unsigned char)c);
    if (c>='A' && c<='F') return (uint8_t)(10 + c - 'A');
    return 0xFF;
}
static int nmea_checksum_ok(const char* s){
    if (!s || *s!='$') return 0;
    const char *p = s+1; uint8_t cs = 0;
    while (*p && *p!='*' && *p!='\r' && *p!='\n') cs ^= (uint8_t)*p++;
    if (*p!='*' || !p[1] || !p[2]) return 0;
    uint8_t hi=hex2nib(p[1]), lo=hex2nib(p[2]);
    if (hi>0x0F || lo>0x0F) return 0;
    return cs == (uint8_t)((hi<<4)|lo);
}
// fld: "ddmm.mmmm" (lat) or "dddmm.mmmm" (lon)
static double nmea_coord_to_deg(const char* fld, char hemi){
    if (!fld || !*fld) return NAN;
    double v = atof(fld);
    int deg = (int)(v/100.0);
    double minutes = v - (deg*100.0);
    double dec = deg + minutes/60.0;
    if (hemi=='S' || hemi=='W') dec = -dec;
    return dec;
}
// try parse float; keep previous on empty
static inline void parse_float_opt(const char* tok, float* out){
    if (tok && *tok) *out = (float)atof(tok);
}
static inline void parse_int_opt(const char* tok, int* out){
    if (tok && *tok) *out = atoi(tok);
}
static bool parse_hms(const char *tok, int *h, int *m, int *s){
    if (!tok || strlen(tok) < 6) return false;
    for (int i = 0; i < 6; ++i){
        if (!isdigit((unsigned char)tok[i])) return false;
    }
    int hh = (tok[0]-'0')*10 + (tok[1]-'0');
    int mm = (tok[2]-'0')*10 + (tok[3]-'0');
    int ss = (tok[4]-'0')*10 + (tok[5]-'0');
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 60) return false;
    if (h) *h = hh;
    if (m) *m = mm;
    if (s) *s = ss;
    return true;
}
static bool parse_date_ddmmyy(const char *tok, int *y, int *m, int *d){
    if (!tok || strlen(tok) < 6) return false;
    for (int i = 0; i < 6; ++i){
        if (!isdigit((unsigned char)tok[i])) return false;
    }
    int dd = (tok[0]-'0')*10 + (tok[1]-'0');
    int mm = (tok[2]-'0')*10 + (tok[3]-'0');
    int yy = (tok[4]-'0')*10 + (tok[5]-'0');
    if (dd <= 0 || dd > 31 || mm <= 0 || mm > 12) return false;
    int full_year = 2000 + yy; // NMEA gives two-digit year
    if (y) *y = full_year;
    if (m) *m = mm;
    if (d) *d = dd;
    return true;
}

// -------------- sentence parsers --------------
// $--GGA, time, lat, N/S, lon, E/W, fixq, sats, hdop, alt, M, ...
static void parse_gga(char *line){
    char *p=line, *tok; int fld=0;
    gps_fix_t f = fix_copy();

    const char *lat_str=NULL, *lon_str=NULL;
    char ns='N', ew='E';
    int fixq=0, sats=f.sats_used;
    float hdop=f.hdop, altm=f.altitude_m;
    int hh=-1, mm=-1, ss=-1; bool have_time=false;

    while ((tok=strsep(&p, ","))!=NULL){
        fld++;
        switch(fld){
            case 2: if (parse_hms(tok, &hh, &mm, &ss)) have_time = true; break;
            case 3: lat_str = tok; break;
            case 4: if (*tok) ns = tok[0]; break;
            case 5: lon_str = tok; break;
            case 6: if (*tok) ew = tok[0]; break;
            case 7: parse_int_opt(tok, &fixq); break;          // fix quality
            case 8: parse_int_opt(tok, &sats); break;          // satellites USED
            case 9: parse_float_opt(tok, &hdop); break;        // HDOP
            case 10: parse_float_opt(tok, &altm); break;       // altitude (m)
            default: break;
        }
    }
    if (lat_str && *lat_str){ double la=nmea_coord_to_deg(lat_str, ns); if (!isnan(la)) f.lat_deg=la; }
    if (lon_str && *lon_str){ double lo=nmea_coord_to_deg(lon_str, ew); if (!isnan(lo)) f.lon_deg=lo; }

    f.sats_used  = sats;
    f.hdop       = hdop;
    f.altitude_m = altm;
    if (fixq > 0) f.valid = true; // 1=GPS, 2=DGPS, etc.
    if (have_time){
        f.hour = hh; f.min = mm; f.sec = ss;
        f.time_valid = true;
    }

    fix_commit(&f);
}

// $--RMC, time, status(A/V), lat, N/S, lon, E/W, speed(kn), course, date, ...
static void parse_rmc(char *line){
    char *p=line, *tok; int fld=0;
    gps_fix_t f = fix_copy();

    const char *lat_str=NULL, *lon_str=NULL;
    char ns='N', ew='E';
    char status='V';
    float spd=f.speed_kmh, crs=f.course_deg;
    int hh=-1, mm=-1, ss=-1; bool have_time=false;
    int dd=-1, mo=-1, yy=-1; bool have_date=false;

    while ((tok=strsep(&p, ","))!=NULL){
        fld++;
        switch(fld){
            case 2:  if (parse_hms(tok, &hh, &mm, &ss)) have_time = true; break; // time (UTC)
            case 3:  if (*tok){ status = tok[0]; if (status=='a') status='A'; } break;         // status A/V
            case 4:  lat_str = tok; break;
            case 5:  if (*tok) ns = tok[0]; break;
            case 6:  lon_str = tok; break;
            case 7:  if (*tok) ew = tok[0]; break;
            case 8:  if (*tok) spd = (float)(atof(tok)*1.852); break; // knots→km/h
            case 9:  if (*tok) { crs=(float)atof(tok); if (!isnan(crs)) crs=fmodf(crs+360.0f,360.0f);} break;
            case 10: if (parse_date_ddmmyy(tok, &yy, &mo, &dd)) have_date = true; break;
            default: break;
        }
    }
    if (status=='A') f.valid = true;
    if (lat_str && *lat_str){ double la=nmea_coord_to_deg(lat_str, ns); if (!isnan(la)) f.lat_deg=la; }
    if (lon_str && *lon_str){ double lo=nmea_coord_to_deg(lon_str, ew); if (!isnan(lo)) f.lon_deg=lo; }

    f.speed_kmh  = spd;
    f.course_deg = crs;
    if (have_time){
        f.hour = hh; f.min = mm; f.sec = ss;
        f.time_valid = true;
    }
    if (have_date){
        f.year = yy; f.month = mo; f.day = dd;
        f.date_valid = true;
    }

    fix_commit(&f);
}

// $--GLL, lat, N/S, lon, E/W, time, status(A/V), mode
static void parse_gll(char *line){
    char *p=line, *tok; int fld=0;
    gps_fix_t f = fix_copy();
    const char *lat_str=NULL,*lon_str=NULL; char ns='N', ew='E';
    char status='V';
    int hh=-1, mm=-1, ss=-1; bool have_time=false;

    while ((tok=strsep(&p, ","))!=NULL){
        fld++;
        switch(fld){
            case 2: lat_str = tok; break;
            case 3: if (*tok) ns = tok[0]; break;
            case 4: lon_str = tok; break;
            case 5: if (*tok) ew = tok[0]; break;
            case 6: if (parse_hms(tok, &hh, &mm, &ss)) have_time = true; break;
            case 7: if (*tok=='A'||*tok=='a'||*tok=='V'||*tok=='v') status = (tok[0]=='A'||tok[0]=='a')?'A':'V'; break;
            case 8: if (*tok=='A'||*tok=='a'||*tok=='V'||*tok=='v') status = (tok[0]=='A'||tok[0]=='a')?'A':'V'; break;
            default: break;
        }
    }
    if (status=='A') f.valid = true;
    if (lat_str && *lat_str){ double la=nmea_coord_to_deg(lat_str, ns); if (!isnan(la)) f.lat_deg=la; }
    if (lon_str && *lon_str){ double lo=nmea_coord_to_deg(lon_str, ew); if (!isnan(lo)) f.lon_deg=lo; }
    if (have_time){
        f.hour = hh; f.min = mm; f.sec = ss;
        f.time_valid = true;
    }
    fix_commit(&f);
}

// $--GSA, M, fix_type(1/2/3), s1..s12, PDOP, HDOP, VDOP
static void parse_gsa(char *line){
    char *p=line, *tok; int fld=0;
    gps_fix_t f = fix_copy();
    int ftype=f.fix_type;
    float pd=f.pdop, hd=f.hdop, vd=f.vdop;
    // fallback: capture last three numeric fields seen
    float tail[3]={NAN,NAN,NAN}; int tlen=0;

    while ((tok=strsep(&p, ","))!=NULL){
        fld++;
        if (fld==3 && *tok) ftype = atoi(tok);
        // capture any numeric
        if (*tok && (isdigit((unsigned char)tok[0]) || tok[0]=='.')){
            if (tlen<3) tail[tlen++]=(float)atof(tok);
            else { tail[0]=tail[1]; tail[1]=tail[2]; tail[2]=(float)atof(tok); }
        }
        if (fld==16 && *tok) pd=(float)atof(tok);
        if (fld==17 && *tok) hd=(float)atof(tok);
        if (fld==18 && *tok) vd=(float)atof(tok);
    }
    // fallback if canonical positions missing
    if (isnan(pd) && tlen==3){ pd=tail[0]; hd=tail[1]; vd=tail[2]; }

    f.fix_type = ftype;
    f.pdop = pd; f.hdop = hd; f.vdop = vd;
    if (ftype >= 2) f.valid = true; // 2D/3D ⇒ valid

    fix_commit(&f);
}

// $--GSV, total_msgs, msg_idx, total_sats, ...
static void parse_gsv(char *line){
    char *p = line, *tok;
    int fld = 0;
    gps_fix_t f = fix_copy();
    int total = f.sats_view;  // keep current value
    while ((tok = strsep(&p, ",")) != NULL){
        fld++;
        if (fld == 4 && *tok) {
            int t = atoi(tok);
            if (t > total) total = t;  // track highest seen
        }
    }
    f.sats_view = total;
    fix_commit(&f);
}


// dispatch
static void parse_nmea_line(const char *line_in){
    if (!nmea_checksum_ok(line_in)) return;
    char *line = strdup(line_in); if (!line) return;

    // Identify after talker prefix ($GN/$GP/$BD/...)
    if      (strstr(line, "GGA,")) parse_gga(line);
    else if (strstr(line, "RMC,")) parse_rmc(line);
    else if (strstr(line, "GLL,")) parse_gll(line);
    else if (strstr(line, "GSA,")) parse_gsa(line);
    else if (strstr(line, "GSV,")) parse_gsv(line);
    // ZDA/VTG/TXT ignored for now

    free(line);
}

// ---------------- task & API ----------------
static void gps_task(void *arg){
    uint8_t buf[256]; char line[160]; size_t ln=0;

    // Hand UART0 fully to GPS (stop console printing)
    esp_log_set_vprintf(NULL);

    while (1){
        int n = uart_read_bytes(GPS_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        for (int i=0;i<n;i++){
            char c=(char)buf[i];
            if (c=='\r') continue;
            if (c=='\n'){
                line[ln]=0;
                if (ln>6 && line[0]=='$') parse_nmea_line(line);
                ln=0;
            } else if (ln < sizeof(line)-1){
                line[ln++]=c;
            } else {
                ln=0; // overflow, drop
            }
        }
    }
}

void gps_start(int baud){
    if (!s_fix_mux) s_fix_mux = xSemaphoreCreateMutex();

    // reset state
    gps_fix_t z = {0};
    z.lat_deg = z.lon_deg = NAN;
    z.hdop = z.pdop = z.vdop = NAN;
    z.hour = z.min = z.sec = -1;
    z.year = z.month = z.day = -1;
    fix_commit(&z);

    // UART0 → GPS
    uart_flush(GPS_UART);
    uart_driver_delete(GPS_UART);

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_GPIO, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, BG_TASK_CORE);
}

gps_fix_t gps_get_fix(void){
    gps_fix_t out;
    if (!s_fix_mux){
        memset(&out,0,sizeof(out));
        out.lat_deg = out.lon_deg = NAN;
        out.hdop = out.pdop = out.vdop = NAN;
        out.hour = out.min = out.sec = -1;
        out.year = out.month = out.day = -1;
        return out;
    }
    fix_lock(); out = s_fix; fix_unlock();
    return out;
}

// ===== GPS service (moved from app_main.c) =====
static gps_fix_t s_cached;
static SemaphoreHandle_t s_cached_mux;
static TaskHandle_t s_time_bridge_task = NULL;

static inline bool _sane_lat(double d){ return !isnan(d) && fabs(d) <= 90.0 && fabs(d) > 1e-5; }
static inline bool _sane_lon(double d){ return !isnan(d) && fabs(d) <=180.0 && fabs(d) > 1e-5; }

bool gps_is_good_fix(const gps_fix_t *f){
    if (f->fix_type >= 3) return true;                       // 3D fix
    if (f->hdop > 0 && f->hdop <= 2.5f) return true;         // good geometry
    if (f->valid && f->sats_used >= 6 && f->hdop > 0 && f->hdop <= 5.0f) return true;
    return false;
}

static void gps_service_task(void *arg){
    (void)arg;
    for(;;){
        gps_fix_t f = gps_get_fix();

        const bool fresh_lat = _sane_lat(f.lat_deg);
        const bool fresh_lon = _sane_lon(f.lon_deg);
        const bool good = gps_is_good_fix(&f);

        xSemaphoreTake(s_cached_mux, portMAX_DELAY);
        bool have_lat = isfinite(s_cached.lat_deg);
        bool have_lon = isfinite(s_cached.lon_deg);

        if (!have_lat && fresh_lat) s_cached.lat_deg = f.lat_deg;
        if (!have_lon && fresh_lon) s_cached.lon_deg = f.lon_deg;
        if (good && fresh_lat && fresh_lon){ s_cached.lat_deg=f.lat_deg; s_cached.lon_deg=f.lon_deg; }

        if (f.time_valid){
            s_cached.hour = f.hour; s_cached.min = f.min; s_cached.sec = f.sec;
            s_cached.time_valid = true;
        }
        if (f.date_valid){
            s_cached.year = f.year; s_cached.month = f.month; s_cached.day = f.day;
            s_cached.date_valid = true;
        }

        // sats: prefer in-view if provided, else used
        s_cached.sats_view = (f.sats_view>0)?f.sats_view:((f.sats_used>0)?f.sats_used:0);
        s_cached.valid = good || fresh_lat || fresh_lon;
        s_cached.hdop = f.hdop; s_cached.pdop = f.pdop; s_cached.vdop = f.vdop;
        s_cached.fix_type = f.fix_type;
        s_cached.altitude_m = f.altitude_m;
        s_cached.speed_kmh  = f.speed_kmh;
        s_cached.course_deg = f.course_deg;
        xSemaphoreGive(s_cached_mux);

        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 Hz
    }
}

void gps_service_start(int baud){
    static bool started=false;
    if (started) return;

    // Ensure base GPS task is running
    gps_start(baud);

    // Init cached state
    if (!s_cached_mux) s_cached_mux = xSemaphoreCreateMutex();
    xSemaphoreTake(s_cached_mux, portMAX_DELAY);
    memset(&s_cached, 0, sizeof(s_cached));
    s_cached.lat_deg = s_cached.lon_deg = NAN;
    s_cached.hdop = s_cached.pdop = s_cached.vdop = NAN;
    s_cached.hour = s_cached.min = s_cached.sec = -1;
    s_cached.year = s_cached.month = s_cached.day = -1;
    xSemaphoreGive(s_cached_mux);

    xTaskCreatePinnedToCore(gps_service_task, "gps_svc", 4096, NULL, 4, NULL, BG_TASK_CORE);
    started = true;
}

void gps_services_start(int baud)
{
    gps_service_start(baud);
    gps_system_time_bridge_start();
}

gps_fix_t gps_cached_fix(void){
    gps_fix_t out;
    if (!s_cached_mux){
        memset(&out,0,sizeof(out));
        out.lat_deg = out.lon_deg = NAN;
        out.hdop = out.pdop = out.vdop = NAN;
        out.hour = out.min = out.sec = -1;
        out.year = out.month = out.day = -1;
        return out;
    }
    xSemaphoreTake(s_cached_mux, portMAX_DELAY);
    out = s_cached;
    xSemaphoreGive(s_cached_mux);
    return out;
}

static void gps_system_time_bridge_task(void *arg)
{
    (void)arg;
    const TickType_t stale_window = pdMS_TO_TICKS(30000); // mark invalid if >30 s old
    TickType_t last_valid = 0;

    for (;;) {
        gps_fix_t f = gps_cached_fix();
        TickType_t now = xTaskGetTickCount();

        bool have_time = f.time_valid && f.hour >= 0 && f.hour < 24 && f.min >= 0 && f.min < 60;
        if (have_time) {
            system_state_set_time(true, (uint8_t)f.hour, (uint8_t)f.min);
            last_valid = now;
        } else if (last_valid != 0 && (now - last_valid) > stale_window) {
            system_state_set_time(false, 0, 0);
            last_valid = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void gps_system_time_bridge_start(void)
{
    if (s_time_bridge_task) return;

    BaseType_t ok = xTaskCreatePinnedToCore(
        gps_system_time_bridge_task,
        "gps_time",
        2048,
        NULL,
        4,
        &s_time_bridge_task,
        BG_TASK_CORE
    );
    if (ok != pdPASS) {
        s_time_bridge_task = NULL;
    }
}

static inline void fb_clear_rect(uint8_t *fb, int x0, int y0, int w, int h){
    if (!fb || w <= 0 || h <= 0) return;
    for (int y = y0; y < y0 + h; ++y){
        if ((unsigned)y >= PANEL_H) continue;
        for (int x = x0; x < x0 + w; ++x){
            if ((unsigned)x >= PANEL_W) continue;
            int idx = y * PANEL_W + x;
            fb[idx >> 3] &= (uint8_t)~(1u << (7 - (idx & 7)));
        }
    }
}

void gps_display_time(uint8_t *fb){
    if (!fb) return;
    gps_fix_t f = gps_cached_fix();

    bool have_time = f.time_valid && f.hour >= 0 && f.hour < 24 && f.min >= 0 && f.min < 60;
    char buf[12];
    if (have_time) snprintf(buf, sizeof buf, "%02d:%02d", f.hour, f.min);
    else snprintf(buf, sizeof buf, "--:--");

    int len = (int)strlen(buf);
    int width = (len > 0) ? (len * 4 - 1) : 0; // 3x5 font → 3px + 1px spacing
    int x = PANEL_W - width - 2; if (x < 0) x = 0;
    int y = 0;
    fb_clear_rect(fb, (x>0)?(x-1):0, y, width + 2, 6); // 5px font height + padding
    oled_draw_text3x5(fb, x, y, buf);
}

static void gps_ui_task(void *arg){
    char lineSats[32], lineLat[32], lineLon[32];
    for(;;){
        gps_fix_t f = gps_cached_fix();

        snprintf(lineSats, sizeof lineSats, "SATS: %d",
                 (f.sats_view > 0) ? f.sats_view : ((f.sats_used > 0) ? f.sats_used : 0));

        if (isfinite(f.lat_deg)) {
            double v = fabs(f.lat_deg); char hemi = (f.lat_deg >= 0) ? 'N' : 'S';
            snprintf(lineLat, sizeof lineLat, "LAT: %.5f%c", v, hemi);
        } else lineLat[0] = 0;

        if (isfinite(f.lon_deg)) {
            double v = fabs(f.lon_deg); char hemi = (f.lon_deg >= 0) ? 'E' : 'W';
            snprintf(lineLon, sizeof lineLon, "LON: %.5f%c", v, hemi);
        } else lineLon[0] = 0;

        oled_render_three_lines(lineSats, lineLat, lineLon);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void gps_ui_start(void){
    xTaskCreatePinnedToCore(gps_ui_task, "gps_ui", 4096, NULL, 4, NULL, APP_TASK_CORE);
}
