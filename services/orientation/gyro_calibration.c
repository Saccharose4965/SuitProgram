#include "gyro_calibration.h"
#include "orientation_service.h"
#include "storage_sd.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

static gyro_calibration_status_t s;
static int64_t start,last;
static unsigned count;
static float mean[3],m2[3],accel0[3];
static bool valid(const float b[3])
{
    for(int i=0;i<3;i++) if(!isfinite(b[i]) || fabsf(b[i])>10) return false;
    return true;
}
void gyro_calibration_cancel(void) { s.state=GYRO_IDLE; }
void gyro_calibration_begin(void)
{
    s=(gyro_calibration_status_t){.state=GYRO_SETTLING};
    start=esp_timer_get_time();last=0;count=0;
    for(int i=0;i<3;i++) {mean[i]=0;m2[i]=0;}
}
gyro_calibration_status_t gyro_calibration_status(void) { return s; }
void gyro_calibration_tick(void)
{
    int64_t now=esp_timer_get_time();
    if(s.state==GYRO_SETTLING) {
        if(now-start<2000000) return;
        s.state=GYRO_SAMPLING;start=now;
    }
    if(s.state!=GYRO_SAMPLING) return;
    mpu6500_sample_t raw;
    if(!orientation_service_read_sample(&raw) || raw.t_us<=0 || now-raw.t_us>250000 || raw.t_us>now) {
        s.state=GYRO_NO_DATA;return;
    }
    if(raw.t_us!=last) {
        last=raw.t_us;
        float a[3]={raw.ax_g,raw.ay_g,raw.az_g};
        float b[3]={raw.gx_dps,raw.gy_dps,raw.gz_dps};
        float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        if(!valid(b) || !isfinite(norm) || norm<.9f || norm>1.1f) {s.state=GYRO_MOVED;return;}
        for(int i=0;i<3;i++) {
            if(!count) accel0[i]=a[i];
            if(fabsf(a[i]-accel0[i])>.08f) {s.state=GYRO_MOVED;return;}
        }
        ++count;
        for(int i=0;i<3;i++) {
            float d=b[i]-mean[i];mean[i]+=d/count;m2[i]+=d*(b[i]-mean[i]);
        }
    }
    s.progress=fminf(1,(now-start)/5000000.f);
    if(now-start<5000000) return;
    if(count<100) {s.state=GYRO_NO_DATA;return;}
    for(int i=0;i<3;i++) {
        if(m2[i]/(count-1)>.25f) {s.state=GYRO_MOVED;return;}
        s.bias[i]=mean[i];
    }
    s.state=GYRO_READY;
}
static bool load_file(const char *name,float b[3])
{
    char path[128],extra;int version;
    if(storage_sd_make_path(path,sizeof(path),name)!=ESP_OK) return false;
    FILE *f=fopen(path,"r");if(!f) return false;
    int n=fscanf(f,"GYRO %d %f %f %f %c",&version,&b[0],&b[1],&b[2],&extra);
    bool io_ok=!ferror(f);fclose(f);
    return io_ok && n==4 && version==1 && valid(b);
}
esp_err_t gyro_calibration_load(void)
{
    float b[3];
    if(!load_file("gyro_bias.txt",b) && !load_file("gyro_bias.bak",b)) return ESP_FAIL;
    return orientation_service_set_bias(b)?ESP_OK:ESP_FAIL;
}
esp_err_t gyro_calibration_save(void)
{
    if(s.state!=GYRO_READY && s.state!=GYRO_SAVE_FAILED) return ESP_ERR_INVALID_STATE;
    char path[128],tmp[128],backup[128];
    if(storage_sd_make_path(path,sizeof(path),"gyro_bias.txt")!=ESP_OK ||
       storage_sd_make_path(tmp,sizeof(tmp),"gyro_bias.tmp")!=ESP_OK ||
       storage_sd_make_path(backup,sizeof(backup),"gyro_bias.bak")!=ESP_OK) goto fail;
    FILE *f=fopen(tmp,"w");if(!f) goto fail;
    bool ok=fprintf(f,"GYRO 1 %.9g %.9g %.9g\n",s.bias[0],s.bias[1],s.bias[2])>0;
    if(fflush(f)!=0) ok=false;
    if(fsync(fileno(f))!=0) ok=false;
    if(fclose(f)!=0) ok=false;
    if(!ok) goto fail;
    // FAT rename cannot overwrite a destination. Keep the last valid reference
    // available across the replacement gap and restore it if installation fails.
    float previous[3];
    bool have_previous=load_file("gyro_bias.txt",previous);
    if(have_previous) {
        if(unlink(backup)!=0 && errno!=ENOENT) goto fail;
        if(rename(path,backup)!=0) goto fail;
    } else if(unlink(path)!=0 && errno!=ENOENT) goto fail;
    if(rename(tmp,path)!=0) {
        if(have_previous) (void)rename(backup,path);
        goto fail;
    }
    if(!orientation_service_set_bias(s.bias)) goto fail;
    s.state=GYRO_SAVED;return ESP_OK;
fail:
    s.state=GYRO_SAVE_FAILED;return ESP_FAIL;
}
