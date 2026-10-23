"""Production calibration state machine and file persistence with fake sensor/time."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class GyroTest(unittest.TestCase):
    def test_session_and_storage(self):
        files=['services/orientation/include/gyro_calibration.h',
               'services/orientation/gyro_calibration.c']
        code='\n'.join(line for name in files for line in (ROOT/name).read_text().splitlines()
                       if not line.startswith(('#include','#pragma once')))
        prefix=r'''
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE -2
typedef struct {float ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps;int64_t t_us;} mpu6500_sample_t;
static int64_t now=1000000;
static bool available=true,sd=true;
static float applied[3]={-1.9f,.5f,0};
static mpu6500_sample_t sensor={.az_g=1,.gx_dps=1,.gy_dps=-2,.gz_dps=.3f};
static int64_t esp_timer_get_time(void){return now;}
static bool orientation_service_read_sample(mpu6500_sample_t *r){*r=sensor;return available;}
static bool orientation_service_set_bias(const float b[3]){memcpy(applied,b,sizeof(applied));return true;}
static esp_err_t storage_sd_make_path(char *p,size_t n,const char *name){if(!sd)return ESP_FAIL;snprintf(p,n,"%s",name);return ESP_OK;}
'''
        test=r'''
static void measure(void) {
    gyro_calibration_begin();now+=2000000;
    for(int i=0;i<=250;i++){sensor.t_us=now;gyro_calibration_tick();now+=20000;}
}
int main(void) {
    measure();assert(s.state==GYRO_READY && fabsf(s.bias[2]-.3f)<1e-5);
    assert(applied[0]==-1.9f); // measurement alone never applies
    sd=false;assert(gyro_calibration_save()==ESP_FAIL && applied[0]==-1.9f);
    sd=true;assert(gyro_calibration_save()==ESP_OK && applied[0]==1);
    applied[0]=9;assert(gyro_calibration_load()==ESP_OK && applied[0]==1);
    // Saving a second reference replaces the existing file.
    sensor.gx_dps=2;measure();assert(gyro_calibration_save()==ESP_OK);
    applied[0]=9;assert(gyro_calibration_load()==ESP_OK && applied[0]==2);
    // The backup restores the previous saved value if the main file is lost.
    assert(unlink("gyro_bias.txt")==0);
    assert(gyro_calibration_load()==ESP_OK && applied[0]==1);
    applied[0]=2;
    assert(unlink("gyro_bias.bak")==0);
    gyro_calibration_begin();gyro_calibration_cancel();now+=9000000;
    gyro_calibration_tick();assert(s.state==GYRO_IDLE && applied[0]==2);
    available=false;measure();assert(s.state==GYRO_NO_DATA && applied[0]==2);available=true;
    sensor.az_g=2;measure();assert(s.state==GYRO_MOVED && applied[0]==2);sensor.az_g=1;
    gyro_calibration_begin();now+=2000000;
    for(int i=0;i<=250;i++){sensor.gx_dps=i%2?2:4;sensor.t_us=now;gyro_calibration_tick();now+=20000;}
    assert(s.state==GYRO_MOVED && applied[0]==2);
    gyro_calibration_begin();now+=2000000;sensor.t_us=now+1;
    gyro_calibration_tick();assert(s.state==GYRO_NO_DATA);
    const char *bad[]={"GYRO 2 1 2 3", "GYRO 1 nan 0 0", "GYRO 1 11 0 0", "GYRO 1 1 2", "GYRO 1 1 2 3 junk"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        FILE *f=fopen("gyro_bias.txt","w");assert(f);fputs(bad[i],f);fclose(f);
        assert(gyro_calibration_load()==ESP_FAIL && applied[0]==2);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-gyro-') as tmp:
            p=Path(tmp);(p/'test.c').write_text(prefix+code+test)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined',str(p/'test.c'),'-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],cwd=tmp,check=True)

if __name__=='__main__':unittest.main()
