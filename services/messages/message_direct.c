#include "message_direct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_random.h"
#include "audio_wav.h"
#include "storage_sd.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>

enum { START=1, DATA=2, END=3, ACK=4, CHUNK=180, MAX_FILE=16*1024*1024 };
typedef struct { uint8_t mac[6]; size_t len; uint8_t wire[192]; } packet_t;
static QueueHandle_t s_rx, s_ack;
static uint32_t value(const uint8_t *p) { uint32_t v; memcpy(&v,p,4); return v; }
static uint32_t hash_bytes(uint32_t h,const uint8_t *p,size_t n)
{ while(n--) h=(h^*p++)*16777619u; return h; }
static void header(uint8_t *p,uint8_t op,uint32_t id,uint32_t v)
{ p[0]='M';p[1]='2';p[2]=op;p[3]=0;memcpy(p+4,&id,4);memcpy(p+8,&v,4); }

void message_direct_receive(link_msg_type_t type,const uint8_t *mac,const uint8_t *data,size_t len)
{
    if(type!=LINK_MSG_MESSAGE || !mac || !data || len<12 || len>192 ||
       data[0]!='M' || data[1]!='2' || !s_rx || !s_ack) return;
    packet_t p={.len=len}; memcpy(p.mac,mac,6);memcpy(p.wire,data,len);
    (void)xQueueSend(data[2]==ACK?s_ack:s_rx,&p,0);
}

static void receiver(void *arg)
{
    (void)arg;
    FILE *file=NULL;
    uint32_t id=0,total=0,received=0,hash=2166136261u,completed=0,completed_size=0;
    uint8_t peer[6]={0},completed_peer[6]={0};
    char partial[136]={0},final[128]={0};
    TickType_t last=0;
    for(;;) {
        packet_t p;
        if(xQueueReceive(s_rx,&p,pdMS_TO_TICKS(500))!=pdTRUE) {
            if(file && xTaskGetTickCount()-last>pdMS_TO_TICKS(10000)) {
                fclose(file);file=NULL;remove(partial);
            }
            continue;
        }
        uint32_t sid=value(p.wire+4),v=value(p.wire+8);
        uint8_t reply[12]; header(reply,ACK,sid,received);
        bool matches=file && sid==id && !memcmp(peer,p.mac,6);
        if(p.wire[2]==START && p.len==12) {
            if(matches) { header(reply,ACK,sid,0); }
            else if(file || !v || v>MAX_FILE || storage_mount_sd()!=ESP_OK) reply[3]=1;
            else {
                mkdir("/sdcard/messages",0775);mkdir("/sdcard/messages/inbox",0775);
                snprintf(final,sizeof(final),"/sdcard/messages/inbox/%02x%02x%02x%02x%02x%02x_%08lx.wav",
                    p.mac[0],p.mac[1],p.mac[2],p.mac[3],p.mac[4],p.mac[5],(unsigned long)sid);
                snprintf(partial,sizeof(partial),"%s.part",final);
                struct stat st;
                if(stat(final,&st)==0) reply[3]=1;
                else file=fopen(partial,"wb");
                if(!file) reply[3]=1;
                else { id=sid;total=v;received=0;hash=2166136261u;memcpy(peer,p.mac,6);header(reply,ACK,sid,0); }
            }
        } else if(matches && p.wire[2]==DATA && p.len>12) {
            size_t n=p.len-12;
            if(v==received && n<=total-received) {
                if(fwrite(p.wire+12,1,n,file)!=n) reply[3]=1;
                else { hash=hash_bytes(hash,p.wire+12,n);received+=n; }
            } else if(v+n!=received) reply[3]=1; // permit only the immediately repeated chunk
            memcpy(reply+8,&received,4);
        } else if(matches && p.wire[2]==END && p.len==12) {
            bool valid=received==total && hash==v;
            if(fflush(file)!=0) valid=false;
            if(fclose(file)!=0) valid=false;
            file=NULL;
            FILE *check=valid?fopen(partial,"rb"):NULL;
            audio_wav_info_t info;
            valid=check && audio_wav_parse_file(check,&info)==ESP_OK && info.data_size>0 &&
                (uint64_t)info.data_offset+info.data_size<=total;
            if(check) fclose(check);
            if(valid && rename(partial,final)==0) {
                completed=id;completed_size=received;memcpy(completed_peer,peer,6);
                telemetry_file_received(final);
            }
            else { reply[3]=1;remove(partial); }
            memcpy(reply+8,&received,4);
        } else if(p.wire[2]==END && sid==completed && !memcmp(p.mac,completed_peer,6)) {
            memcpy(reply+8,&completed_size,4); // final ACK retry after successful rename
        } else continue;
        if(file && sid==id && !memcmp(peer,p.mac,6)) last=xTaskGetTickCount();
        (void)link_send_frame_to(p.mac,LINK_MSG_MESSAGE,reply,sizeof(reply),false);
        if(reply[3] && matches && file) { fclose(file);file=NULL;remove(partial); }
    }
}

esp_err_t message_direct_init(void)
{
    if(s_rx) return ESP_OK;
    s_rx=xQueueCreate(8,sizeof(packet_t));s_ack=xQueueCreate(8,sizeof(packet_t));
    if(!s_rx || !s_ack) goto fail;
    if(xTaskCreatePinnedToCore(receiver,"message_rx",4096,NULL,2,NULL,1)!=pdPASS) goto fail;
    return ESP_OK;
fail:
    if(s_rx) vQueueDelete(s_rx);
    if(s_ack) vQueueDelete(s_ack);
    s_rx=s_ack=NULL;return ESP_ERR_NO_MEM;
}

static esp_err_t exchange(uint8_t *peer,uint8_t *wire,size_t len,uint32_t expected,bool probe)
{
    for(unsigned retry=0;retry<4;retry++) {
        static const uint8_t broadcast[6]={255,255,255,255,255,255};
        esp_err_t err=link_send_frame_to(probe?broadcast:peer,LINK_MSG_MESSAGE,wire,len,false);
        if(err!=ESP_OK) return err;
        TickType_t start=xTaskGetTickCount();
        while(xTaskGetTickCount()-start<pdMS_TO_TICKS(400)) {
            packet_t p;
            if(xQueueReceive(s_ack,&p,pdMS_TO_TICKS(20))!=pdTRUE) continue;
            if(value(p.wire+4)!=value(wire+4) || (!probe && memcmp(p.mac,peer,6))) continue;
            if(p.wire[3]) { if(probe) continue; return ESP_FAIL; }
            if(value(p.wire+8)!=expected) continue;
            if(probe) memcpy(peer,p.mac,6);
            return ESP_OK;
        }
    }
    return probe?ESP_ERR_NOT_FOUND:ESP_ERR_TIMEOUT;
}

esp_err_t message_direct_send(const char *path,telemetry_progress_cb_t cb,void *ctx)
{
    if(link_active_path()!=LINK_PATH_ESPNOW) return ESP_ERR_NOT_FOUND;
    esp_err_t err=message_direct_init();if(err!=ESP_OK)return err;
    struct stat st;if(stat(path,&st) || st.st_size<=0 || st.st_size>MAX_FILE)return ESP_ERR_INVALID_SIZE;
    FILE *f=fopen(path,"rb");if(!f)return ESP_FAIL;
    audio_wav_info_t info;
    err=audio_wav_parse_file(f,&info);
    if(err!=ESP_OK) { fclose(f);return err; }
    rewind(f);
    uint8_t wire[192],peer[6]={0};uint32_t id=esp_random(),sent=0,hash=2166136261u;
    header(wire,START,id,st.st_size);xQueueReset(s_ack);
    err=exchange(peer,wire,12,0,true);
    while(err==ESP_OK && sent<(uint32_t)st.st_size) {
        size_t n=fread(wire+12,1,CHUNK,f);
        if(!n) { err=ESP_FAIL;break; }
        header(wire,DATA,id,sent);hash=hash_bytes(hash,wire+12,n);
        err=exchange(peer,wire,12+n,sent+n,false);
        if(err==ESP_OK) { sent+=n;if(cb)cb(sent,st.st_size,ctx); }
    }
    fclose(f);
    if(err==ESP_OK) { header(wire,END,id,hash);err=exchange(peer,wire,12,sent,false); }
    return err;
}
