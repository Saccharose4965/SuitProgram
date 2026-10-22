#include "message_notifications.h"
#include <stdatomic.h>
#include <string.h>

static _Atomic uint32_t s_arrivals, s_seen;

// Overrides telemetry's optional hook; direct suit delivery calls it too.
void telemetry_file_received(const char *path)
{
    const char prefix[]="/sdcard/messages/inbox/";
    if(!path || strncmp(path,prefix,sizeof(prefix)-1))return;
    const char *name=path+sizeof(prefix)-1;
    size_t n=strlen(name);
    if(n<5 || strchr(name,'/') || strcmp(name+n-4,".wav"))return;
    atomic_fetch_add(&s_arrivals,1);
}
uint32_t message_arrival_snapshot(void) {return atomic_load(&s_arrivals);}
unsigned message_unseen_count(void)
{
    uint32_t seen=atomic_load(&s_seen);
    uint32_t count=atomic_load(&s_arrivals)-seen;
    return count>99?99:count;
}
void message_mark_seen(uint32_t snapshot) {atomic_store(&s_seen,snapshot);}
