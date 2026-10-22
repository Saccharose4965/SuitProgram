from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class NotificationsTest(unittest.TestCase):
    def test_arrivals_and_snapshot(self):
        code=r'''
#include "message_notifications.h"
#include <assert.h>
void telemetry_file_received(const char *);
int main(void){
    assert(message_unseen_count()==0);
    telemetry_file_received(0);
    telemetry_file_received("/sdcard/messages/outbox/msg1.wav");
    telemetry_file_received("/sdcard/messages/inbox/x.wav.part");
    telemetry_file_received("/sdcard/messages/inbox/../x.wav");
    assert(message_unseen_count()==0);
    telemetry_file_received("/sdcard/messages/inbox/a.wav");
    assert(message_unseen_count()==1);
    uint32_t before_scan=message_arrival_snapshot();
    telemetry_file_received("/sdcard/messages/inbox/b.wav");
    message_mark_seen(before_scan);
    assert(message_unseen_count()==1); // arrived during scan: keep badge
    for(int i=0;i<110;i++)telemetry_file_received("/sdcard/messages/inbox/c.wav");
    assert(message_unseen_count()==99);
    message_mark_seen(message_arrival_snapshot());
    assert(message_unseen_count()==0);
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-notifications-') as tmp:
            path=Path(tmp)
            (path/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            '-I',str(ROOT/'main'),str(ROOT/'main/message_notifications.c'),
                            str(path/'test.c'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

if __name__=='__main__':unittest.main()
