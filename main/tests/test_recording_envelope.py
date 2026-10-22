from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class EnvelopeTest(unittest.TestCase):
    def test_pcm_windows(self):
        code=r'''
#include "recording_envelope.h"
#include <assert.h>
#include <string.h>
int main(void){
    recording_envelope_t a={.window=800},b={.window=800};
    int16_t pcm[1600]={0};uint8_t out[96];
    pcm[799]=-32768;pcm[1599]=16384;
    recording_envelope_feed(&a,pcm,1600);
    for(unsigned i=0;i<1600;i++)recording_envelope_feed(&b,pcm+i,1);
    assert(!memcmp(&a,&b,sizeof(a))); // DMA chunk boundaries cannot change time scale
    assert(recording_envelope_copy(&a,out,96)==2 && out[0]==255 && out[1]==128);
    assert(recording_envelope_copy(&a,out,1)==1 && out[0]==128);
    a=(recording_envelope_t){.window=1};
    for(int i=0;i<100;i++){int16_t value=i*128;recording_envelope_feed(&a,&value,1);}
    assert(recording_envelope_copy(&a,out,96)==96);
    for(int i=0;i<96;i++)assert(out[i]==i+4);
    a=(recording_envelope_t){.window=800};
    int16_t quiet=0;
    recording_envelope_feed(&a,&quiet,1);assert(a.count==0);
    recording_envelope_push(&a); // final partial window on stop
    assert(recording_envelope_copy(&a,out,96)==1 && out[0]==0);
    uint8_t again[96];assert(recording_envelope_copy(&a,again,96)==1 && again[0]==out[0]);
    assert(recording_envelope_copy(&a,out,0)==0);
}
'''
        with tempfile.TemporaryDirectory(prefix='suit-envelope-') as tmp:
            path=Path(tmp);(path/'test.c').write_text(code)
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            '-I',str(ROOT/'components/microphone/include'),str(path/'test.c'),
                            '-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

if __name__=='__main__':unittest.main()
