from pathlib import Path
import subprocess
import tempfile
import unittest


class VoiceResampleTest(unittest.TestCase):
    def test_rates_and_boundaries(self):
        source = r'''
#include <assert.h>
#include <stdlib.h>
#include "voice_resample.h"
static unsigned pos, count;
static int16_t data[16000];
static bool read_sample(int16_t *v) {
    if(pos==count)return false;
    *v=data[pos++];return true;
}
int main(void) {
    for(unsigned rate=44100;rate<=48000;rate+=3900) {
        for(unsigned n=1;n<=16000;n=n==1?17:16000) {
            count=n;pos=0;
            for(unsigned i=0;i<n;i++)data[i]=(i%2)?-32768:32767;
            voice_resample_t s={.input_rate=16000,.output_rate=rate,.remaining=(uint64_t)n*rate/16000};
            unsigned frames=0;int16_t out;
            while(voice_resample_next(&s,read_sample,&out)) {
                if(frames==0)assert(out==data[0]);
                ++frames;
            }
            assert(frames==(uint64_t)n*rate/16000 && pos==n);
            assert(!voice_resample_next(&s,read_sample,&out));
            if(n==16000)break;
        }
    }
    count=2;pos=0;data[0]=0;data[1]=3000;
    voice_resample_t s={.input_rate=16000,.output_rate=48000,.remaining=6};
    int16_t out;int expected[]={0,1000,2000,3000,3000,3000};
    for(int i=0;i<6;i++){assert(voice_resample_next(&s,read_sample,&out));assert(out==expected[i]);}
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(source)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(Path(__file__).resolve().parents[1] / 'include'),
                            str(path / 'test.c'), '-o', str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], check=True)


if __name__ == '__main__':
    unittest.main()
