"""Host checks for the exact oscillator used by Arm Synth."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]

class SynthTest(unittest.TestCase):
    def test_voice(self):
        source = r'''
#include "synth_voice.h"
#include <assert.h>
#include <stdlib.h>
int main(void) {
    synth_voice_t v;
    synth_voice_init(&v);
    int16_t out[24000];
    synth_voice_render(&v,out,24000,220,0);
    for(int i=0;i<24000;i++) assert(out[i]==0);
    synth_voice_render(&v,out,24000,440,1);
    int crossings=0;
    for(int i=12001;i<24000;i++) {
        if(out[i-1]<=0 && out[i]>0) crossings++;
        assert(abs(out[i])<=12000);
    }
    assert(crossings>=219 && crossings<=221);
    // Parameter changes preserve phase and have a smooth envelope.
    int last=out[23999];
    synth_voice_render(&v,out,24000,622.25f,0);
    assert(abs(out[0]-last)<2000);
    assert(v.gain<.0001f && abs(out[23999])<=1);
    assert(isfinite(v.phase) && v.phase>=0 && v.phase<1);
    for(int rate=44100;rate<=48000;rate+=3900) {
        synth_voice_init(&v);v.rate=rate;
        synth_voice_render(&v,out,24000,440,1);
        synth_voice_render(&v,out,24000,440,1);
        crossings=0;
        for(int i=1;i<24000;i++)if(out[i-1]<=0 && out[i]>0)crossings++;
        assert(abs(crossings-(int)(440.0f*24000/rate))<=1);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="suit-synth-") as tmp:
            path = pathlib.Path(tmp)
            (path / "test.c").write_text(source)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-I", str(ROOT / "main"),
                            str(path / "test.c"), "-lm", "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)

if __name__ == "__main__":
    unittest.main()
