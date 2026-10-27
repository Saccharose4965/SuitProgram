"""Absolute filter output and caller-owned heading reference regression."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class ReferenceTest(unittest.TestCase):
    def test_heading_reference(self):
        source = r'''
#include "orientation_reference.h"
#include <assert.h>
int main(void) {
    imu_orientation_t filter;
    imu_orientation_init(&filter,.05f);
    filter.gyro_bias_dps[0]=1;filter.gyro_bias_dps[1]=-2;filter.gyro_bias_dps[2]=.3f;
    for(int i=0;i<100;i++) imu_orientation_update(&filter,0,0,1,1,-2,.3f,1000000+i*5000);
    assert(fabsf(filter.q_BW.w-1)<1e-6f && fabsf(filter.q_BW.z)<1e-6f);
    filter.ready=true; filter.have_sample=true;
    orientation_reference_t reference;
    orientation_reference_reset(&reference);
    // q = yaw * pitch * finger-axis roll; only yaw belongs in reference.
    for (int i=0;i<200;i++) {
        float a=i*.013f, b=.12f, c=i*.019f;
        imu_quat_t q={cosf(a)*cosf(b)*cosf(c)-sinf(a)*sinf(b)*sinf(c),
            cosf(a)*sinf(b)*cosf(c)-sinf(a)*cosf(b)*sinf(c),
            cosf(a)*cosf(b)*sinf(c)+sinf(a)*sinf(b)*cosf(c),
            cosf(a)*sinf(b)*sinf(c)+sinf(a)*cosf(b)*cosf(c)};
        filter.q_BW=q;
        assert(orientation_reference_calibrate(&reference,q));
        assert(reference.heading.x==0 && reference.heading.y==0);
        // Quaternion signs may differ, rotations must match.
        float dot=reference.heading.w*cosf(a)+reference.heading.z*sinf(a);
        assert(fabsf(fabsf(dot)-1)<1e-5f);
        assert(filter.q_BW.w==q.w && filter.q_BW.x==q.x);
        imu_quat_t relative;
        assert(orientation_reference_relative(&reference,q,&relative));
        assert(isfinite(relative.w));
        imu_quat_t saved=reference.heading;
        assert(!orientation_reference_calibrate(&reference,(imu_quat_t){NAN,0,0,0}));
        assert(reference.heading.w==saved.w && reference.heading.z==saved.z);
        assert(!orientation_reference_calibrate(&reference,(imu_quat_t){.70710678f,.70710678f,0,0}));
        assert(reference.heading.w==saved.w && reference.heading.z==saved.z);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="suit-reference-") as tmp:
            p = Path(tmp)
            (p / "test.c").write_text(source)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined",
                            "-I", str(ROOT / "services/orientation/include"),
                            str(p / "test.c"), str(ROOT / "services/orientation/orientation.c"),
                            "-lm", "-o", str(p / "test")], check=True)
            subprocess.run([str(p / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
