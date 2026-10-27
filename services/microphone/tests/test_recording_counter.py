"""Host regression for recoverable automatic recording numbers."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


class RecordingCounterTest(unittest.TestCase):
    def test_recovery(self):
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "recording_counter.h"

static const char *root;
int storage_mount_sd(void) { return ESP_OK; }
int storage_sd_make_path(char *out, size_t size, const char *relative) {
    int n = snprintf(out, size, "%s/%s", root, relative);
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_NO_MEM;
}
static void path(char *out, const char *suffix) {
    snprintf(out, 512, "%s/.rec_counter%s", root, suffix);
}
static void write_raw(const char *suffix, const void *data, size_t size) {
    char name[512]; path(name, suffix);
    FILE *file = fopen(name, "wb"); assert(file);
    assert(fwrite(data, 1, size, file) == size);
    assert(fclose(file) == 0);
}
static void clear_all(void) {
    char name[512];
    const char *suffixes[] = {"", ".bak", ".tmp"};
    for (unsigned i=0; i<3; ++i) { path(name, suffixes[i]); unlink(name); }
}
static uint32_t next(void) {
    uint32_t value=999; assert(recording_counter_next(&value)==ESP_OK); return value;
}
int main(int argc, char **argv) {
    assert(argc==2); root=argv[1];
    assert(next()==0); assert(next()==1);

    clear_all();
    write_raw("", "x", 1);
    uint32_t nine=9; write_raw(".bak", &nine, sizeof(nine));
    assert(next()==10); assert(next()==11);

    clear_all();
    uint32_t ten=10, twenty=20;
    write_raw("", &ten, sizeof(ten));
    write_raw(".tmp", &twenty, sizeof(twenty));
    assert(next()==21);

    clear_all();
    write_raw("", "bad", 3); write_raw(".bak", "also bad", 8);
    assert(next()==0); assert(next()==1);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="suit-counter-") as tmp:
            temp = Path(tmp)
            include = temp / "include"
            include.mkdir()
            (include / "esp_err.h").write_text(
                "typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_FAIL 0x101\n"
                "#define ESP_ERR_NO_MEM 0x102\n#define ESP_ERR_INVALID_ARG 0x103\n"
                "#define ESP_ERR_INVALID_STATE 0x104\n"
                "#define ESP_ERR_INVALID_RESPONSE 0x105\n"
            )
            (include / "esp_check.h").write_text(
                "#define ESP_RETURN_ON_ERROR(x,t,m) do{(void)(t);(void)(m);int e=(x);if(e)return e;}while(0)\n"
            )
            (include / "esp_log.h").write_text(
                "#define ESP_LOGE(tag,...) ((void)(tag))\n#define ESP_LOGW(tag,...) ((void)(tag))\n"
            )
            (include / "storage_sd.h").write_text(
                "#include <stddef.h>\nint storage_mount_sd(void);\n"
                "int storage_sd_make_path(char *,size_t,const char *);\n"
            )
            source = temp / "test.c"
            source.write_text(harness)
            binary = temp / "test"
            subprocess.run([
                "gcc", "-std=c11", "-D_POSIX_C_SOURCE=200809L", "-Wall", "-Wextra",
                "-Werror", "-fsanitize=address,undefined", f"-I{include}",
                f"-I{ROOT / 'services/microphone'}",
                str(ROOT / "services/microphone/recording_counter.c"), str(source),
                "-o", str(binary),
            ], check=True)
            card = temp / "card"; card.mkdir()
            subprocess.run([str(binary), str(card)], check=True)


if __name__ == "__main__":
    unittest.main()
