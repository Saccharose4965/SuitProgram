"""Host regression for strict and recoverable score persistence."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


class ScoresTest(unittest.TestCase):
    def test_persistence_and_recovery(self):
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "scores.h"

static const char *root;

int storage_mount_sd(void) { return ESP_OK; }
int storage_sd_make_path(char *out, size_t size, const char *relative) {
    int n = snprintf(out, size, "%s/%s", root, relative);
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_NO_MEM;
}

static void write_text(const char *name, const char *text) {
    char path[512];
    snprintf(path, sizeof(path), "%s/scores/%s", root, name);
    FILE *file = fopen(path, "wb");
    assert(file);
    assert(fwrite(text, 1, strlen(text), file) == strlen(text));
    assert(fclose(file) == 0);
}

static uint32_t load(const char *key) {
    uint32_t value = 999;
    assert(scores_load_high(key, &value) == ESP_OK);
    return value;
}

int main(int argc, char **argv) {
    assert(argc == 2);
    root = argv[1];

    assert(load("missing") == 0);
    assert(scores_load_high("../escape", &(uint32_t){0}) == ESP_ERR_INVALID_ARG);

    uint32_t resulting = 0;
    assert(scores_update_high("snake", 12, &resulting) == ESP_OK && resulting == 12);
    assert(scores_update_high("snake", 7, &resulting) == ESP_OK && resulting == 12);
    assert(load("snake") == 12);

    write_text("backup.txt", "broken\n");
    write_text("backup.txt.bak", "31\n");
    assert(load("backup") == 31);
    assert(load("backup") == 31);

    write_text("interrupted.txt", "8\n");
    write_text("interrupted.txt.tmp", "19\n");
    assert(load("interrupted") == 19);
    assert(load("interrupted") == 19);

    write_text("junk.txt", "14 points\n");
    assert(scores_load_high("junk", &resulting) == ESP_ERR_INVALID_RESPONSE);
    assert(scores_update_high("junk", 20, &resulting) == ESP_ERR_INVALID_RESPONSE);

    write_text("overflow.txt", "4294967296\n");
    assert(scores_load_high("overflow", &resulting) == ESP_ERR_INVALID_RESPONSE);
    write_text("negative.txt", "-1\n");
    assert(scores_load_high("negative", &resulting) == ESP_ERR_INVALID_RESPONSE);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="suit-scores-") as tmp:
            tmp_path = Path(tmp)
            include = tmp_path / "include"
            include.mkdir()
            (include / "esp_err.h").write_text(
                "typedef int esp_err_t;\n"
                "#define ESP_OK 0\n#define ESP_FAIL 0x101\n"
                "#define ESP_ERR_NO_MEM 0x102\n#define ESP_ERR_INVALID_ARG 0x103\n"
                "#define ESP_ERR_INVALID_STATE 0x104\n"
                "#define ESP_ERR_INVALID_RESPONSE 0x105\n"
            )
            (include / "esp_check.h").write_text(
                "#define ESP_RETURN_ON_ERROR(x,t,m) do { (void)(t); (void)(m); "
                "int e=(x); if(e) return e; } while(0)\n"
            )
            (include / "esp_log.h").write_text(
                "#define ESP_LOGE(tag,...) ((void)(tag))\n"
                "#define ESP_LOGW(tag,...) ((void)(tag))\n"
            )
            (include / "storage_sd.h").write_text(
                "#include <stddef.h>\nint storage_mount_sd(void);\n"
                "int storage_sd_make_path(char *, size_t, const char *);\n"
            )
            harness_path = tmp_path / "test.c"
            harness_path.write_text(harness)
            binary = tmp_path / "test"
            subprocess.run(
                [
                    "gcc", "-std=c11", "-D_POSIX_C_SOURCE=200809L",
                    "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
                    f"-I{include}", f"-I{ROOT / 'services/scores/include'}",
                    str(ROOT / "services/scores/scores.c"), str(harness_path),
                    "-o", str(binary),
                ],
                check=True,
            )
            card = tmp_path / "card"
            card.mkdir()
            subprocess.run([str(binary), str(card)], check=True)


if __name__ == "__main__":
    unittest.main()
