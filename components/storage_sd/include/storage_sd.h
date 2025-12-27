#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mount / unmount the SD card (SPI mode via SDSPI host)
esp_err_t   storage_mount_sd(void);
void        storage_unmount_sd(void);

// Introspection / path helpers
bool        storage_sd_is_mounted(void);
const char* storage_sd_mount_point(void);
esp_err_t   storage_sd_make_path(char* out, size_t outsz, const char* rel);

// Simple atomic boot counter kept on the SD root
esp_err_t storage_boot_counter_increment(uint32_t *out_count);

// Optional: list a directory (pass NULL for root)
esp_err_t   storage_sd_list_dir(const char* rel_or_null);

#ifdef __cplusplus
}
#endif
