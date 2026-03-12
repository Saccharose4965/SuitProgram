#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*logo_skip_check_fn_t)(void *ctx);

void logo_set_skip_check(logo_skip_check_fn_t fn, void *ctx);
void anim_logo(void);

#ifdef __cplusplus
}
#endif
