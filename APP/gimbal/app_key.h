//
// Created by guan on 2026/4/1.
//

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool prev[4];
} app_key_toggle_ctx_t;

static inline void app_key_toggle_init(app_key_toggle_ctx_t *ctx) {
    for(size_t i = 0; i < 4; i++) {
        ctx->prev[i] = false;
    }
}

/* 单路：raw 上升沿（0->1）时翻转 flag */
static inline void app_key_toggle_one(bool *flag, bool raw, bool *prev) {
    if(raw && !*prev) {
        *flag = !*flag;
    }
    *prev = raw;
}

/* 四路：与 key1..key4 顺序一致 */
static inline void app_key_toggle_4(bool flags[4], const bool raw[4], app_key_toggle_ctx_t *ctx) {
    for(size_t i = 0; i < 4; i++) {
        app_key_toggle_one(&flags[i], raw[i], &ctx->prev[i]);
    }
}

#ifdef __cplusplus
}
#endif
