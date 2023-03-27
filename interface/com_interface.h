/*
 * Copyright (c) 2022-2022, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-3-26      lihongquan   port to zephyr
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void com_inface_t;

typedef struct
{
    const char *name;
    const void *user_data;
    void (*init)(com_inface_t *obj);
    int (*read)(com_inface_t *obj, void *buf, uint32_t length, uint32_t timeout_ms);
    int (*write)(com_inface_t *obj, const void *src, uint32_t size, uint32_t timeout_ms);
    void (*flush)(com_inface_t *obj);
} com_drv_t;

void com_init(com_inface_t *p);
const char *com_device_name(com_inface_t *p);
int com_read(com_inface_t *p, void *buf, uint32_t length, uint32_t timeout_ms);
int com_write(com_inface_t *p, const void *src, uint32_t size, uint32_t timeout_ms);
void com_flush(com_inface_t *p);
