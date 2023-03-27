/*
 * Copyright (c) 2022-2022, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-22     lihongquan   port to esp32
 * 2023-3-26      lihongquan   port to zephyr
 */
#ifndef __AT_ADAPTER__
#define __AT_ADAPTER__

#include <assert.h>
#include <stddef.h>
#include <string.h>
#include "at_def.h"
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#ifndef rt_strcmp
#define rt_strcmp                       strcmp
#endif

#ifndef rt_memcpy
#define rt_memcpy                       memcpy
#endif

#ifndef rt_memcmp
#define rt_memcmp                       memcmp
#endif

#ifndef rt_memset
#define rt_memset                       memset
#endif

#ifndef rt_strlen
#define rt_strlen                       strlen
#endif

#ifndef rt_strncmp
#define rt_strncmp                      strncmp
#endif

#ifndef rt_strstr
#define rt_strstr                       strstr
#endif

#ifndef rt_snprintf
#define rt_snprintf                     snprintk
#endif

#ifndef rt_kprintf
#define rt_kprintf                      printk
#endif

#ifndef rt_malloc
#define rt_malloc                       k_malloc
#endif

#ifndef rt_free
#define rt_free                         k_free
#endif

#ifndef rt_calloc
#define rt_calloc                       k_calloc
#endif

#ifndef rt_tick_from_millisecond
#define rt_tick_from_millisecond        
#endif

#ifndef rt_tick_get
#define rt_tick_get                     k_uptime_get_32
#endif

#ifndef LOG_E
#define LOG_E(...)                      LOG_ERR(__VA_ARGS__)
#endif 

#ifndef LOG_W
#define LOG_W(...)                      LOG_WRN(__VA_ARGS__)
#endif 

#ifndef LOG_I
#define LOG_I(...)                      LOG_INF(__VA_ARGS__)
#endif 

#ifndef LOG_D
#define LOG_D(...)                      LOG_DBG(__VA_ARGS__)
#endif 

rt_mutex_t rt_mutex_create (const char *name, rt_uint8_t flag);
int rt_mutex_take(rt_mutex_t mutex, uint32_t timeout);
int rt_mutex_release(rt_mutex_t mutex);
void rt_mutex_delete(rt_mutex_t mutex);

rt_sem_t rt_sem_create(const char *name, rt_uint32_t value, rt_uint8_t flag);
rt_err_t rt_sem_control(rt_sem_t sem, int cmd, void *arg);
int rt_sem_take(rt_sem_t sem, uint32_t timeout);
void rt_sem_release(rt_sem_t sem);
void rt_sem_delete(rt_sem_t sem);

void *rt_realloc(void *ptr, size_t size);
rt_thread_t rt_thread_create(const char *name, void (*entry)(void *),
                             void *parameter, rt_uint32_t stack_size,
                             rt_int32_t priority, rt_uint32_t tick);
void rt_thread_startup(rt_thread_t thread);

#endif
