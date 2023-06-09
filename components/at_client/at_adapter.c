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
#include "at.h"
#include "at_adapter.h"
#include <zephyr/kernel.h>
#include <string.h>

void *rt_realloc(void *ptr, size_t size)
{
    if (ptr)
    {
        rt_free(ptr);
    }

    return rt_malloc(size);
}

rt_err_t rt_sem_control(rt_sem_t sem, int cmd, void *arg)
{
    if ((RT_IPC_CMD_RESET == cmd) && k_sem_count_get(sem))
    {
        k_sem_reset(sem);
    }

    return RT_EOK;
}

rt_mutex_t rt_mutex_create(const char *name, rt_uint8_t flag)
{
    rt_mutex_t ret = NULL;

    ret = rt_malloc(sizeof(struct k_mutex));

    if (ret)
    {
        k_mutex_init(ret);
    }

    return ret;
}

rt_sem_t rt_sem_create(const char *name, rt_uint32_t value, rt_uint8_t flag)
{
    rt_sem_t ret = NULL;

    ret = rt_malloc(sizeof(struct k_sem));

    if (ret)
    {
        k_sem_init(ret, 0, (0 == value) ? (1) : (value));
    }

    return ret;
}

rt_thread_t rt_thread_create(const char *name,
                             void (*entry)(void *),
                             void *parameter,
                             rt_uint32_t stack_size,
                             rt_int32_t priority,
                             rt_uint32_t tick)
{
    rt_thread_t tcb = NULL;
    k_thread_stack_t *stack = NULL;
    
    /* MPU 开启后对栈顶设置写保护，栈基地址必须是 Z_KERNEL_STACK_OBJ_ALIGN 整倍数,否则会造成写保护被错误触发，程序不能正常运行 */
    stack = k_aligned_alloc(Z_KERNEL_STACK_OBJ_ALIGN, Z_KERNEL_STACK_SIZE_ADJUST(stack_size));
    tcb = k_malloc(sizeof(struct k_thread));
    memset(tcb, 0, sizeof(struct k_thread));

    if (tcb && stack)
    {
        k_thread_create(tcb, stack, stack_size, (k_thread_entry_t)entry, parameter, NULL, NULL, priority, 0, K_NO_WAIT);
    }
    else
    {
        rt_free(tcb);
        rt_free(stack);
        tcb = NULL;
    }

    return tcb;
}

void rt_thread_startup(rt_thread_t thread)
{
    k_thread_start(thread);
}

int rt_mutex_take(rt_mutex_t mutex, uint32_t timeout)
{
    return k_mutex_lock(mutex, SYS_TIMEOUT_MS(timeout));
}

int rt_mutex_release(rt_mutex_t mutex)
{
    return k_mutex_unlock(mutex);
}

void rt_mutex_delete(rt_mutex_t mutex)
{
    rt_free(mutex);
}

int rt_sem_take(rt_sem_t sem, uint32_t timeout)
{
    return k_sem_take(sem, SYS_TIMEOUT_MS(timeout));
}

void rt_sem_release(rt_sem_t sem)
{
    k_sem_give(sem);
}

void rt_sem_delete(rt_sem_t sem)
{
    rt_free(sem);
}
