/*
 * Copyright (c) 2022-2022, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-3-26      lihongquan   port to zephyr
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include "zigbee_drv.h"
#include "zigbee_common.h"

#define ZIGBEE_UART_NAME "usart1"
#define ZIGBEE_UART_RX_RING_BUFFER_SIZE 512
#define ZIGBEE_UART_TX_RING_BUFFER_SIZE 512

typedef struct
{
    const struct device *dev;

    uint16_t rx_front;
    uint16_t rx_rear;
    uint8_t rx_ring_buffer[ZIGBEE_UART_RX_RING_BUFFER_SIZE];
    struct k_sem rx_sem;

    uint16_t tx_front;
    uint16_t tx_rear;
    uint8_t tx_ring_buffer[ZIGBEE_UART_TX_RING_BUFFER_SIZE];
    struct k_sem tx_sem;
} zigbee_uart_t;

zigbee_uart_t s_zigbee_uart = {.dev = DEVICE_DT_GET(ZIGBEE_UART_NODE)};

static int zigbee_uart_irq_input(zigbee_uart_t *uart, uint8_t c)
{
    int rear = uart->rx_rear + 1;

    if (rear >= ZIGBEE_UART_RX_RING_BUFFER_SIZE)
    {
        rear = 0;
    }

    if (rear == uart->rx_front)
    {
        /* input was lost */
        return 1;
    }

    uart->rx_ring_buffer[uart->rx_rear] = c;
    uart->rx_rear = rear;
    k_sem_give(&uart->rx_sem);

    return 1;
}

static void zigbee_uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c = 0;
    zigbee_uart_t *uart = user_data;

    uart_irq_update(dev);

    if (uart_irq_rx_ready(dev))
    {
        while (1)
        {
            if (uart_fifo_read(dev, &c, 1) == 0)
            {
                break;
            }
            zigbee_uart_irq_input(uart, c);
        }
    }

    if (uart_irq_tx_ready(dev))
    {
        if (uart->tx_front == uart->tx_rear)
        {
            uart_irq_tx_disable(dev);
        }
        else
        {
            uart_fifo_fill(dev, &uart->tx_ring_buffer[uart->tx_front++], 1);

            if (uart->tx_front >= ZIGBEE_UART_RX_RING_BUFFER_SIZE)
            {
                uart->tx_front = 0U;
            }

            k_sem_give(&uart->tx_sem);
        }
    }
}

void zigbee_uart_init(com_inface_t *p)
{
    com_drv_t *obj = (com_drv_t *)p;
    zigbee_uart_t *uart = (zigbee_uart_t *)obj->user_data;

    if (obj)
    {
        if (!device_is_ready(uart->dev))
        {
            printk("zigbee uart not ready\n");
            return;
        }

        k_sem_init(&uart->rx_sem, 0, K_SEM_MAX_LIMIT);
        k_sem_init(&uart->tx_sem, ZIGBEE_UART_TX_RING_BUFFER_SIZE - 1, K_SEM_MAX_LIMIT);

        uart_irq_callback_user_data_set(uart->dev, zigbee_uart_isr, uart);
        uart_irq_rx_enable(uart->dev);
    }
}

static int zigbee_uart_read_char(zigbee_uart_t *uart, uint32_t timeout)
{
    int ret = 0;
    uint8_t c = 0;
    unsigned int key = 0;

    ret = k_sem_take(&uart->rx_sem, SYS_TIMEOUT_MS(timeout));

    if (ret < 0)
    {
        return ret;
    }

    key = irq_lock();
    c = uart->rx_ring_buffer[uart->rx_front++];

    if (uart->rx_front >= ZIGBEE_UART_RX_RING_BUFFER_SIZE)
    {
        uart->rx_front = 0U;
    }

    irq_unlock(key);

    return c;
}

static int zigbee_uart_read(com_inface_t *p, void *buf, uint32_t length, uint32_t timeout)
{
    uint32_t end_time = k_uptime_get_32() + timeout;
    uint32_t remaining_time = timeout;

    com_drv_t *obj = (com_drv_t *)p;
    zigbee_uart_t *uart = (zigbee_uart_t *)obj->user_data;

    int res = 0;
    int out = 0;
    uint8_t *data = buf;

    while (length && (remaining_time <= timeout))
    {
        res = zigbee_uart_read_char(uart, remaining_time);

        if (res < 0)
        {
            if (out == 0)
            {
                errno = -res;
                return res;
            }

            return out;
        }

        *data++ = (uint8_t)res;
        ++out;
        --length;

        if (timeout != SYS_FOREVER_MS)
        {
            remaining_time = end_time - k_uptime_get_32();
        }
    }

    return out;
}

static int zigbee_uart_write_char(zigbee_uart_t *uart, uint8_t c, uint32_t timeout)
{
    int ret = 0;
    int tx_rear = 0;
    unsigned int key = 0;

    ret = k_sem_take(&uart->tx_sem, SYS_TIMEOUT_MS(timeout));

    if (ret < 0)
    {
        return ret;
    }

    key = irq_lock();
    tx_rear = uart->tx_rear + 1;

    if (tx_rear >= ZIGBEE_UART_TX_RING_BUFFER_SIZE)
    {
        tx_rear = 0;
    }

    if (tx_rear == uart->tx_front)
    {
        irq_unlock(key);
        return -ENOSPC;
    }

    uart->tx_ring_buffer[uart->tx_rear] = c;
    uart->tx_rear = tx_rear;
    irq_unlock(key);
    uart_irq_tx_enable(uart->dev);

    return 0;
}

static int zigbee_uart_write(com_inface_t *p, const void *src, uint32_t size, uint32_t timeout)
{
    uint32_t end_time = k_uptime_get_32() + timeout;
    uint32_t remaining_time = timeout;

    com_drv_t *obj = (com_drv_t *)p;
    zigbee_uart_t *uart = (zigbee_uart_t *)obj->user_data;

    int res = 0;
    int out = 0;
    const uint8_t *data = src;

    while (size && (remaining_time <= timeout))
    {
        res = zigbee_uart_write_char(uart, *data++, remaining_time);

        if (res < 0)
        {
            if (out == 0)
            {
                errno = -res;
                return res;
            }

            return out;
        }

        ++out;
        --size;

        if (timeout != SYS_FOREVER_MS)
        {
            remaining_time = end_time - k_uptime_get_32();
        }
    }

    return out;
}

static void zigbee_uart_flush(com_inface_t *p)
{
    unsigned int key = 0;
    com_drv_t *obj = (com_drv_t *)p;
    zigbee_uart_t *uart = (zigbee_uart_t *)obj->user_data;

    k_sem_reset(&uart->rx_sem);
    key = irq_lock();
    uart->rx_front = 0;
    uart->rx_rear = 0;
    irq_unlock(key);
}

com_drv_t *zigbee_uart_drv_get(void)
{
    static com_drv_t drv = {
        .name = ZIGBEE_UART_NAME,
        .init = zigbee_uart_init,
        .flush = zigbee_uart_flush,
        .read = zigbee_uart_read,
        .write = zigbee_uart_write,
        .user_data = &s_zigbee_uart};

    return &drv;
}
