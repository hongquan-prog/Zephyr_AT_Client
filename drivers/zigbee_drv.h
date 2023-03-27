/*
 * Copyright (c) 2022-2022, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-3-27      lihongquan   port to zephyr
 */

#pragma once

#include "com_interface.h"

typedef enum
{
    ZIGBEE_NRST_PIN,
    ZIGBEE_NETWOK_PIN,
    ZIGBEE_BAUD_RESET_PIN,
    ZIGBEE_PIN_NUM
} zigbee_pin_def;

void zigbee_gpio_init(void);
void zigbee_gpio_set(zigbee_pin_def pin, int level);
com_drv_t *zigbee_uart_drv_get(void);
