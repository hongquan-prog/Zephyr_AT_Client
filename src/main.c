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
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "at.h"
#include "zigbee_drv.h"

int zigbee_buad_read(at_response_t resp)
{
	int ret = -1;

	at_resp_set_info(resp, 256, 1, 100);

	if (0 == at_exec_cmd(resp, "AT+BAUD?") && (1 == resp->line_counts))
	{
		at_resp_parse_line_args(resp, 1, "BAUD=%d", &ret);
	}

	return ret;
}

void main(void)
{
	com_init(zigbee_uart_drv_get());

	zigbee_gpio_init();
	zigbee_gpio_set(ZIGBEE_NETWOK_PIN, 1);
	zigbee_gpio_set(ZIGBEE_BAUD_RESET_PIN, 0);
	zigbee_gpio_set(ZIGBEE_NRST_PIN, 0);
	k_msleep(2000);
	at_client_init(zigbee_uart_drv_get(), 256);
	at_response_t resp = at_create_resp(256, 0, 1000);

	while (1)
	{
		printk("module's baud:%d\r\n", zigbee_buad_read(resp));
		k_msleep(1000);
	}
}
