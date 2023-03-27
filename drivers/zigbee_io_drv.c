#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "zigbee_drv.h"
#include "zigbee_common.h"

static const struct gpio_dt_spec s_zigbee_io[ZIGBEE_PIN_NUM] = {
    GPIO_DT_SPEC_GET(ZIGBEE_NODE, nrst_gpios),
    GPIO_DT_SPEC_GET(ZIGBEE_NODE, network_gpios),
    GPIO_DT_SPEC_GET(ZIGBEE_NODE, baud_reset_gpios)};

void zigbee_gpio_init(void)
{
    for (int i = 0; i < ZIGBEE_PIN_NUM; i++)
    {
        if (!gpio_is_ready_dt(&s_zigbee_io[i]))
        {
            printk("%s is not ready\n", s_zigbee_io[i].port->name);
            return;
        }

        if (gpio_pin_configure_dt(&s_zigbee_io[i], GPIO_OUTPUT) < 0)
        {
            printk("%s config failed\n", s_zigbee_io[i].port->name);
        }
    }
}

void zigbee_gpio_set(zigbee_pin_def pin, int level)
{
    if (pin < ZIGBEE_PIN_NUM)
    {
        gpio_pin_set_dt(&s_zigbee_io[pin], level);
    }
}
