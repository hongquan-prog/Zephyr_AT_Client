#pragma once

#define ZIGBEE_NODE DT_CHOSEN(zephyr_zigbee)
#if !DT_NODE_HAS_STATUS(ZIGBEE_NODE, okay)
#error "Unsupported zigbee: please check devicetree and retry"
#endif
#define ZIGBEE_UART_NODE DT_PARENT(ZIGBEE_NODE)
