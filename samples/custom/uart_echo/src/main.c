/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <uart.h>

#define UARTE_DEV DT_UART_0_NAME
struct device *uarte_dev;

void main(void)
{
    int i;
    const int buf_len = 256;
    u8_t buf[buf_len];
    uint8_t c;

    uarte_dev = device_get_binding(UARTE_DEV);
    if (!uarte_dev)
    {
        printk("UARTE: not found\n");
    }
    else
    {
        printk("UARTE: found device\n");
    }

    c = '.';
    i = 0;
    printk("Type something and press <Enter>\n");
    while (1)
    {
        k_sleep(40);
        if (uart_poll_in(uarte_dev, &c) == 0)
        {
            if (c == 13)
            {
                buf[i] = '\0';
                printk("Buffer: %s\n", buf);
                i = 0;
            }
            else if (i == buf_len)
            {
                buf[i] = '\0';
                printk("\nBuffer: %s\n", buf);
                buf[i = 0] = c;
            }
            else
            {
                buf[i++] = c;
            }
        }
    }

    printk("End.\n\n");
};

