/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <spi.h>
#include <uart.h>

#define SPI_DEV  DT_SPI_3_NAME
#define GPIO_DEV DT_GPIO_P1_DEV_NAME

#define SLEEP_TIME 500

#define PIN_A0   8
#define PIN_RES  14

struct device *spi_dev;
struct device *gpio_dev;

struct spi_cs_control cs;

void spi_init(void)
{
    spi_dev = device_get_binding(SPI_DEV);
    if (!spi_dev)
    {
        printk("SPI: Device driver not found.\n");
    }
    else
    {
        printk("SPI device found\n");
    }

    gpio_pin_configure(gpio_dev, PIN_A0, GPIO_DIR_OUT);
    gpio_pin_configure(gpio_dev, PIN_RES, GPIO_DIR_OUT);
    gpio_pin_write(gpio_dev, PIN_RES, 0);
    k_sleep(10);
    gpio_pin_write(gpio_dev, PIN_RES, 1);
    gpio_pin_configure(gpio_dev, 10, GPIO_DIR_OUT);
}



struct spi_cs_control cs = {
    //.gpio_dev = gpio_dev,
    .gpio_pin = 10,
    .delay = 10,
};

struct spi_config spi_cfg_slow = {
    .frequency = 500001,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .slave = 0,
    .cs = &cs,
};

uint8_t buffer_tx[1];
uint8_t buffer_rx[1];
#define BUF_SIZE 1

const struct spi_buf tx_bufs[] = {
    {
        .buf = buffer_tx,
        .len = BUF_SIZE,
    },
};
const struct spi_buf rx_bufs[] = {
    {
        .buf = buffer_rx,
        .len = BUF_SIZE,
    },
};

const struct spi_buf_set tx = {
    .buffers = &tx_bufs[0],
    .count = ARRAY_SIZE(tx_bufs)
};

const struct spi_buf_set rx = {
    .buffers = &rx_bufs[0],
    .count = ARRAY_SIZE(rx_bufs)
};


void lcd_send_byte(uint8_t cmd)
{
    uint8_t ret;
    gpio_pin_write(gpio_dev, PIN_A0, 0);
    buffer_tx[0] = cmd;
    ret = spi_transceive(spi_dev, &spi_cfg_slow, &tx, &rx);
    if (ret != 0)
    {
        printk("SPI transfer failed %d\n", ret);
    }
    gpio_pin_write(gpio_dev, PIN_A0, 0);
}

void lcd_send_data(uint8_t *data, uint32_t size)
{
    uint8_t ret;
    gpio_pin_write(gpio_dev, PIN_A0, 1);
    while (size > 0)
    {
        buffer_tx[0] = *data;
        ret = spi_transceive(spi_dev, &spi_cfg_slow, &tx, &rx);
        if (ret != 0)
        {
            printk("SPI transfer failed %d\n", ret);
        }
        ++data;
        --size;
    }
    gpio_pin_write(gpio_dev, PIN_A0, 0);
}

void sendPattern(int start)
{
    uint8_t bytes[] = { 0x11, 0x22, 0x44, 0x88 };

    lcd_send_data(&bytes[start], (4 - start));
    {
        int i;
        for (i = 0; i < 32; ++i) { lcd_send_data(&bytes[0], 4); }
    }
}

void setLine(int lineNum)
{
    uint8_t cmdBuf[] = { 0xB0,
                         0x10, /* (4) address 0 */
                         0x00, /* (4) address 0 */
    };
    int i;

    cmdBuf[0] |= lineNum;

    for (i = 0; i < sizeof(cmdBuf); ++i)
    {
        lcd_send_byte(cmdBuf[i]);
    }
}

void sendPatterns(int start)
{
    int lineNum;

    for (lineNum = 0; lineNum < 4; ++lineNum)
    {
        setLine(lineNum);
        sendPattern(start);
    }
}

void main(void)
{
    int start;

    gpio_dev = device_get_binding(GPIO_DEV);
    cs.gpio_dev = gpio_dev;
    if (!gpio_dev)
    {
        printk("GPIO: Device driver not found.\n");
    }
    else
    {
        printk("GPIO device found\n");
    }

    k_sleep(1000);

    spi_init();
    uint8_t cmds[] = {
#if 1
                       0xE2, /* internal reset */
#endif
                       0xA0, /* Ram address seg output to normal */
                       0xAF, /* enable display */
                       0xC0, /* common output scan dir */
                       0xA2, /* LCD bias */
                       0x2F, /* Power control set */
                       0x21, /* Resistor ratio */
                       0x81, /* Electronic volume */
                       0x20, /* -//- */
                       /* my commands */
                       0xA6, /* normal display */
                       0xA4, /* normal display */
                       0x40, /* (2) start line 0 */
                       0xB3, /* (3) page 0 to start */
                       0x10, /* (4) address 0 */
                       0x00, /* (4) address 0 */
    };

    int cmdi;
    for (cmdi = 0; cmdi < sizeof(cmds); ++cmdi)
    {
        lcd_send_byte(cmds[cmdi]);
    }

    start = 0;
    printk("Animating...\n");

    while (1)
    {
        sendPatterns(start);
        if (++start == 4) { start = 0; }
        k_sleep(40);
    }

    printk("End.\n\n");
};

